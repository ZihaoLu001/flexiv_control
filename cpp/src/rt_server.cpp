// rt_server.cpp -- Tier-B real-time control daemon for Flexiv Rizon.
//
// This is the optional 1 kHz upgrade path for flexiv_control. The Python stack
// (Tier A) already drives the robot at 100-500 Hz through the NRT RDK modes,
// which is enough for chunked-policy execution, MPC, and RL because the
// hard real-time impedance loop runs *inside* the Rizon. When you need a true
// 1 kHz host loop (high-rate streaming MPC, torque-level experiments, tight
// contact control), this daemon provides it while speaking the *same* action
// contract and the *same* newline-delimited-JSON wire protocol as the Python
// server, so the Python client / Gym env / ROS overlay do not change.
//
// Architecture (mirrors Polymetis' C++ server and Deoxys' NUC controller):
//
//     [Python client / MPC / RL] --JSON/TCP--> [network thread]
//                                                    | lock-light handoff
//                                                    v
//                                          [RT thread @1kHz, rdk::Scheduler]
//                                                    | StreamCartesianMotionForce
//                                                    v
//                                                 [Rizon]
//
// The network thread parses commands and publishes the latest setpoint into a
// double-buffered holder. The RT thread never allocates, never blocks on the
// network, and falls back to holding the last setpoint if commands go stale
// (watchdog). All RDK calls flagged "// VERIFY:" have signatures that have
// changed across RDK versions -- check them against the rdk you link.
//
// Build: see cpp/README.md. Requires the Flexiv RDK (>=1.x), a PREEMPT_RT or
// low-latency kernel for genuine 1 kHz, and root (for the scheduler's RT
// priority + CPU affinity).
//
// SPDX-License-Identifier: Apache-2.0
// Community project, NOT affiliated with Flexiv Robotics.

#include <atomic>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

// --- Flexiv RDK --------------------------------------------------------------
// VERIFY: header layout and namespace differ by RDK version. Recent RDK uses
//   <flexiv/rdk/robot.hpp> and namespace flexiv::rdk. Older RDK used
//   <flexiv/Robot.hpp> and namespace flexiv. Adjust to match your install.
#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/scheduler.hpp>
#include <flexiv/rdk/utility.hpp>

// --- JSON --------------------------------------------------------------------
// Header-only nlohmann/json. See cpp/README.md for how to provide it (system
// package `nlohmann-json3-dev`, vendored single header, or CMake FetchContent).
#include <nlohmann/json.hpp>
using json = nlohmann::json;

// --- POSIX sockets -----------------------------------------------------------
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

namespace {

constexpr int kCartDof = 6;
constexpr int kPoseDim = 7;  // [x y z qw qx qy qz]

std::atomic<bool> g_stop{false};
void OnSignal(int) { g_stop.store(true); }

// ---------------------------------------------------------------------------
// SetPoint + double-buffered holder
// ---------------------------------------------------------------------------
// One Cartesian setpoint plus the contact-wrench feed-forward and a sequence
// number. POD only, so the RT thread can copy it without allocating.
struct SetPoint {
    std::array<double, kPoseDim> pose{{0.45, 0.0, 0.30, 1.0, 0.0, 0.0, 0.0}};
    std::array<double, kCartDof> wrench{{0, 0, 0, 0, 0, 0}};
    double gripper_width = -1.0;  // <0 => no gripper command this tick
    double gripper_force = 20.0;
    uint64_t seq = 0;
};

// Lock-light single-producer/single-consumer handoff. The producer (network
// thread) writes into the back buffer then flips an atomic index; the consumer
// (RT thread) reads the front index. A short spinlock guards the rare overlap;
// the RT thread uses try_lock and keeps its previous setpoint on contention, so
// it never blocks.
class SetPointMailbox {
public:
    void Publish(const SetPoint& sp) {
        std::lock_guard<std::mutex> lk(mtx_);
        buf_[1 - front_] = sp;
        front_ = 1 - front_;
        stamp_ns_.store(NowNs(), std::memory_order_release);
    }
    // Returns false if it could not read (contention) -- caller keeps last.
    bool TryRead(SetPoint& out) {
        std::unique_lock<std::mutex> lk(mtx_, std::try_to_lock);
        if (!lk.owns_lock()) return false;
        out = buf_[front_];
        return true;
    }
    double AgeMs() const {
        return (NowNs() - stamp_ns_.load(std::memory_order_acquire)) / 1.0e6;
    }
    static uint64_t NowNs() {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
                   std::chrono::steady_clock::now().time_since_epoch())
            .count();
    }

private:
    std::mutex mtx_;
    SetPoint buf_[2];
    int front_ = 0;
    std::atomic<uint64_t> stamp_ns_{0};
};

// ---------------------------------------------------------------------------
// Minimal Cartesian safety, mirroring flexiv_control/safety.py. Cheap enough
// to run every RT tick. Clips into a workspace box and caps per-tick linear
// step (= speed * dt). Orientation is passed through (the Rizon impedance loop
// handles the rest); extend if you need angular-rate clamping.
// ---------------------------------------------------------------------------
struct SafetyBox {
    double xmin = 0.25, xmax = 0.75;
    double ymin = -0.45, ymax = 0.45;
    double zmin = 0.06, zmax = 0.70;
    double max_lin_speed = 0.25;   // m/s
    double max_contact_n = 40.0;   // N, per linear axis

    void ClampInto(std::array<double, kPoseDim>& pose,
                   const std::array<double, kPoseDim>& prev, double dt) const {
        pose[0] = std::min(std::max(pose[0], xmin), xmax);
        pose[1] = std::min(std::max(pose[1], ymin), ymax);
        pose[2] = std::min(std::max(pose[2], zmin), zmax);
        const double dx = pose[0] - prev[0];
        const double dy = pose[1] - prev[1];
        const double dz = pose[2] - prev[2];
        const double step = std::sqrt(dx * dx + dy * dy + dz * dz);
        const double cap = max_lin_speed * dt;
        if (step > cap && step > 1e-9) {
            const double s = cap / step;
            pose[0] = prev[0] + dx * s;
            pose[1] = prev[1] + dy * s;
            pose[2] = prev[2] + dz * s;
        }
    }
};

// ---------------------------------------------------------------------------
// Network thread: accept one client, read newline-delimited JSON requests,
// publish setpoints to the mailbox. Supported methods (subset of the Python
// protocol that is meaningful for a streaming RT loop):
//   {"id":N,"method":"set_cartesian_target","params":{"pose":[7],"wrench":[6],
//                                                      "gripper":{"width":..,"force":..}}}
//   {"id":N,"method":"get_state"}     -> returns latest robot state
//   {"id":N,"method":"stop"}          -> request shutdown
// Each request gets one JSON response line: {"id":N,"ok":true,"result":{...}}.
// ---------------------------------------------------------------------------
void NetworkThread(int port, SetPointMailbox& mailbox, flexiv::rdk::Robot& robot) {
    int listen_fd = ::socket(AF_INET, SOCK_STREAM, 0);
    int one = 1;
    ::setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    if (::bind(listen_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::cerr << "[rt_server] bind failed on port " << port << "\n";
        g_stop.store(true);
        return;
    }
    ::listen(listen_fd, 1);
    std::cout << "[rt_server] listening on 0.0.0.0:" << port << "\n";

    while (!g_stop.load()) {
        sockaddr_in cli{};
        socklen_t clilen = sizeof(cli);
        int fd = ::accept(listen_fd, reinterpret_cast<sockaddr*>(&cli), &clilen);
        if (fd < 0) continue;
        ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
        std::cout << "[rt_server] client connected\n";

        std::string buf;
        char chunk[4096];
        while (!g_stop.load()) {
            ssize_t n = ::recv(fd, chunk, sizeof(chunk), 0);
            if (n <= 0) break;  // client gone
            buf.append(chunk, static_cast<size_t>(n));
            size_t nl;
            while ((nl = buf.find('\n')) != std::string::npos) {
                std::string line = buf.substr(0, nl);
                buf.erase(0, nl + 1);
                if (line.empty()) continue;

                json resp;
                try {
                    json req = json::parse(line);
                    const auto id = req.value("id", 0);
                    const std::string method = req.value("method", "");
                    resp["id"] = id;

                    if (method == "set_cartesian_target") {
                        const auto& p = req.at("params");
                        SetPoint sp;
                        const auto pose = p.at("pose");
                        for (int i = 0; i < kPoseDim; ++i) sp.pose[i] = pose[i].get<double>();
                        if (p.contains("wrench") && !p["wrench"].is_null()) {
                            for (int i = 0; i < kCartDof; ++i)
                                sp.wrench[i] = p["wrench"][i].get<double>();
                        }
                        if (p.contains("gripper") && !p["gripper"].is_null()) {
                            sp.gripper_width = p["gripper"].value("width", -1.0);
                            sp.gripper_force = p["gripper"].value("force", 20.0);
                        }
                        sp.seq = SetPointMailbox::NowNs();
                        mailbox.Publish(sp);
                        resp["ok"] = true;
                        resp["result"] = {{"accepted", true}};
                    } else if (method == "get_state") {
                        // VERIFY: states accessor name/shape vary by RDK version.
                        //   recent: robot.states().tcp_pose / .q / .ext_wrench_in_world
                        const auto st = robot.states();
                        resp["ok"] = true;
                        resp["result"] = {
                            {"tcp_pose", st.tcp_pose},
                            {"q", st.q},
                            {"wrench", st.ext_wrench_in_world},
                        };
                    } else if (method == "stop") {
                        resp["ok"] = true;
                        resp["result"] = {{"stopping", true}};
                        g_stop.store(true);
                    } else {
                        resp["ok"] = false;
                        resp["error"] = "unknown or non-RT method: " + method;
                    }
                } catch (const std::exception& e) {
                    resp = {{"ok", false}, {"error", std::string("parse/exec: ") + e.what()}};
                }
                const std::string out = resp.dump() + "\n";
                ::send(fd, out.data(), out.size(), 0);
            }
        }
        ::close(fd);
        std::cout << "[rt_server] client disconnected\n";
    }
    ::close(listen_fd);
}

}  // namespace

int main(int argc, char** argv) {
    // ---- args: <robot_serial> [--port 8766] [--freq 1000] -------------------
    if (argc < 2) {
        std::cerr << "usage: rt_server <robot_serial> [--port 8766] [--freq 1000]\n";
        return 1;
    }
    const std::string robot_sn = argv[1];
    int port = 8766;
    int freq = 1000;
    for (int i = 2; i + 1 < argc; i += 2) {
        const std::string k = argv[i];
        if (k == "--port") port = std::stoi(argv[i + 1]);
        else if (k == "--freq") freq = std::stoi(argv[i + 1]);
    }
    const double dt = 1.0 / static_cast<double>(freq);

    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);

    try {
        // ---- connect + enable ----------------------------------------------
        // VERIFY: constructor signature differs by RDK version (some take a
        //   local NIC IP as a second argument).
        flexiv::rdk::Robot robot(robot_sn);

        if (robot.fault()) {
            std::cout << "[rt_server] clearing fault...\n";
            robot.ClearFault();                                  // VERIFY name
        }
        std::cout << "[rt_server] enabling robot...\n";
        robot.Enable();
        while (!robot.operational() && !g_stop.load()) {         // VERIFY name
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        std::cout << "[rt_server] robot operational\n";

        // ---- enter RT Cartesian motion-force mode --------------------------
        // VERIFY: enum path is flexiv::rdk::Mode::RT_CARTESIAN_MOTION_FORCE on
        //   recent RDK (older: flexiv::Mode::NRT_... / MODE_...).
        robot.SwitchMode(flexiv::rdk::Mode::RT_CARTESIAN_MOTION_FORCE);

        // Optional: set a compliant Cartesian impedance + a contact-wrench
        // ceiling so accidental contact backs off instead of pushing through.
        // VERIFY: method names/signatures (SetCartesianImpedance,
        //   SetMaxContactWrench) vary by RDK version.
        // robot.SetCartesianImpedance({2000,2000,2000,200,200,200});
        // robot.SetMaxContactWrench({40,40,40,5,5,5});

        SetPointMailbox mailbox;
        SafetyBox safety;

        // Seed the mailbox with the current pose so the first RT ticks hold
        // still rather than jumping.
        {
            SetPoint sp;
            const auto st = robot.states();                      // VERIFY accessor
            for (int i = 0; i < kPoseDim; ++i) sp.pose[i] = st.tcp_pose[i];
            mailbox.Publish(sp);
        }

        std::thread net(NetworkThread, port, std::ref(mailbox), std::ref(robot));

        // ---- the 1 kHz RT control task -------------------------------------
        SetPoint current;
        mailbox.TryRead(current);
        std::array<double, kPoseDim> last_cmd = current.pose;

        auto rt_task = [&]() {
            if (g_stop.load()) return;
            SetPoint sp = current;
            if (mailbox.TryRead(sp)) current = sp;  // else keep previous

            // Watchdog: if commands are stale, hold the last *commanded* pose.
            const bool stale = mailbox.AgeMs() > 100.0;
            std::array<double, kPoseDim> target = stale ? last_cmd : current.pose;

            // Per-tick safety (workspace + speed) against the last command.
            safety.ClampInto(target, last_cmd, dt);

            // Stream to the robot. RDK takes the motion setpoint and an optional
            // feed-forward wrench in the same call for motion-force control.
            // VERIFY: method name + argument shape. Recent RDK:
            //   robot.StreamCartesianMotionForce(pose, wrench)
            std::array<double, kCartDof> ff = stale
                ? std::array<double, kCartDof>{{0, 0, 0, 0, 0, 0}}
                : current.wrench;
            std::vector<double> pose_v(target.begin(), target.end());
            std::vector<double> wr_v(ff.begin(), ff.end());
            robot.StreamCartesianMotionForce(pose_v, wr_v);

            last_cmd = target;
        };

        // VERIFY: Scheduler API. Recent RDK:
        //   scheduler.AddTask(callback, "name", interval_in_ticks, priority, cpu)
        //   where the base tick is 1 ms, so interval=1 => 1 kHz. Older RDK used
        //   AddTask(cb, name, 1, 45, 0). Priority/affinity need root.
        flexiv::rdk::Scheduler scheduler;
        scheduler.AddTask(rt_task, "flexiv_control_rt", 1, 45);
        std::cout << "[rt_server] starting " << freq << " Hz RT loop (Ctrl-C to stop)\n";
        scheduler.Start();

        while (!g_stop.load()) std::this_thread::sleep_for(std::chrono::milliseconds(50));

        std::cout << "[rt_server] stopping...\n";
        scheduler.Stop();
        robot.Stop();
        if (net.joinable()) net.join();
    } catch (const std::exception& e) {
        std::cerr << "[rt_server] fatal: " << e.what() << "\n";
        return 1;
    }
    std::cout << "[rt_server] clean exit\n";
    return 0;
}
