# Security Policy

`flexiv_control` commands physical robot hardware, so safety and security issues
are taken seriously.

## Reporting a vulnerability

Please report security vulnerabilities **privately** — do not open a public issue
for a security problem. Use GitHub's private vulnerability reporting:

> Repository → **Security** tab → **Report a vulnerability**
> (https://github.com/ZihaoLu001/flexiv_control/security/advisories/new)

Include the affected version/commit, a description, and a reproduction if
possible. You'll get an acknowledgement and a fix or mitigation timeline.

## Supported versions

This project is in alpha (0.x). Security fixes land on `main` and the latest
release; please track the latest version.

## Safety note (read before running on hardware)

This is **alpha, hardware-unvalidated** software. Before any motion on a real
robot: keep an E-stop within reach, validate your `SafetyProfile` — especially
the workspace box — on your own cell at low speed, and review the
`# VERIFY:` markers in the RDK backend against your installed RDK version
(see `docs/safety.md` and `docs/versions.md`).
