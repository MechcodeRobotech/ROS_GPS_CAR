# Documentation map

Start with the installation route matching your machine and connection.

| Need | Guide |
|---|---|
| Choose native Linux, Docker, Windows Wi-Fi, or Windows USB | [Docker installation matrix](docker.md#choose-an-installation) |
| Install ROS 2 and run directly on Ubuntu | [Native installation](install.md), [setup launcher](../setup/README.md) |
| Docker installation, USB attachment, flashing, start/stop, backup | [Docker guide](docker.md) |
| Current Docker test evidence and untested hardware paths | [Verification record](docker_verification.md) |
| Windows acceptance and agent continuation | [Windows handoff](windows_handoff.md) |
| Everyday operation and command examples | [Running](running.md) |
| UI tabs, configuration, waypoint controls | [Web UI](web_ui.md) |
| Wiring, pin assignments, power, sensors | [Hardware](hardware.md) |
| Components and data flow | [Architecture](architecture.md) |
| Message types, units, topics, HTTP endpoints | [Topics and API](topics.md) |
| Interlocks and attended tests | [Safety](safety.md), [acceptance](acceptance.md) |
| Calibration and control parameters | [Calibration](calibration.md), [tuning](tuning.md) |
| Offline viewer and executable build | [Viewer README](../webapp/README.md) |
| Source state, current work, next steps for Gemini | [HANDOFF](../HANDOFF.md) |

`main` contains editable sources. `workspace`, `firmware`, and `docker` are
mechanically generated subsets. An ignored local viewer executable, CAD export,
Wi-Fi secret, or build cache is not part of a fresh clone. Follow the build
instructions rather than assuming those development-machine files are present.
