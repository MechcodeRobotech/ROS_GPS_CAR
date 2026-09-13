# Audit evidence

All probes run locally with fake hardware/ROS objects. Do not run them against a ROS topic or robot endpoint.

From the project root:

```bash
python3 gpt_found_bug_evidence/reproduce.py

g++ -std=c++17 -DMOCK_ARDUINO -DUNIT_TEST \
  -I firmware/test/mock -I firmware/config -I firmware/lib/PIDF \
  -I firmware/src/estimator -I firmware/src/control -I firmware/src/safety \
  gpt_found_bug_evidence/reproduce.cpp firmware/test/mock/mock_arduino.cpp \
  firmware/lib/PIDF/PIDF.cpp -o /tmp/gps-audit-reproduce
/tmp/gps-audit-reproduce
```

The Python probe resolves ROOT relative to its own location. It extracts unmodified production classes with AST and supplies local message/publisher stubs. The configuration reproduction uses a temporary directory that is removed afterwards. These are diagnostic reproductions, not additions to the project's test suite. Logs record the original audit run, with the personal home path normalized to `${USER_HOME}` for portable publication; results and timestamps are unchanged.
