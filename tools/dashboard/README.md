# Telemetry dashboard (Buddy 1)

Laptop-side tool for the car's MQTT telemetry. Python 3.8+.

| File | What it is |
|---|---|
| `tel_codec.py` | Frame decoder/encoder, a mirror of `src/common/data_repr.h` (standard library only) |
| `tel_dashboard.py` | CLI: `listen`, `send`, `decode-log`, `decode`, `check` |
| `test_tel_decode.py` | Unit tests, including the golden frames shared with `tests/host/test_tel.c` |
| `tel_views.py` | Per-subsystem views (`--view`) with MEASURED/DERIVED/REPORTED/SIMULATED/N/A labels, and the evidence checks behind `check` (dashboard integration) |
| `test_tel_views.py` | Unit tests for the views and checks, with regressions on the hardware logs in `testing/` |
| `stationary_test.ps1`, `SUBSYSTEM_TESTS.md` | Flash, view live and check: IMU and obstacle tests with the motors never commanded |

```
pip install paho-mqtt                                   # only for listen / send
python tel_dashboard.py listen --broker 192.168.1.10    # Ctrl+C prints the run summary (peak hump)
python tel_dashboard.py send start --broker 192.168.1.10 --allow-motion   # START drives the motors
python tel_dashboard.py decode-log saved_console_log.txt   # console backend, no broker
python tel_dashboard.py decode-log live.txt --view imu --follow   # live, while serial_capture.ps1 writes live.txt
python tel_dashboard.py check imu live.txt                 # also: obstacle (--expect-cm, --expect-width-mm), mqtt
python tel_dashboard.py listen --broker 192.168.1.10 --view obstacle --save run.txt
python tel_dashboard.py decode 010706efbeaddef93f
python -m unittest -v test_tel_decode test_tel_views
```

If a frame layout in `data_repr.h` changes, update `tel_codec.py`, the golden
strings in `test_tel_decode.py` and the ones in `tests/host/test_tel.c`
together. Both test files fail until they agree.

Protocol, topics and setup: `app_program/subsystem1/MQTT.md`.
