# Telemetry dashboard (Buddy 1)

Laptop-side tool for the car's MQTT telemetry. Python 3.8+.

| File | What it is |
|---|---|
| `tel_codec.py` | Frame decoder/encoder, a mirror of `src/common/data_repr.h` (standard library only) |
| `tel_dashboard.py` | CLI: `listen`, `send`, `decode-log`, `decode` |
| `test_tel_decode.py` | Unit tests, including the golden frames shared with `tests/host/test_tel.c` |

```
pip install paho-mqtt                                   # only for listen / send
python tel_dashboard.py listen --broker 192.168.1.10    # Ctrl+C prints the run summary (peak hump)
python tel_dashboard.py send start --broker 192.168.1.10
python tel_dashboard.py decode-log saved_console_log.txt   # console backend, no broker
python tel_dashboard.py decode 010706efbeaddef93f
python -m unittest -v test_tel_decode
```

If a frame layout in `data_repr.h` changes, update `tel_codec.py`, the golden
strings in `test_tel_decode.py` and the ones in `tests/host/test_tel.c`
together. Both test files fail until they agree.

Protocol, topics and setup: `app_program/subsystem1/MQTT.md`.
