# STC_Core
Repo for STC32G144

## WiFi start/stop GUI

This repo now includes a simple desktop control tool at [tools/wifi_ctrl_gui.py](tools/wifi_ctrl_gui.py).

- Firmware side:
  `project/user/main.c` now accepts `START`, `STOP`, and `PING` commands over the existing WiFi TCP link.
- Safety:
  `START` is treated as a heartbeat. If the GUI or WiFi link disappears for `WIFI_CTRL_COMMAND_TIMEOUT_MS`, the firmware sends `send_ctrl_data(0, 0, 0, LOWER_CTRL_UART)` automatically.
- Desktop side:
  Run `python tools/wifi_ctrl_gui.py`. The GUI opens a TCP server and sends `START` repeatedly while run mode is active.

The PC network interface must match `WIFI_SPI_TARGET_IP` and `WIFI_SPI_TARGET_PORT` in `libraries/zf_device/zf_device_wifi_spi.h`.
This GUI uses the same TCP connection as the existing WiFi image/debug stream, so it will drain that data but does not display it.
