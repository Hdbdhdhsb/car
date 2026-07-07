import re
import socket
import threading
import tkinter as tk
from contextlib import suppress
from pathlib import Path
from tkinter import ttk


REPO_ROOT = Path(__file__).resolve().parents[1]
WIFI_HEADER = REPO_ROOT / "libraries" / "zf_device" / "zf_device_wifi_spi.h"

DEFAULT_BIND_HOST = "0.0.0.0"
DEFAULT_PORT = 8086
HEARTBEAT_INTERVAL_MS = 200


def load_firmware_socket_defaults():
    """Read the firmware WiFi target config so the GUI can show the expected IP/port."""
    ip_addr = "192.168.43.5"
    port = DEFAULT_PORT

    if not WIFI_HEADER.exists():
        return ip_addr, port

    header_text = WIFI_HEADER.read_text(encoding="utf-8", errors="ignore")
    ip_match = re.search(r'#define\s+WIFI_SPI_TARGET_IP\s+"([^"]+)"', header_text)
    port_match = re.search(r'#define\s+WIFI_SPI_TARGET_PORT\s+"([^"]+)"', header_text)

    if ip_match:
        ip_addr = ip_match.group(1)

    if port_match:
        with suppress(ValueError):
            port = int(port_match.group(1))

    return ip_addr, port


class WifiControlServer:
    """Single-client TCP server used by the car's WiFi module."""

    def __init__(self, app, host, port):
        self.app = app
        self.host = host
        self.port = port
        self.stop_event = threading.Event()
        self.server_thread = None
        self.server_socket = None
        self.client_socket = None
        self.client_lock = threading.Lock()
        self.total_bytes_received = 0

    def start(self):
        if self.server_thread and self.server_thread.is_alive():
            return

        self.stop_event.clear()
        self.server_thread = threading.Thread(target=self._run, daemon=True)
        self.server_thread.start()

    def close(self):
        self.stop_event.set()

        with self.client_lock:
            client_socket = self.client_socket
            self.client_socket = None

        if client_socket is not None:
            with suppress(OSError):
                client_socket.shutdown(socket.SHUT_RDWR)
            with suppress(OSError):
                client_socket.close()

        if self.server_socket is not None:
            with suppress(OSError):
                self.server_socket.close()
            self.server_socket = None

    def has_client(self):
        with self.client_lock:
            return self.client_socket is not None

    def send_command(self, command):
        payload = f"{command}\n".encode("ascii")

        with self.client_lock:
            client_socket = self.client_socket

        if client_socket is None:
            raise RuntimeError("No car is connected.")

        client_socket.sendall(payload)

    def _run(self):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
                server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                server_socket.bind((self.host, self.port))
                server_socket.listen(1)
                server_socket.settimeout(1.0)
                self.server_socket = server_socket
                self.app.call_in_ui(
                    self.app.set_status,
                    f"Listening on {self.host}:{self.port}. Waiting for the car to connect.",
                )

                while not self.stop_event.is_set():
                    try:
                        client_socket, client_addr = server_socket.accept()
                    except socket.timeout:
                        continue
                    except OSError:
                        break

                    client_socket.settimeout(1.0)
                    self._set_client(client_socket)
                    self.app.call_in_ui(self.app.on_client_connected, client_addr)
                    self._receive_loop(client_socket)
                    self._clear_client(client_socket)
                    self.app.call_in_ui(self.app.on_client_disconnected)
        except OSError as exc:
            self.app.call_in_ui(self.app.set_status, f"Server error: {exc}")

    def _receive_loop(self, client_socket):
        while not self.stop_event.is_set():
            try:
                packet = client_socket.recv(4096)
            except socket.timeout:
                continue
            except OSError:
                break

            if not packet:
                break

            self.total_bytes_received += len(packet)
            self.app.call_in_ui(self.app.update_rx_bytes, self.total_bytes_received)

    def _set_client(self, client_socket):
        with self.client_lock:
            old_socket = self.client_socket
            self.client_socket = client_socket

        if old_socket is not None:
            with suppress(OSError):
                old_socket.shutdown(socket.SHUT_RDWR)
            with suppress(OSError):
                old_socket.close()

    def _clear_client(self, client_socket):
        with self.client_lock:
            if self.client_socket is client_socket:
                self.client_socket = None

        with suppress(OSError):
            client_socket.close()


class CarControlApp(tk.Tk):
    """Small desktop GUI for sending START/STOP over the existing WiFi TCP link."""

    def __init__(self):
        super().__init__()

        self.target_ip, self.target_port = load_firmware_socket_defaults()
        self.title("WiFi Car Control")
        self.geometry("460x240")
        self.resizable(False, False)

        self.connected = False
        self.run_mode = False
        self.heartbeat_job = None

        self.status_var = tk.StringVar(value="Starting server...")
        self.client_var = tk.StringVar(value="Client: disconnected")
        self.bytes_var = tk.StringVar(value="RX bytes: 0")
        self.note_var = tk.StringVar(
            value=(
                f"Firmware target is {self.target_ip}:{self.target_port}. "
                "Your PC must own that IP, or update zf_device_wifi_spi.h."
            )
        )

        self.server = WifiControlServer(self, DEFAULT_BIND_HOST, self.target_port)

        self._build_ui()
        self._update_button_state()

        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.server.start()

    def _build_ui(self):
        root = ttk.Frame(self, padding=18)
        root.pack(fill="both", expand=True)

        title = ttk.Label(root, text="Car WiFi Control", font=("Segoe UI", 16, "bold"))
        title.pack(anchor="w")

        status = ttk.Label(root, textvariable=self.status_var, wraplength=420)
        status.pack(anchor="w", pady=(10, 6))

        client = ttk.Label(root, textvariable=self.client_var)
        client.pack(anchor="w")

        rx_bytes = ttk.Label(root, textvariable=self.bytes_var)
        rx_bytes.pack(anchor="w", pady=(4, 14))

        button_row = ttk.Frame(root)
        button_row.pack(fill="x")

        self.start_button = tk.Button(
            button_row,
            text="Start",
            width=14,
            bg="#2d7d46",
            fg="white",
            activebackground="#245f36",
            activeforeground="white",
            relief="flat",
            command=self.start_run_mode,
        )
        self.start_button.pack(side="left", padx=(0, 12))

        self.stop_button = tk.Button(
            button_row,
            text="Stop",
            width=14,
            bg="#b33a3a",
            fg="white",
            activebackground="#8d2d2d",
            activeforeground="white",
            relief="flat",
            command=self.stop_run_mode,
        )
        self.stop_button.pack(side="left")

        note = ttk.Label(root, textvariable=self.note_var, wraplength=420, foreground="#555555")
        note.pack(anchor="w", pady=(18, 0))

    def call_in_ui(self, callback, *args):
        self.after(0, lambda: callback(*args))

    def set_status(self, message):
        self.status_var.set(message)

    def update_rx_bytes(self, total_bytes):
        self.bytes_var.set(f"RX bytes: {total_bytes}")

    def on_client_connected(self, client_addr):
        self.connected = True
        self.client_var.set(f"Client: {client_addr[0]}:{client_addr[1]}")
        self.set_status("Car connected. Start will begin the send_ctrl_data loop.")
        self._update_button_state()

    def on_client_disconnected(self):
        self.connected = False

        if self.run_mode:
            self._cancel_heartbeat()
            self.run_mode = False
            self.set_status("Car disconnected. Run mode cleared; firmware timeout will stop the car.")
        else:
            self.set_status("Car disconnected. Waiting for reconnection.")

        self.client_var.set("Client: disconnected")
        self._update_button_state()

    def _update_button_state(self):
        can_start = self.connected and not self.run_mode
        can_stop = self.connected and self.run_mode

        self.start_button.configure(state=("normal" if can_start else "disabled"))
        self.stop_button.configure(state=("normal" if can_stop else "disabled"))

    def start_run_mode(self):
        if not self.connected:
            self.set_status("No car connected yet.")
            return

        if self.run_mode:
            return

        self.run_mode = True
        self.set_status("Run mode enabled. Sending START heartbeat.")
        self._update_button_state()
        self._send_start_heartbeat()

    def stop_run_mode(self):
        if self.run_mode:
            self.run_mode = False
            self._cancel_heartbeat()

        try:
            self.server.send_command("STOP")
            self.set_status("STOP sent. The car should send send_ctrl_data(0, 0, 0, uart).")
        except (RuntimeError, OSError) as exc:
            self.set_status(f"Failed to send STOP: {exc}")

        self._update_button_state()

    def _send_start_heartbeat(self):
        if not self.run_mode:
            return

        if not self.connected:
            self.run_mode = False
            self._update_button_state()
            self.set_status("Heartbeat stopped because the car is not connected.")
            return

        try:
            self.server.send_command("START")
        except (RuntimeError, OSError) as exc:
            self.run_mode = False
            self._update_button_state()
            self.set_status(f"Failed to send START heartbeat: {exc}")
            return

        self.heartbeat_job = self.after(HEARTBEAT_INTERVAL_MS, self._send_start_heartbeat)

    def _cancel_heartbeat(self):
        if self.heartbeat_job is not None:
            self.after_cancel(self.heartbeat_job)
            self.heartbeat_job = None

    def on_close(self):
        self._cancel_heartbeat()

        if self.connected:
            with suppress(RuntimeError, OSError):
                self.server.send_command("STOP")

        self.server.close()
        self.destroy()


if __name__ == "__main__":
    CarControlApp().mainloop()
