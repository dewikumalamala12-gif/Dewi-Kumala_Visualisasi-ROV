"""
ROV Thruster Control - Pixhawk 6C / ArduSub
Fitur:
- Input joystick (pygame) dan keyboard
- Kontrol ArduSub melalui MAVLink MANUAL_CONTROL
- ARM / DISARM dan Emergency Stop
- Monitoring baterai, heading, flight mode, dan SERVO_OUTPUT_RAW
- Monitoring PWM bersifat baca-saja; PWM dihasilkan Pixhawk, bukan laptop

Jalankan:
    pip install pygame pymavlink pyserial
    python rov_control.py

Catatan:
- Tutup QGroundControl saat memakai port serial yang sama.
- Pastikan firmware ArduSub dan konfigurasi frame/motor output benar.
- Uji pertama kali tanpa propeller atau dengan thruster diamankan.
"""

import os
import time
import tkinter as tk
from tkinter import ttk, messagebox

# Pygame hanya dipakai untuk joystick, tidak perlu membuka jendela video.
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

import pygame
from pymavlink import mavutil
from serial.tools import list_ports


LOOP_MS = 50                 # 20 Hz
DEADZONE = 0.08
MAX_PWM_CHANNELS = 16
TEXT_WIDGETS = (tk.Entry, tk.Spinbox, ttk.Entry, ttk.Spinbox, ttk.Combobox)


class ROVApp:
    def __init__(self, root):
        self.root = root
        self.root.title("ROV Thruster Control - Pixhawk 6C")
        self.root.geometry("1000x760")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.master = None
        self.connected = False
        self.js = None
        self.last_js_scan = 0.0
        self.last_gcs_heartbeat = 0.0
        self.emg_latched = False
        self.keys = set()

        self.pwm_outputs = {}   # key: (port, channel 1..8), value: PWM us
        self.battery_text = "-"
        self.heading_text = "-"
        self.alt_text = "-"

        pygame.init()
        pygame.joystick.init()

        self.build_ui()
        self.refresh_ports()
        self.scan_joystick()

        self.root.bind_all("<KeyPress>", self.key_down)
        self.root.bind_all("<KeyRelease>", self.key_up)
        self.root.bind("<FocusOut>", lambda event: self.keys.clear())

        self.loop()

    # ---------------------------------------------------------
    # UI
    # ---------------------------------------------------------
    def build_ui(self):
        pad = {"padx": 6, "pady": 4}

        connection = ttk.LabelFrame(self.root, text="Koneksi Pixhawk")
        connection.grid(row=0, column=0, columnspan=2, sticky="ew", **pad)

        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value="115200")

        ttk.Label(connection, text="Port:").grid(row=0, column=0, **pad)
        self.port_cb = ttk.Combobox(
            connection, textvariable=self.port_var, width=12, state="readonly"
        )
        self.port_cb.grid(row=0, column=1, **pad)
        ttk.Button(
            connection, text="Refresh", command=self.refresh_ports
        ).grid(row=0, column=2, **pad)

        ttk.Label(connection, text="Baud:").grid(row=0, column=3, **pad)
        ttk.Entry(
            connection, textvariable=self.baud_var, width=9
        ).grid(row=0, column=4, **pad)

        self.btn_conn = ttk.Button(
            connection, text="Connect", command=self.toggle_connect
        )
        self.btn_conn.grid(row=0, column=5, **pad)

        self.lbl_conn = tk.Label(
            connection, text="TERPUTUS", fg="white", bg="#b00020", width=13
        )
        self.lbl_conn.grid(row=0, column=6, **pad)

        # Kontrol
        control = ttk.LabelFrame(self.root, text="Kontrol ROV")
        control.grid(row=1, column=0, sticky="nsew", **pad)

        self.lbl_arm = tk.Label(
            control, text="DISARMED", fg="white", bg="#555",
            width=16, font=("Segoe UI", 12, "bold")
        )
        self.lbl_arm.grid(row=0, column=0, columnspan=2, sticky="ew", **pad)

        ttk.Button(control, text="ARM", command=self.do_arm).grid(
            row=1, column=0, sticky="ew", **pad
        )
        ttk.Button(control, text="DISARM", command=self.do_disarm).grid(
            row=1, column=1, sticky="ew", **pad
        )

        tk.Button(
            control,
            text="EMERGENCY STOP (SPASI)",
            bg="#c62828",
            fg="white",
            activebackground="#8e0000",
            font=("Segoe UI", 10, "bold"),
            command=self.emergency
        ).grid(row=2, column=0, columnspan=2, sticky="ew", **pad)

        self.src_var = tk.StringVar(value="joystick")
        ttk.Radiobutton(
            control, text="Joystick", variable=self.src_var, value="joystick"
        ).grid(row=3, column=0, sticky="w", **pad)
        ttk.Radiobutton(
            control, text="Keyboard", variable=self.src_var, value="keyboard"
        ).grid(row=3, column=1, sticky="w", **pad)

        ttk.Label(
            control,
            text="W/S: maju-mundur | A/D: geser\n"
                 "Q/E: yaw | R/F: naik-turun\n"
                 "Spasi: emergency stop",
            foreground="#666"
        ).grid(row=4, column=0, columnspan=2, sticky="w", **pad)

        ttk.Label(control, text="Batas daya kontrol (%):").grid(
            row=5, column=0, columnspan=2, sticky="w", **pad
        )
        self.power = tk.Scale(
            control, from_=0, to=100, orient="horizontal", length=230
        )
        self.power.set(30)
        self.power.grid(row=6, column=0, columnspan=2, **pad)

        # Mapping joystick
        mapping = ttk.LabelFrame(self.root, text="Joystick dan Mapping Axis")
        mapping.grid(row=1, column=1, sticky="nsew", **pad)

        self.lbl_js = ttk.Label(mapping, text="Joystick: -")
        self.lbl_js.grid(row=0, column=0, columnspan=4, sticky="w", **pad)

        ttk.Button(
            mapping, text="Cari Joystick",
            command=lambda: self.scan_joystick(force=True)
        ).grid(row=0, column=4, **pad)

        self.axis_vars = {}
        self.inv_vars = {}

        # Default axis hanya contoh. Sesuaikan dari hasil cek_joystick.py.
        defaults = {
            "surge": (1, True),
            "sway": (0, False),
            "yaw": (2, False),
            "heave": (3, True),
        }

        for row, name in enumerate(("surge", "sway", "yaw", "heave"), start=1):
            ttk.Label(mapping, text=name.capitalize()).grid(
                row=row, column=0, sticky="w", **pad
            )
            axis_var = tk.IntVar(value=defaults[name][0])
            invert_var = tk.BooleanVar(value=defaults[name][1])

            ttk.Spinbox(
                mapping, from_=0, to=15, width=5, textvariable=axis_var
            ).grid(row=row, column=1, **pad)

            ttk.Checkbutton(
                mapping, text="Balik arah", variable=invert_var
            ).grid(row=row, column=2, sticky="w", **pad)

            self.axis_vars[name] = axis_var
            self.inv_vars[name] = invert_var

        ttk.Label(mapping, text="Tombol emergency joystick:").grid(
            row=5, column=0, columnspan=2, sticky="w", **pad
        )
        self.btn_emg = tk.IntVar(value=0)
        ttk.Spinbox(
            mapping, from_=0, to=31, width=5, textvariable=self.btn_emg
        ).grid(row=5, column=2, **pad)

        self.lbl_raw = ttk.Label(mapping, text="Axis mentah: -")
        self.lbl_raw.grid(row=6, column=0, columnspan=5, sticky="w", **pad)

        # Input monitor dan telemetri
        telemetry = ttk.LabelFrame(self.root, text="Input Kontrol dan Telemetri")
        telemetry.grid(row=2, column=0, columnspan=2, sticky="ew", **pad)

        self.lbl_input = ttk.Label(
            telemetry, text="SURGE: 0.00 | SWAY: 0.00 | HEAVE: 0.00 | YAW: 0.00",
            font=("Consolas", 10)
        )
        self.lbl_input.grid(row=0, column=0, sticky="w", **pad)

        self.lbl_tel = ttk.Label(
            telemetry, text="Baterai: - | Heading: - | Alt: - | Mode: -"
        )
        self.lbl_tel.grid(row=1, column=0, sticky="w", **pad)

        # PWM monitor
        pwm_frame = ttk.LabelFrame(
            self.root, text="Pixhawk SERVO_OUTPUT_RAW Monitor (PWM)"
        )
        pwm_frame.grid(row=3, column=0, columnspan=2, sticky="ew", **pad)

        ttk.Label(
            pwm_frame,
            text="Data PWM dibaca dari Pixhawk melalui MAVLink. "
                 "Port/channel mengikuti pesan firmware.",
            foreground="#666"
        ).grid(row=0, column=0, sticky="w", **pad)

        self.lbl_pwm = ttk.Label(
            pwm_frame,
            text="Belum ada data PWM.",
            font=("Consolas", 9),
            justify="left"
        )
        self.lbl_pwm.grid(row=1, column=0, sticky="w", **pad)

        # Log
        log_frame = ttk.LabelFrame(self.root, text="Log")
        log_frame.grid(row=4, column=0, columnspan=2, sticky="nsew", **pad)

        self.log_box = tk.Text(log_frame, height=9, width=110, state="disabled")
        self.log_box.grid(row=0, column=0, sticky="nsew", **pad)

        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_columnconfigure(1, weight=1)
        self.root.grid_rowconfigure(4, weight=1)

    def log(self, message):
        self.log_box.config(state="normal")
        self.log_box.insert("end", time.strftime("[%H:%M:%S] ") + message + "\n")
        self.log_box.see("end")
        self.log_box.config(state="disabled")

    # ---------------------------------------------------------
    # Serial connection / MAVLink
    # ---------------------------------------------------------
    def refresh_ports(self):
        ports = [p.device for p in list_ports.comports()]
        self.port_cb["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])

    def toggle_connect(self):
        if self.connected:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        port = self.port_var.get().strip()
        if not port:
            messagebox.showwarning("Port kosong", "Pilih port Pixhawk terlebih dahulu.")
            return

        try:
            baud = int(self.baud_var.get())
            if baud <= 0:
                raise ValueError("Baud rate harus lebih besar dari nol.")
        except ValueError as exc:
            messagebox.showerror("Baud tidak valid", str(exc))
            return

        self.log(f"Menghubungkan ke {port} dengan baud {baud} ...")

        try:
            self.master = mavutil.mavlink_connection(
                port, baud=baud, autoreconnect=True
            )
            heartbeat = self.master.wait_heartbeat(timeout=8)

            if heartbeat is None:
                raise RuntimeError(
                    "Heartbeat tidak diterima. Periksa port, baud rate, kabel, "
                    "dan pastikan aplikasi lain tidak memakai port tersebut."
                )

        except Exception as exc:
            if self.master:
                try:
                    self.master.close()
                except Exception:
                    pass
            self.master = None
            self.log(f"Koneksi gagal: {exc}")
            messagebox.showerror("Gagal terhubung", str(exc))
            return

        self.connected = True
        self.btn_conn.config(text="Disconnect")
        self.lbl_conn.config(text="TERHUBUNG", bg="#2e7d32")
        self.log(
            f"Pixhawk terhubung. System={self.master.target_system}, "
            f"Component={self.master.target_component}"
        )

        # Minta pesan SERVO_OUTPUT_RAW (MAVLink message ID 36) setiap 100 ms.
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                36,       # SERVO_OUTPUT_RAW
                100000,   # interval: 100.000 us = 10 Hz
                0, 0, 0, 0, 0
            )
            self.log("Permintaan SERVO_OUTPUT_RAW 10 Hz dikirim.")
        except Exception as exc:
            self.log(f"Tidak dapat meminta interval PWM: {exc}")

        # MANUAL merupakan mode kontrol ArduSub yang umum untuk MANUAL_CONTROL.
        # Penggantian mode otomatis dapat gagal jika firmware/mode berbeda.
        try:
            mode_mapping = self.master.mode_mapping() or {}
            if "MANUAL" in mode_mapping:
                self.master.set_mode("MANUAL")
                self.log("Permintaan mode MANUAL dikirim.")
            else:
                self.log("Mode MANUAL tidak ditemukan pada mode mapping firmware.")
        except Exception as exc:
            self.log(f"Gagal meminta mode MANUAL: {exc}")

        # Meminta stream telemetri umum sebagai tambahan.
        try:
            self.master.mav.request_data_stream_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                4,
                1
            )
        except Exception as exc:
            self.log(f"Permintaan data stream gagal: {exc}")

    def disconnect(self):
        if self.master is not None:
            try:
                self.send_neutral()
                if self.master.motors_armed():
                    self.master.arducopter_disarm()
            except Exception as exc:
                self.log(f"Peringatan saat menghentikan kontrol: {exc}")

            try:
                self.master.close()
            except Exception:
                pass

        self.master = None
        self.connected = False
        self.pwm_outputs.clear()
        self.btn_conn.config(text="Connect")
        self.lbl_conn.config(text="TERPUTUS", bg="#b00020")
        self.lbl_arm.config(text="DISARMED", bg="#555")
        self.lbl_pwm.config(text="Belum ada data PWM.")
        self.log("Koneksi diputus.")

    def need_connection(self):
        if not self.connected or self.master is None:
            messagebox.showinfo("Belum terhubung", "Hubungkan Pixhawk terlebih dahulu.")
            return False
        return True

    # ---------------------------------------------------------
    # ARM / DISARM / Emergency
    # ---------------------------------------------------------
    def do_arm(self):
        if not self.need_connection():
            return

        confirmed = messagebox.askyesno(
            "Konfirmasi ARM",
            "ARM akan mengaktifkan sistem motor.\n\n"
            "Pastikan propeller dilepas atau area thruster benar-benar aman.\n\n"
            "Lanjutkan?"
        )
        if not confirmed:
            return

        try:
            self.master.arducopter_arm()
            self.log("Perintah ARM dikirim. Tunggu status ARMED dari Pixhawk.")
        except Exception as exc:
            self.log(f"Gagal mengirim ARM: {exc}")
            messagebox.showerror("ARM gagal", str(exc))

    def do_disarm(self):
        if not self.need_connection():
            return

        try:
            self.send_neutral()
            self.master.arducopter_disarm()
            self.log("Perintah DISARM dikirim.")
        except Exception as exc:
            self.log(f"Gagal mengirim DISARM: {exc}")

    def emergency(self):
        if self.connected and self.master is not None:
            try:
                self.send_neutral()
                self.master.arducopter_disarm()
            except Exception as exc:
                self.log(f"Emergency command error: {exc}")

        self.log("!!! EMERGENCY STOP: perintah netral dan DISARM dikirim !!!")

    def send_neutral(self):
        if self.master is None:
            return

        # MANUAL_CONTROL: x=0, y=0, z=500 (netral), r=0, buttons=0.
        self.master.mav.manual_control_send(
            self.master.target_system, 0, 0, 500, 0, 0
        )

    # ---------------------------------------------------------
    # Keyboard / joystick input
    # ---------------------------------------------------------
    def key_down(self, event):
        if isinstance(event.widget, TEXT_WIDGETS):
            return

        key = event.keysym.lower()
        if key == "space":
            self.emergency()

        self.keys.add(key)

    def key_up(self, event):
        self.keys.discard(event.keysym.lower())

    def scan_joystick(self, force=False):
        self.last_js_scan = time.time()

        if self.js is not None and not force:
            return

        try:
            pygame.joystick.quit()
            pygame.joystick.init()

            if pygame.joystick.get_count() > 0:
                self.js = pygame.joystick.Joystick(0)
                self.js.init()
                self.lbl_js.config(
                    text=f"Joystick: {self.js.get_name()} "
                         f"({self.js.get_numaxes()} axis, "
                         f"{self.js.get_numbuttons()} tombol)"
                )
                self.log("Joystick terdeteksi: " + self.js.get_name())
            else:
                self.js = None
                self.lbl_js.config(text="Joystick: tidak terdeteksi")

        except pygame.error as exc:
            self.js = None
            self.lbl_js.config(text="Joystick: tidak terdeteksi")
            self.log(f"Joystick error: {exc}")

    @staticmethod
    def apply_deadzone(value):
        return 0.0 if abs(value) < DEADZONE else value

    def read_joystick(self):
        output = {
            "surge": 0.0,
            "sway": 0.0,
            "yaw": 0.0,
            "heave": 0.0
        }

        if self.js is None:
            return output

        try:
            pygame.event.pump()
            axis_count = self.js.get_numaxes()

            raw_axes = [
                round(self.js.get_axis(i), 2) for i in range(axis_count)
            ]
            self.lbl_raw.config(text=f"Axis mentah: {raw_axes}")

            for name in output:
                index = self.axis_vars[name].get()

                if 0 <= index < axis_count:
                    value = self.apply_deadzone(self.js.get_axis(index))
                    if self.inv_vars[name].get():
                        value = -value
                    output[name] = value

            button_index = self.btn_emg.get()
            pressed = (
                0 <= button_index < self.js.get_numbuttons()
                and self.js.get_button(button_index)
            )

            if pressed and not self.emg_latched:
                self.emergency()

            self.emg_latched = bool(pressed)

        except (pygame.error, tk.TclError) as exc:
            self.js = None
            self.lbl_js.config(text="Joystick: terputus")
            self.log(f"Joystick terputus/error: {exc}")

        return output

    def read_keyboard(self):
        pressed = self.keys

        def direction(positive, negative):
            return float(positive in pressed) - float(negative in pressed)

        return {
            "surge": direction("w", "s"),
            "sway": direction("d", "a"),
            "yaw": direction("e", "q"),
            "heave": direction("r", "f")
        }

    # ---------------------------------------------------------
    # PWM monitor
    # ---------------------------------------------------------
    def read_pwm_output(self, message):
        """
        SERVO_OUTPUT_RAW memiliki:
        - port
        - servo1_raw sampai servo8_raw

        Satu pesan membawa 8 channel. Umumnya port 0 dan port 1
        mewakili kelompok output yang berbeda. Nama fisik MAIN/AUX
        harus diverifikasi terhadap firmware dan board yang digunakan.
        """
        port = int(getattr(message, "port", 0))

        for channel in range(1, 9):
            value = int(getattr(message, f"servo{channel}_raw", 0))

            # Simpan nilai yang dilaporkan Pixhawk. Nilai 0 juga dipertahankan
            # karena dapat menunjukkan output tidak aktif/tidak tersedia.
            self.pwm_outputs[(port, channel)] = value

        lines = []
        for port_number in sorted({key[0] for key in self.pwm_outputs}):
            for channel in range(1, 9):
                value = self.pwm_outputs.get((port_number, channel), None)
                if value is not None:
                    global_channel = port_number * 8 + channel
                    lines.append(
                        f"OUT {global_channel:02d}: {value:4d} us"
                    )

        if lines:
            self.lbl_pwm.config(text=" | ".join(lines))

    # ---------------------------------------------------------
    # Main loop and telemetry
    # ---------------------------------------------------------
    def loop(self):
        if self.js is None and time.time() - self.last_js_scan > 3:
            self.scan_joystick()

        if self.src_var.get() == "keyboard":
            raw = self.read_keyboard()
        else:
            raw = self.read_joystick()

        power_scale = self.power.get() / 100.0
        values = {
            key: max(-1.0, min(1.0, raw[key])) * power_scale
            for key in raw
        }

        self.lbl_input.config(
            text=(
                f"SURGE: {values['surge']:+.2f} | "
                f"SWAY: {values['sway']:+.2f} | "
                f"HEAVE: {values['heave']:+.2f} | "
                f"YAW: {values['yaw']:+.2f}"
            )
        )

        if self.connected and self.master is not None:
            try:
                # Jangan kirim perintah gerak saat belum ARMED.
                # Netral tetap dikirim agar input sebelumnya tidak tertahan.
                if self.master.motors_armed():
                    x = int(values["surge"] * 1000)
                    y = int(values["sway"] * 1000)
                    z = int(500 + values["heave"] * 500)
                    r = int(values["yaw"] * 1000)
                else:
                    x, y, z, r = 0, 0, 500, 0

                self.master.mav.manual_control_send(
                    self.master.target_system,
                    x, y, z, r, 0
                )

                now = time.time()
                if now - self.last_gcs_heartbeat >= 1.0:
                    self.master.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_GCS,
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                        0, 0,
                        mavutil.mavlink.MAV_STATE_ACTIVE
                    )
                    self.last_gcs_heartbeat = now

                self.read_telemetry()

            except Exception as exc:
                self.log(f"Kesalahan komunikasi MAVLink: {exc}")
                self.disconnect()

        self.root.after(LOOP_MS, self.loop)

    def read_telemetry(self):
        if self.master is None:
            return

        # Batasi jumlah pesan per siklus agar GUI tetap responsif.
        for _ in range(80):
            message = self.master.recv_match(blocking=False)
            if message is None:
                break

            message_type = message.get_type()

            if message_type == "SERVO_OUTPUT_RAW":
                self.read_pwm_output(message)

            elif message_type == "SYS_STATUS":
                voltage = getattr(message, "voltage_battery", 0)
                if voltage not in (0, 65535):
                    self.battery_text = f"{voltage / 1000:.1f} V"

            elif message_type == "VFR_HUD":
                heading = getattr(message, "heading", None)
                altitude = getattr(message, "alt", None)

                if heading is not None:
                    self.heading_text = f"{heading}°"
                if altitude is not None:
                    self.alt_text = f"{altitude:.1f} m"

        try:
            armed = self.master.motors_armed()
            self.lbl_arm.config(
                text="ARMED" if armed else "DISARMED",
                bg="#d32f2f" if armed else "#555"
            )
        except Exception:
            pass

        mode = self.master.flightmode or "-"
        self.lbl_tel.config(
            text=(
                f"Baterai: {self.battery_text} | "
                f"Heading: {self.heading_text} | "
                f"Alt: {self.alt_text} | Mode: {mode}"
            )
        )

    # ---------------------------------------------------------
    # Close
    # ---------------------------------------------------------
    def on_close(self):
        if self.connected:
            self.disconnect()

        try:
            pygame.quit()
        except pygame.error:
            pass

        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = ROVApp(root)
    root.mainloop()
