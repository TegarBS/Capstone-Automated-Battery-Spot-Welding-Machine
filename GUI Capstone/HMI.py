import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import serial.tools.list_ports
from pymodbus.client import ModbusSerialClient

# =========================================================
# KELAS UTAMA: HMI MESIN LAS (MANUAL, AUTO, PAUSE & STOP)
# =========================================================
class MesinLasHMI:
    def __init__(self, root):
        self.root = root
        self.root.title("Dashboard Mesin Las Baterai (Modbus RTU)")
        self.root.geometry("680x750") # Sedikit diperpanjang untuk tombol baru
        
        self.client = None
        self.job_queue = []    
        self.is_welding = False 
        self.is_paused = False          # [BARU] Bendera Jeda HMI
        self.jeda_antar_titik_ms = 1500 
        self.waktu_homing_ms = 3000     

        self.setup_ui()
        self.update_com_ports()

    def setup_ui(self):
        # --- 1. FRAME KONEKSI SERIAL ---
        frame_conn = ttk.LabelFrame(self.root, text=" 1. Koneksi Serial (Modbus RTU) ")
        frame_conn.pack(padx=10, pady=5, fill="x")

        ttk.Label(frame_conn, text="Port:").grid(row=0, column=0, padx=5, pady=5)
        self.cb_ports = ttk.Combobox(frame_conn, width=15)
        self.cb_ports.grid(row=0, column=1, padx=5, pady=5)

        self.btn_refresh = ttk.Button(frame_conn, text="Refresh", command=self.update_com_ports)
        self.btn_refresh.grid(row=0, column=2, padx=5, pady=5)

        self.btn_connect = ttk.Button(frame_conn, text="Connect", command=self.toggle_connection)
        self.btn_connect.grid(row=0, column=3, padx=5, pady=5)

        # --- 2. FRAME KONTROL MANUAL ---
        frame_manual = ttk.LabelFrame(self.root, text=" 2. Mode Manual (Single Jog & Homing) ")
        frame_manual.pack(padx=10, pady=5, fill="x")

        ttk.Label(frame_manual, text="Target X:").grid(row=0, column=0, padx=5, pady=5)
        self.entry_man_x = ttk.Entry(frame_manual, width=8)
        self.entry_man_x.grid(row=0, column=1, padx=5, pady=5)
        self.entry_man_x.insert(0, "3200")

        ttk.Label(frame_manual, text="Target Y:").grid(row=0, column=2, padx=5, pady=5)
        self.entry_man_y = ttk.Entry(frame_manual, width=8)
        self.entry_man_y.grid(row=0, column=3, padx=5, pady=5)
        self.entry_man_y.insert(0, "3200")

        self.btn_send_manual = ttk.Button(frame_manual, text="KIRIM & LAS", command=self.send_manual_weld, state="disabled")
        self.btn_send_manual.grid(row=0, column=4, padx=5, pady=5)

        self.btn_manual_home = tk.Button(frame_manual, text="HOME MESIN (0,0)", bg="blue", fg="white", font=("Arial", 9, "bold"), command=self.trigger_manual_home, state="disabled")
        self.btn_manual_home.grid(row=0, column=5, padx=10, pady=5)

        # --- 3. FRAME KONTROL OTOMATIS ---
        frame_auto = ttk.LabelFrame(self.root, text=" 3. Mode Otomatis (Grid Baterai CAM) ")
        frame_auto.pack(padx=10, pady=5, fill="x")

        ttk.Label(frame_auto, text="Jumlah Baterai X (Kolom):").grid(row=0, column=0, padx=5, pady=5, sticky="e")
        self.entry_kolom = ttk.Entry(frame_auto, width=8)
        self.entry_kolom.grid(row=0, column=1, padx=5, pady=5)
        self.entry_kolom.insert(0, "4")

        ttk.Label(frame_auto, text="Jumlah Baterai Y (Baris):").grid(row=0, column=2, padx=5, pady=5, sticky="e")
        self.entry_baris = ttk.Entry(frame_auto, width=8)
        self.entry_baris.grid(row=0, column=3, padx=5, pady=5)
        self.entry_baris.insert(0, "3")

        ttk.Label(frame_auto, text="Pitch X (Jarak mm):").grid(row=1, column=0, padx=5, pady=5, sticky="e")
        self.entry_pitch_x = ttk.Entry(frame_auto, width=8)
        self.entry_pitch_x.grid(row=1, column=1, padx=5, pady=5)
        self.entry_pitch_x.insert(0, "18.5")

        ttk.Label(frame_auto, text="Pitch Y (Jarak mm):").grid(row=1, column=2, padx=5, pady=5, sticky="e")
        self.entry_pitch_y = ttk.Entry(frame_auto, width=8)
        self.entry_pitch_y.grid(row=1, column=3, padx=5, pady=5)
        self.entry_pitch_y.insert(0, "18.5")

        frame_auto_btns = tk.Frame(frame_auto)
        frame_auto_btns.grid(row=2, column=0, columnspan=4, pady=10)

        # Tombol Operasional Otomatis
        self.btn_start_auto = tk.Button(frame_auto_btns, text="START AUTO", bg="green", fg="white", font=("Arial", 10, "bold"), command=self.start_auto_weld, state="disabled")
        self.btn_start_auto.pack(side="left", padx=5)

        self.btn_pause_auto = tk.Button(frame_auto_btns, text="PAUSE", bg="orange", fg="black", font=("Arial", 10, "bold"), command=self.toggle_pause, state="disabled")
        self.btn_pause_auto.pack(side="left", padx=5)

        self.btn_stop_auto = tk.Button(frame_auto_btns, text="EMERGENCY STOP", bg="red", fg="white", font=("Arial", 10, "bold"), command=self.stop_auto_weld, state="disabled")
        self.btn_stop_auto.pack(side="left", padx=5)

        # --- 4. FRAME LOG TERMINAL ---
        frame_log = ttk.LabelFrame(self.root, text=" Status & Log Mesin ")
        frame_log.pack(padx=10, pady=5, fill="both", expand=True)

        self.txt_log = scrolledtext.ScrolledText(frame_log, wrap=tk.WORD, height=10)
        self.txt_log.pack(padx=5, pady=5, fill="both", expand=True)

    # =========================================================
    # KONTROL KONEKSI SERIAL
    # =========================================================
    def update_com_ports(self):
        ports = serial.tools.list_ports.comports()
        self.cb_ports['values'] = [port.device for port in ports]
        if ports:
            self.cb_ports.current(0)

    def toggle_connection(self):
        if self.client is None or not self.client.is_socket_open():
            port = self.cb_ports.get()
            if port:
                self.client = ModbusSerialClient(port=port, baudrate=115200, timeout=1)
                if self.client.connect():
                    self.btn_connect.config(text="Disconnect")
                    self.btn_send_manual.config(state="normal")
                    self.btn_manual_home.config(state="normal")
                    self.btn_start_auto.config(state="normal")
                    self.cb_ports.config(state="disabled")
                    self.log_message(f"Berhasil terhubung ke {port} via Modbus RTU.")
                else:
                    messagebox.showerror("Error", "Gagal membuka port serial!")
        else:
            self.stop_auto_weld()
            self.client.close()
            self.btn_connect.config(text="Connect")
            self.btn_send_manual.config(state="disabled")
            self.btn_manual_home.config(state="disabled")
            self.btn_start_auto.config(state="disabled")
            self.btn_pause_auto.config(state="disabled")
            self.btn_stop_auto.config(state="disabled")
            self.cb_ports.config(state="normal")
            self.log_message("Koneksi diputus.")

    # =========================================================
    # FUNGSI MODBUS DASAR
    # =========================================================
    def send_home_command(self):
        if self.client and self.client.is_socket_open():
            self.client.write_coil(2, True)  # Offset 2 = Coil 00003 (Home)
            self.client.write_coil(2, False)
            return True
        return False

    def send_stop_command(self):
        if self.client and self.client.is_socket_open():
            self.client.write_coil(1, True)  # Offset 1 = Coil 00002 (Stop/Abort)
            self.client.write_coil(1, False)

    def send_pause_command(self, state):
        if self.client and self.client.is_socket_open():
            self.client.write_coil(3, state) # Offset 3 = Coil 00004 (Pause/Resume)

    def send_target_modbus(self, x_val, y_val):
        x_low  = x_val & 0xFFFF
        x_high = (x_val >> 16) & 0xFFFF
        y_low  = y_val & 0xFFFF
        y_high = (y_val >> 16) & 0xFFFF

        self.client.write_register(0, x_low)
        self.client.write_register(1, x_high)
        self.client.write_register(2, y_low)
        self.client.write_register(3, y_high)
        self.client.write_coil(0, True)  # Offset 0 = Coil 00001 (Start Weld)
        self.client.write_coil(0, False)

    # =========================================================
    # LOGIKA MODE MANUAL
    # =========================================================
    def trigger_manual_home(self):
        if self.is_welding:
            messagebox.showwarning("Peringatan", "Mesin sedang dalam mode AUTO. Hentikan dulu!")
            return
        try:
            if self.send_home_command():
                self.log_message("[MANUAL HOMING] Perintah Homing dikirim ke STM32...")
        except Exception as e:
            self.log_message(f"ERROR HOMING: {str(e)}")

    def send_manual_weld(self):
        if self.is_welding:
            messagebox.showwarning("Peringatan", "Mesin sedang dalam mode AUTO!")
            return
        if self.client and self.client.is_socket_open():
            try:
                x_val = int(self.entry_man_x.get())
                y_val = int(self.entry_man_y.get())
                self.send_target_modbus(x_val, y_val)
                self.log_message(f"[MANUAL] Eksekusi Titik Las -> X:{x_val} Y:{y_val}")
            except ValueError:
                messagebox.showerror("Error", "Format angka manual salah!")
            except Exception as e:
                self.log_message(f"ERROR MANUAL: {str(e)}")

    # =========================================================
    # LOGIKA MODE AUTO (CAM, PAUSE, STOP)
    # =========================================================
    def generate_grid_jobs(self, cols, rows, pitch_x, pitch_y):
        steps_per_mm = 160 
        jobs = []
        for r in range(rows):
            col_sequence = range(cols) if r % 2 == 0 else range(cols - 1, -1, -1)
            for c in col_sequence:
                x_step = int((c * pitch_x) * steps_per_mm)
                y_step = int((r * pitch_y) * steps_per_mm)
                jobs.append((x_step, y_step))
        return jobs

    def start_auto_weld(self):
        if self.client and self.client.is_socket_open():
            try:
                cols = int(self.entry_kolom.get())
                rows = int(self.entry_baris.get())
                pitch_x = float(self.entry_pitch_x.get())
                pitch_y = float(self.entry_pitch_y.get())
            except ValueError:
                messagebox.showerror("Error", "Format parameter grid salah!")
                return
            
            self.job_queue = self.generate_grid_jobs(cols, rows, pitch_x, pitch_y)
            self.is_welding = True
            self.is_paused = False
            
            # Kunci Tombol UI
            self.btn_start_auto.config(state="disabled")
            self.btn_send_manual.config(state="disabled")
            self.btn_manual_home.config(state="disabled")
            self.btn_pause_auto.config(state="normal", text="PAUSE", bg="orange")
            self.btn_stop_auto.config(state="normal")
            
            self.log_message(f"\n=== MEMULAI MODE AUTO: {len(self.job_queue)} TITIK LAS ===")
            self.log_message("[AUTO] Melakukan Auto Homing...")
            self.send_home_command()

            # Jadwalkan pekerjaan pertama setelah homing selesai
            self.root.after(self.waktu_homing_ms, self.send_next_job)

    def toggle_pause(self):
        if self.is_welding:
            self.is_paused = not self.is_paused
            if self.is_paused:
                self.btn_pause_auto.config(text="RESUME", bg="yellow")
                self.send_pause_command(True)  # Tembak Coil PAUSE ON
                self.log_message("=== MESIN DI-PAUSE ===")
            else:
                self.btn_pause_auto.config(text="PAUSE", bg="orange")
                self.send_pause_command(False) # Tembak Coil PAUSE OFF
                self.log_message("=== MESIN DILANJUTKAN (RESUME) ===")

    def stop_auto_weld(self):
        if self.is_welding or self.is_paused:
            self.is_welding = False
            self.is_paused = False
            self.job_queue.clear()
            
            # 1. Tembak Sinyal E-STOP ke STM32 agar rem seketika
            self.send_stop_command()

            # 2. Kembalikan Tombol UI
            self.btn_start_auto.config(state="normal")
            self.btn_send_manual.config(state="normal")
            self.btn_manual_home.config(state="normal")
            self.btn_pause_auto.config(text="PAUSE", bg="orange", state="disabled")
            self.btn_stop_auto.config(state="disabled")
            
            self.log_message("!!! EMERGENCY STOP: MODE AUTO DIBATALKAN !!!")

    def send_next_job(self):
        # 1. Jika sedang di-Pause, Jangan kirim data baru. Cek lagi 500ms kemudian.
        if self.is_welding and self.is_paused:
            self.root.after(500, self.send_next_job)
            return

        # 2. Jika pengelasan aktif dan masih ada antrean
        if self.is_welding and len(self.job_queue) > 0:
            x_step, y_step = self.job_queue.pop(0) 
            
            try:
                self.send_target_modbus(x_step, y_step)
                self.log_message(f"[AUTO] Titik X:{x_step} Y:{y_step} dieksekusi (Sisa: {len(self.job_queue)})")
                
                # Jadwalkan titik berikutnya
                self.root.after(self.jeda_antar_titik_ms, self.send_next_job)

            except Exception as e:
                self.log_message(f"ERROR AUTO WELD: {str(e)}")
                self.stop_auto_weld()

        # 3. Jika antrean habis
        elif self.is_welding and len(self.job_queue) == 0:
            self.is_welding = False
            self.btn_start_auto.config(state="normal")
            self.btn_send_manual.config(state="normal")
            self.btn_manual_home.config(state="normal")
            self.btn_pause_auto.config(state="disabled")
            self.btn_stop_auto.config(state="disabled")
            self.log_message("=== SELURUH GRID BATERAI SELESAI DILAS ===")

    def log_message(self, msg):
        self.txt_log.insert(tk.END, msg + "\n")
        self.txt_log.see(tk.END)

if __name__ == "__main__":
    root = tk.Tk()
    app = MesinLasHMI(root)
    root.mainloop()