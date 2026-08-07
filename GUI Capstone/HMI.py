import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import serial.tools.list_ports
from pymodbus.client import ModbusSerialClient
import threading
import time

# =========================================================
# KELAS UTAMA: HMI MESIN LAS (ZERO-ARRAY ARCHITECTURE)
# =========================================================
class MesinLasHMI:
    def __init__(self, root):
        self.root = root
        self.root.title("Dashboard Mesin Las Baterai (Modbus RTU)")
        self.root.geometry("720x800") # Diperlebar sedikit agar muat indeks baterai
        
        self.client = None
        self.is_welding = False 
        self.is_paused = False          
        
        # Variabel untuk thread pemantauan live
        self.monitor_thread = None
        self.stop_monitor = False

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

        # --- 2. FRAME KONTROL MANUAL & MONITOR ---
        frame_manual = ttk.LabelFrame(self.root, text=" 2. Mode Manual & Pemantauan (Live) ")
        frame_manual.pack(padx=10, pady=5, fill="x")

        # Baris 0: Monitor Posisi Live
        ttk.Label(frame_manual, text="Posisi X Aktual:").grid(row=0, column=0, padx=5, pady=5, sticky="e")
        self.lbl_pos_x = ttk.Label(frame_manual, text="0.00 mm", font=("Arial", 10, "bold"), foreground="blue")
        self.lbl_pos_x.grid(row=0, column=1, padx=5, pady=5, sticky="w")

        ttk.Label(frame_manual, text="Posisi Y Aktual:").grid(row=0, column=2, padx=5, pady=5, sticky="e")
        self.lbl_pos_y = ttk.Label(frame_manual, text="0.00 mm", font=("Arial", 10, "bold"), foreground="blue")
        self.lbl_pos_y.grid(row=0, column=3, padx=5, pady=5, sticky="w")

        self.btn_manual_home = tk.Button(frame_manual, text="HOME MESIN", bg="blue", fg="white", font=("Arial", 9, "bold"), command=self.trigger_manual_home, state="disabled")
        self.btn_manual_home.grid(row=0, column=4, padx=10, pady=5)

        # [BARU] Baris 1: Monitor Index Baterai Live
        ttk.Label(frame_manual, text="Index Kolom (X):").grid(row=1, column=0, padx=5, pady=5, sticky="e")
        self.lbl_idx_kolom = ttk.Label(frame_manual, text="0", font=("Arial", 10, "bold"), foreground="green")
        self.lbl_idx_kolom.grid(row=1, column=1, padx=5, pady=5, sticky="w")

        ttk.Label(frame_manual, text="Index Baris (Y):").grid(row=1, column=2, padx=5, pady=5, sticky="e")
        self.lbl_idx_baris = ttk.Label(frame_manual, text="0", font=("Arial", 10, "bold"), foreground="green")
        self.lbl_idx_baris.grid(row=1, column=3, padx=5, pady=5, sticky="w")

        # Baris 2: Input Mode Manual Jog
        ttk.Label(frame_manual, text="Target X (mm):").grid(row=2, column=0, padx=5, pady=5, sticky="e")
        self.entry_man_x = ttk.Entry(frame_manual, width=8)
        self.entry_man_x.grid(row=2, column=1, padx=5, pady=5, sticky="w")
        self.entry_man_x.insert(0, "10")

        ttk.Label(frame_manual, text="Target Y (mm):").grid(row=2, column=2, padx=5, pady=5, sticky="e")
        self.entry_man_y = ttk.Entry(frame_manual, width=8)
        self.entry_man_y.grid(row=2, column=3, padx=5, pady=5, sticky="w")
        self.entry_man_y.insert(0, "10")

        self.btn_send_manual = ttk.Button(frame_manual, text="KIRIM JOG", command=self.send_manual_weld)
        self.btn_send_manual.grid(row=2, column=4, padx=5, pady=5)

        # --- 3. FRAME KONTROL OTOMATIS (SETTING PARAMETER) ---
        frame_auto = ttk.LabelFrame(self.root, text=" 3. Mode Otomatis (Setting Parameter Mesin) ")
        frame_auto.pack(padx=10, pady=5, fill="x")

        ttk.Label(frame_auto, text="Jumlah Kolom X:").grid(row=0, column=0, padx=5, pady=5, sticky="e")
        self.entry_kolom = ttk.Entry(frame_auto, width=8)
        self.entry_kolom.grid(row=0, column=1, padx=5, pady=5)
        self.entry_kolom.insert(0, "4")

        ttk.Label(frame_auto, text="Jumlah Baris Y:").grid(row=0, column=2, padx=5, pady=5, sticky="e")
        self.entry_baris = ttk.Entry(frame_auto, width=8)
        self.entry_baris.grid(row=0, column=3, padx=5, pady=5)
        self.entry_baris.insert(0, "3")

        ttk.Label(frame_auto, text="Pitch X (mm):").grid(row=1, column=0, padx=5, pady=5, sticky="e")
        self.entry_pitch_x = ttk.Entry(frame_auto, width=8)
        self.entry_pitch_x.grid(row=1, column=1, padx=5, pady=5)
        self.entry_pitch_x.insert(0, "18.5")

        ttk.Label(frame_auto, text="Pitch Y (mm):").grid(row=1, column=2, padx=5, pady=5, sticky="e")
        self.entry_pitch_y = ttk.Entry(frame_auto, width=8)
        self.entry_pitch_y.grid(row=1, column=3, padx=5, pady=5)
        self.entry_pitch_y.insert(0, "18.5")

        frame_auto_btns = tk.Frame(frame_auto)
        frame_auto_btns.grid(row=2, column=0, columnspan=4, pady=10)

        # Tombol Operasional Otomatis
        self.btn_start_auto = tk.Button(frame_auto_btns, text="START AUTO", bg="green", fg="white", font=("Arial", 10, "bold"), command=self.start_auto_weld, state="disabled")
        self.btn_start_auto.pack(side="left", padx=5)

        self.btn_pause_auto = tk.Button(frame_auto_btns, text="PAUSE", bg="orange", fg="black", font=("Arial", 10, "bold"), command=self.toggle_pause)
        self.btn_pause_auto.pack(side="left", padx=5)

        self.btn_stop_auto = tk.Button(frame_auto_btns, text="EMERGENCY STOP", bg="red", fg="white", font=("Arial", 10, "bold"), command=self.stop_auto_weld)
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
                # Modbus RTU standar industri (8N1)
                self.client = ModbusSerialClient(port=port, baudrate=115200, timeout=1)
                if self.client.connect():
                    self.btn_connect.config(text="Disconnect")
                    self.btn_manual_home.config(state="normal")
                    self.btn_start_auto.config(state="normal")
                    self.cb_ports.config(state="disabled")
                    self.log_message(f"Berhasil terhubung ke {port} via Modbus RTU.")
                    
                    # Mulai thread pemantauan live
                    self.stop_monitor = False
                    self.monitor_thread = threading.Thread(target=self.live_monitor_loop, daemon=True)
                    self.monitor_thread.start()
                else:
                    messagebox.showerror("Error", "Gagal membuka port serial!")
        else:
            self.stop_monitor = True # Matikan pemantauan
            time.sleep(0.5)          # Beri waktu thread berhenti
            self.stop_auto_weld()    # Pastikan mesin direm saat diskonek
            self.client.close()
            self.btn_connect.config(text="Connect")
            self.btn_manual_home.config(state="disabled")
            self.btn_start_auto.config(state="disabled")
            self.btn_pause_auto.config(state="disabled")
            self.btn_stop_auto.config(state="disabled")
            self.cb_ports.config(state="normal")
            self.log_message("Koneksi diputus.")

    # =========================================================
    # LOGIKA MODE MANUAL (JOGGING)
    # =========================================================
    def send_manual_weld(self):
        if self.is_welding:
            messagebox.showwarning("Peringatan", "Mesin sedang dalam mode AUTO!")
            return
            
        if self.client and self.client.is_socket_open():
            try:
                # Ambil jarak geser dari UI (Misal: 10 mm)
                x_mm_float = float(self.entry_man_x.get())
                y_mm_float = float(self.entry_man_y.get())
                
                # Sesuai firmware, manual jog Modbus minta mentahan mm utuh (integer)
                # Jika firmware minta x10 atau x100, ganti logic di sini.
                # Asumsi di firmware: long jogManX_mm = Modbus Value;
                x_val = int(x_mm_float) 
                y_val = int(y_mm_float)

                # Pecah 32-bit jadi 16-bit untuk Modbus
                x_low = x_val & 0xFFFF
                x_high = (x_val >> 16) & 0xFFFF
                y_low = y_val & 0xFFFF
                y_high = (y_val >> 16) & 0xFFFF

                # 1. Simpan Target Relatif di Holding Register 6, 7, 8, 9
                self.client.write_register(6, x_low)
                self.client.write_register(7, x_high)
                self.client.write_register(8, y_low)
                self.client.write_register(9, y_high)
                
                # 2. Tembak Coil Manual Jog (Asumsi Coil 00005 / Offset 4)
                self.client.write_coil(4, True)

                self.log_message(f"[MANUAL JOG] Geser sejauh -> X:{x_val}mm Y:{y_val}mm")
            except ValueError:
                messagebox.showerror("Error", "Format angka manual salah!")
            except Exception as e:
                self.log_message(f"ERROR MANUAL: {str(e)}")
                
    # =========================================================
    # FUNGSI PEMANTAUAN (POLLING) DARI STM32
    # =========================================================
    def live_monitor_loop(self):
        # Membaca Modbus secara continuous setiap 300ms
        while not self.stop_monitor:
            if self.client and self.client.is_socket_open():
                try:
                    # Baca 10 Input Register (Alamat 0 sampai 9) -> 30001 hingga 30010
                    # 0-1: Posisi X
                    # 2-3: Posisi Y
                    # 4-5: Posisi Z
                    # 6  : - Kosong -
                    # 7  : Index Kolom
                    # 8  : Index Baris
                    # 9  : Kode Alarm (IWDG)
                    result = self.client.read_input_registers(address=0, count=10)
                    if not result.isError():
                        
                        # 1. Rekonstruksi 32-bit Posisi (Dalam x100 mm)
                        x_pos = (result.registers[1] << 16) | result.registers[0]
                        y_pos = (result.registers[3] << 16) | result.registers[2]
                        
                        # Fix signed 32-bit (komplemen 2) jika motor bergerak minus
                        if x_pos > 2147483647: x_pos -= 4294967296
                        if y_pos > 2147483647: y_pos -= 4294967296
                        
                        # 2. Ambil Index Baterai (16-bit)
                        idx_kolom = result.registers[7]
                        idx_baris = result.registers[8]
                        
                        # 3. Update UI Utama (Menggunakan after agar thread aman)
                        self.root.after(0, self.update_dashboard, x_pos, y_pos, idx_kolom, idx_baris)
                        
                        # 4. Cek Alarm Watchdog di Register 9
                        alarm_code = result.registers[9]
                        if alarm_code == 1:
                            self.root.after(0, self.trigger_watchdog_alarm)
                            
                except Exception as e:
                    # Abaikan error timeout sesaat agar thread tidak crash
                    pass 
            time.sleep(0.3) # Rate refresh HMI (300 ms)

    def update_dashboard(self, x, y, idx_col, idx_row):
        # Kembalikan ke nilai milimeter (x100 dari firmware)
        posisi_x_mm = x / 100.0
        posisi_y_mm = y / 100.0
        
        # Update teks label di HMI
        self.lbl_pos_x.config(text=f"{posisi_x_mm:.2f} mm")
        self.lbl_pos_y.config(text=f"{posisi_y_mm:.2f} mm")
        self.lbl_idx_kolom.config(text=str(idx_col))
        self.lbl_idx_baris.config(text=str(idx_row))

    def trigger_watchdog_alarm(self):
        # Mencegah pop-up spam
        if not hasattr(self, 'iwdg_alert_shown') or not self.iwdg_alert_shown:
            self.iwdg_alert_shown = True
            
            # Reset UI Status otomatis
            self.is_welding = False
            self.is_paused = False
            self.btn_start_auto.config(state="normal")
            self.btn_pause_auto.config(state="normal")
            self.btn_stop_auto.config(state="normal")
            
            self.log_message("\n!!! ALARM: IWDG WATCHDOG RESET !!!")
            messagebox.showerror(
                "CRITICAL SYSTEM ERROR", 
                "WARNING: Mesin baru saja mengalami Freeze/Hang (Kemungkinan gangguan EMI)!\n\n"
                "Sistem Pengaman Watchdog (IWDG) Hardware telah mengambil alih dan me-reset mikrokontroler STM32 secara paksa untuk mencegah bahaya.\n\n"
                "Harap periksa grounding mesin atau jauhkan kabel USB dari trafo las!"
            )
            
            # Opsional: Kirim reset ke register 9 jika firmware mengizinkan tulis ke input reg
            # Tapi karena ini Input Register, biasanya firmware mereset dirinya sendiri setelah dibaca.

    # =========================================================
    # FUNGSI AKSI & PERINTAH (MODBUS WRITE)
    # =========================================================
    def trigger_manual_home(self):
        if self.is_welding:
            messagebox.showwarning("Peringatan", "Mesin sedang dalam mode AUTO. Hentikan dulu!")
            return
        if self.client and self.client.is_socket_open():
            try:
                # Tembak Coil Homing (Asumsi Offset 2)
                self.client.write_coil(2, True) 
                self.log_message("[AKSI] Perintah HOMING dikirim ke Mesin.")
            except Exception as e:
                self.log_message(f"ERROR HOMING: {str(e)}")

    def start_auto_weld(self):
        if self.client and self.client.is_socket_open():
            try:
                cols = int(self.entry_kolom.get())
                rows = int(self.entry_baris.get())
                pitch_x_float = float(self.entry_pitch_x.get())
                pitch_y_float = float(self.entry_pitch_y.get())
                
                # Konversi float ke integer (x10)
                pitch_x_int = int(pitch_x_float * 10)
                pitch_y_int = int(pitch_y_float * 10)
                
            except ValueError:
                messagebox.showerror("Error", "Format parameter mesin salah!")
                return
            
            try:
                # 1. Kirim Konfigurasi Parameter (Write Single Register)
                self.log_message("Mengirim parameter Matriks ke MCU...")
                self.client.write_register(0, cols)
                self.client.write_register(1, rows)
                self.client.write_register(2, pitch_x_int)
                self.client.write_register(3, pitch_y_int)

                # 2. Picu Tombol START (Coil 00001 / Offset 0)
                self.client.write_coil(0, True)
                
                self.is_welding = True
                self.is_paused = False
                self.iwdg_alert_shown = False # Reset alarm flag
                
                # Kunci Tombol UI
                self.btn_start_auto.config(state="disabled")
                self.btn_manual_home.config(state="disabled")
                self.btn_pause_auto.config(state="normal", text="PAUSE", bg="orange")
                self.btn_stop_auto.config(state="normal")
                
                self.log_message(f"\n=== MODE AUTO AKTIF: Mesin mengambil alih kontrol! ===")
                
            except Exception as e:
                self.log_message(f"ERROR PENGIRIMAN PARAMETER: {str(e)}")

    def toggle_pause(self):
        if self.is_welding and self.client and self.client.is_socket_open():
            self.is_paused = not self.is_paused
            try:
                if self.is_paused:
                    self.btn_pause_auto.config(text="RESUME", bg="yellow")
                    # Tembak Coil PAUSE ON (Offset 3)
                    self.client.write_coil(3, True) 
                    self.log_message("=== PERINTAH JEDA DIKIRIM ===")
                else:
                    self.btn_pause_auto.config(text="PAUSE", bg="orange")
                    # Tembak Coil PAUSE OFF
                    self.client.write_coil(3, False) 
                    self.log_message("=== PERINTAH LANJUT (RESUME) DIKIRIM ===")
            except Exception as e:
                self.log_message(f"ERROR PAUSE: {str(e)}")

    def stop_auto_weld(self):
        if self.client and self.client.is_socket_open():
            try:
                # Tembak Sinyal E-STOP ke STM32 (Coil Offset 1)
                self.client.write_coil(1, True)
                
                self.is_welding = False
                self.is_paused = False
                
                # Kembalikan Tombol UI
                self.btn_start_auto.config(state="normal")
                self.btn_manual_home.config(state="normal")
                self.btn_pause_auto.config(text="PAUSE", bg="orange", state="normal")
                self.btn_stop_auto.config(state="normal")
                
                self.log_message("!!! PERINTAH EMERGENCY STOP DIKIRIM !!!")
            except Exception as e:
                self.log_message(f"ERROR STOP: {str(e)}")

    def log_message(self, msg):
        self.txt_log.insert(tk.END, msg + "\n")
        self.txt_log.see(tk.END)

if __name__ == "__main__":
    root = tk.Tk()
    app = MesinLasHMI(root)
    root.mainloop()