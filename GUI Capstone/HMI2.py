import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import serial.tools.list_ports
from pymodbus.client import ModbusSerialClient

# =========================================================
# KELAS UTAMA: HMI MESIN LAS (MODBUS RTU)
# =========================================================
class MesinLasHMI:
    def __init__(self, root):
        self.root = root
        self.root.title("HMI Mesin Las Baterai Otomatis (Modbus RTU)")
        self.root.geometry("600x550")
        
        # Objek Modbus Client
        self.client = None
        
        # Variabel Status Pekerjaan Otomatis
        self.job_queue = []    
        self.is_welding = False 
        
        # Waktu perkiraan mesin bergerak dan mengelas (dalam milidetik)
        self.jeda_antar_titik_ms = 1500 

        self.setup_ui()
        self.update_com_ports()

    def setup_ui(self):
        # --- 1. Frame Koneksi ---
        frame_conn = ttk.LabelFrame(self.root, text="Koneksi Serial (Modbus RTU)")
        frame_conn.pack(padx=10, pady=5, fill="x")

        ttk.Label(frame_conn, text="Port:").grid(row=0, column=0, padx=5, pady=5)
        self.cb_ports = ttk.Combobox(frame_conn, width=15)
        self.cb_ports.grid(row=0, column=1, padx=5, pady=5)

        self.btn_refresh = ttk.Button(frame_conn, text="Refresh", command=self.update_com_ports)
        self.btn_refresh.grid(row=0, column=2, padx=5, pady=5)

        self.btn_connect = ttk.Button(frame_conn, text="Connect", command=self.toggle_connection)
        self.btn_connect.grid(row=0, column=3, padx=5, pady=5)

        # --- 2. Frame Parameter CAM (Grid Baterai) ---
        frame_cam = ttk.LabelFrame(self.root, text="Pengaturan Grid Baterai (Milimeter)")
        frame_cam.pack(padx=10, pady=5, fill="x")

        # Baris 1: Jumlah Kolom & Baris
        ttk.Label(frame_cam, text="Jumlah Baterai X (Kolom):").grid(row=0, column=0, padx=5, pady=5, sticky="e")
        self.entry_kolom = ttk.Entry(frame_cam, width=8)
        self.entry_kolom.grid(row=0, column=1, padx=5, pady=5)
        self.entry_kolom.insert(0, "4")

        ttk.Label(frame_cam, text="Jumlah Baterai Y (Baris):").grid(row=0, column=2, padx=5, pady=5, sticky="e")
        self.entry_baris = ttk.Entry(frame_cam, width=8)
        self.entry_baris.grid(row=0, column=3, padx=5, pady=5)
        self.entry_baris.insert(0, "3")

        # Baris 2: Jarak Pitch
        ttk.Label(frame_cam, text="Pitch X (Jarak mm):").grid(row=1, column=0, padx=5, pady=5, sticky="e")
        self.entry_pitch_x = ttk.Entry(frame_cam, width=8)
        self.entry_pitch_x.grid(row=1, column=1, padx=5, pady=5)
        self.entry_pitch_x.insert(0, "18.5")

        ttk.Label(frame_cam, text="Pitch Y (Jarak mm):").grid(row=1, column=2, padx=5, pady=5, sticky="e")
        self.entry_pitch_y = ttk.Entry(frame_cam, width=8)
        self.entry_pitch_y.grid(row=1, column=3, padx=5, pady=5)
        self.entry_pitch_y.insert(0, "18.5")

        # Tombol Mulai Sekuens
        self.btn_start_auto = ttk.Button(frame_cam, text="START AUTO WELD", command=self.start_auto_weld, state="disabled")
        self.btn_start_auto.grid(row=2, column=0, columnspan=4, pady=10)

        # --- 3. Frame Log Terminal ---
        frame_log = ttk.LabelFrame(self.root, text="Status & Log Mesin")
        frame_log.pack(padx=10, pady=5, fill="both", expand=True)

        self.txt_log = scrolledtext.ScrolledText(frame_log, wrap=tk.WORD, height=10)
        self.txt_log.pack(padx=5, pady=5, fill="both", expand=True)

    # --- KONTROL UI & KONEKSI ---
    def update_com_ports(self):
        ports = serial.tools.list_ports.comports()
        self.cb_ports['values'] = [port.device for port in ports]
        if ports:
            self.cb_ports.current(0)

    def toggle_connection(self):
        if self.client is None or not self.client.is_socket_open():
            port = self.cb_ports.get()
            if port:
                # Menginisialisasi Pymodbus Client
                self.client = ModbusSerialClient(port=port, baudrate=115200, timeout=0.5)
                
                if self.client.connect():
                    self.btn_connect.config(text="Disconnect")
                    self.btn_start_auto.config(state="normal")
                    self.cb_ports.config(state="disabled")
                    self.log_message(f"Terhubung ke {port} via Modbus RTU.")
                else:
                    messagebox.showerror("Error", "Gagal membuka port serial!")
        else:
            self.client.close()
            self.btn_connect.config(text="Connect")
            self.btn_start_auto.config(state="disabled")
            self.cb_ports.config(state="normal")
            self.is_welding = False
            self.log_message("Koneksi diputus.")

    # --- ALGORITMA CAM & SEQUENCER ---
    def generate_grid_jobs(self, cols, rows, pitch_x, pitch_y):
        steps_per_mm = 160 # Matematika: Sabuk GT2 Pulley 20T, Motor 1/32 Step
        jobs = []
        
        for r in range(rows):
            # Algoritma Zig-Zag
            col_sequence = range(cols) if r % 2 == 0 else range(cols - 1, -1, -1)
            
            for c in col_sequence:
                x_mm = c * pitch_x
                y_mm = r * pitch_y
                
                x_step = int(x_mm * steps_per_mm)
                y_step = int(y_mm * steps_per_mm)
                
                # Masukkan ke list dalam bentuk Tuple angka, bukan string
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
                messagebox.showerror("Error", "Format input angka salah!")
                return
            
            # Hasilkan daftar rute titik absolut (Tuple Integer)
            self.job_queue = self.generate_grid_jobs(cols, rows, pitch_x, pitch_y)
            self.is_welding = True
            
            self.log_message(f"=== MEMULAI PEKERJAAN: {len(self.job_queue)} TITIK LAS ===")
            
            # Eksekusi titik pertama
            self.send_next_job()

    def send_next_job(self):
        if self.is_welding and len(self.job_queue) > 0:
            
            # Ambil target koordinat X dan Y (Bentuk Tuple)
            x_step, y_step = self.job_queue.pop(0) 
            slave_id = 1

            try:
                # 1. PECAH DATA 32-BIT KE 16-BIT (MODBUS STANDAR)
                x_low  = x_step & 0xFFFF
                x_high = (x_step >> 16) & 0xFFFF
                y_low  = y_step & 0xFFFF
                y_high = (y_step >> 16) & 0xFFFF

                # 2. TULIS KOORDINAT KE HOLDING REGISTERS STM32 (Alamat 0 - 3)
                self.client.write_register(0, x_low, slave=slave_id)
                self.client.write_register(1, x_high, slave=slave_id)
                self.client.write_register(2, y_low, slave=slave_id)
                self.client.write_register(3, y_high, slave=slave_id)

                # 3. TRIGGER START WELD (Coil 00001: ON lalu OFF instan)
                self.client.write_coil(0, True, slave=slave_id)
                self.client.write_coil(0, False, slave=slave_id)

                self.log_message(f"Titik X:{x_step} Y:{y_step} dieksekusi (Sisa: {len(self.job_queue)})")

                # 4. LOOPING OTOMATIS: Jadwalkan pengiriman titik berikutnya 
                # setelah jeda yang cukup untuk mesin bergerak dan mengelas
                self.root.after(self.jeda_antar_titik_ms, self.send_next_job)

            except Exception as e:
                self.log_message(f"ERROR MODBUS: {str(e)}")
                self.is_welding = False

        elif self.is_welding and len(self.job_queue) == 0:
            self.log_message("=== SEMUA TITIK BATERAI SELESAI DILAS! ===")
            self.is_welding = False

    def log_message(self, msg):
        self.txt_log.insert(tk.END, msg + "\n")
        self.txt_log.see(tk.END)

if __name__ == "__main__":
    root = tk.Tk()
    app = MesinLasHMI(root)
    root.mainloop()