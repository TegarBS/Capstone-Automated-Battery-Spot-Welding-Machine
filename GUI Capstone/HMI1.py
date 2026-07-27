import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import serial.tools.list_ports
from pymodbus.client import ModbusSerialClient

# =========================================================
# KELAS UTAMA: HMI MESIN LAS MANUAL (MODBUS RTU)
# =========================================================
class MesinLasHMI:
    def __init__(self, root):
        self.root = root
        self.root.title("HMI Manual Jog - Mesin Las (Modbus RTU)")
        self.root.geometry("550x450")
        
        # Objek Modbus Client (Menggantikan SerialManager lama)
        self.client = None

        self.setup_ui()
        self.update_com_ports()

    def setup_ui(self):
        # --- Frame Koneksi ---
        frame_conn = ttk.LabelFrame(self.root, text="Koneksi Serial (Modbus RTU)")
        frame_conn.pack(padx=10, pady=5, fill="x")

        ttk.Label(frame_conn, text="Port:").grid(row=0, column=0, padx=5, pady=5)
        self.cb_ports = ttk.Combobox(frame_conn, width=15)
        self.cb_ports.grid(row=0, column=1, padx=5, pady=5)

        self.btn_refresh = ttk.Button(frame_conn, text="Refresh", command=self.update_com_ports)
        self.btn_refresh.grid(row=0, column=2, padx=5, pady=5)

        self.btn_connect = ttk.Button(frame_conn, text="Connect", command=self.toggle_connection)
        self.btn_connect.grid(row=0, column=3, padx=5, pady=5)

        # --- Frame Pengiriman Koordinat Tunggal ---
        frame_ctrl = ttk.LabelFrame(self.root, text="Kirim Koordinat & Trigger Las")
        frame_ctrl.pack(padx=10, pady=5, fill="x")

        ttk.Label(frame_ctrl, text="Target X:").grid(row=0, column=0, padx=5, pady=5)
        self.entry_x = ttk.Entry(frame_ctrl, width=10)
        self.entry_x.grid(row=0, column=1, padx=5, pady=5)
        self.entry_x.insert(0, "3200")

        ttk.Label(frame_ctrl, text="Target Y:").grid(row=0, column=2, padx=5, pady=5)
        self.entry_y = ttk.Entry(frame_ctrl, width=10)
        self.entry_y.grid(row=0, column=3, padx=5, pady=5)
        self.entry_y.insert(0, "3200")

        self.btn_send = ttk.Button(frame_ctrl, text="KIRIM & LAS", command=self.send_coordinate, state="disabled")
        self.btn_send.grid(row=0, column=4, padx=10, pady=5)

        # --- Frame Penerimaan Data & Log ---
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
                # Inisialisasi koneksi Modbus RTU
                self.client = ModbusSerialClient(port=port, baudrate=115200, timeout=1)
                
                if self.client.connect():
                    self.btn_connect.config(text="Disconnect")
                    self.btn_send.config(state="normal")
                    self.cb_ports.config(state="disabled")
                    self.log_message(f"Terhubung ke {port} via Modbus RTU.")
                else:
                    messagebox.showerror("Error", "Gagal membuka port serial!")
        else:
            self.client.close()
            self.btn_connect.config(text="Connect")
            self.btn_send.config(state="disabled")
            self.cb_ports.config(state="normal")
            self.log_message("Koneksi diputus.")

    # --- PENGIRIMAN DATA MODBUS ---
    def send_coordinate(self):
        if self.client and self.client.is_socket_open():
            try:
                # 1. Ambil nilai angka dari kotak input
                x_val = int(self.entry_x.get())
                y_val = int(self.entry_y.get())
                
                slave_id = 1

                # 2. Pecah nilai 32-bit (long) menjadi Low Word dan High Word 16-bit
                x_low  = x_val & 0xFFFF
                x_high = (x_val >> 16) & 0xFFFF
                y_low  = y_val & 0xFFFF
                y_high = (y_val >> 16) & 0xFFFF

                # 3. Tulis parameter koordinat ke Holding Registers (40001 - 40004)
                # Fungsi ini otomatis menunggu ACK dari STM32 sebelum lanjut ke baris berikutnya
                self.client.write_register(0, x_low, slave=slave_id)
                self.client.write_register(1, x_high, slave=slave_id)
                self.client.write_register(2, y_low, slave=slave_id)
                self.client.write_register(3, y_high, slave=slave_id)

                # 4. Berikan Pulsa Eksekusi ke Coil 00001 (ON lalu OFF instan)
                self.client.write_coil(0, True, slave=slave_id)
                self.client.write_coil(0, False, slave=slave_id)

                self.log_message(f"[Modbus RTU] Sukses Kirim & Trigger Mesin ke X:{x_val} Y:{y_val}")

            except ValueError:
                messagebox.showerror("Error", "Masukkan angka koordinat step yang valid!")
            except Exception as e:
                self.log_message(f"ERROR: Komunikasi Modbus gagal! ({str(e)})")
        else:
            messagebox.showwarning("Peringatan", "Koneksi serial belum tersambung!")

    def log_message(self, msg):
        self.txt_log.insert(tk.END, msg + "\n")
        self.txt_log.see(tk.END)

if __name__ == "__main__":
    root = tk.Tk()
    app = MesinLasHMI(root)
    root.mainloop()