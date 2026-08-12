import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import serial.tools.list_ports
from pymodbus.client import ModbusSerialClient
import threading
import time

class ModbusTestHMI:
    def __init__(self, root):
        self.root = root
        self.root.title("HMI Pengujian Modbus (Berdasarkan Tipe Address)")
        self.root.geometry("600x650")
        
        # Pengaturan jumlah input form per tab
        self.NUM_POLL = 10
        self.NUM_SEND = 3
        
        self.client = None
        self.monitor_thread = None
        self.stop_monitor = False

        # Dictionary untuk menyimpan semua referensi widget secara dinamis dan rapi
        self.ui_refs = {
            'HR':   {'poll_addr': [], 'poll_val': [], 'send_addr': [], 'send_val': [], 'send_btn': []}, # Holding Register
            'COIL': {'poll_addr': [], 'poll_val': [], 'send_addr': [], 'send_val': [], 'send_btn': []}, # Coil
            'IR':   {'poll_addr': [], 'poll_val': []}, # Input Register (Read Only)
            'DI':   {'poll_addr': [], 'poll_val': []}  # Discrete Input (Read Only)
        }

        self.setup_ui()
        self.update_com_ports()

    def setup_ui(self):
        # ==========================================
        # FRAME KONEKSI SERIAL
        # ==========================================
        frame_conn = ttk.LabelFrame(self.root, text=" Koneksi Serial (Modbus RTU) ")
        frame_conn.pack(padx=10, pady=5, fill="x")

        ttk.Label(frame_conn, text="Port:").grid(row=0, column=0, padx=5, pady=5)
        self.cb_ports = ttk.Combobox(frame_conn, width=15)
        self.cb_ports.grid(row=0, column=1, padx=5, pady=5)

        self.btn_refresh = ttk.Button(frame_conn, text="Refresh", command=self.update_com_ports)
        self.btn_refresh.grid(row=0, column=2, padx=5, pady=5)

        self.btn_connect = ttk.Button(frame_conn, text="Connect", command=self.toggle_connection)
        self.btn_connect.grid(row=0, column=3, padx=5, pady=5)

        # ==========================================
        # SISTEM TAB (NOTEBOOK) UNTUK TIPE MODBUS
        # ==========================================
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(padx=10, pady=10, fill="both", expand=True)

        tab_hr   = ttk.Frame(self.notebook)
        tab_coil = ttk.Frame(self.notebook)
        tab_ir   = ttk.Frame(self.notebook)
        tab_di   = ttk.Frame(self.notebook)

        self.notebook.add(tab_hr, text="Holding Reg (4x) [R/W]")
        self.notebook.add(tab_coil, text="Coils (0x) [R/W]")
        self.notebook.add(tab_ir, text="Input Reg (3x) [Read Only]")
        self.notebook.add(tab_di, text="Discrete (1x) [Read Only]")

        # Membangun UI untuk masing-masing tab secara otomatis
        self.build_tab(tab_hr, 'HR', has_send=True)
        self.build_tab(tab_coil, 'COIL', has_send=True)
        self.build_tab(tab_ir, 'IR', has_send=False)
        self.build_tab(tab_di, 'DI', has_send=False)

        # ==========================================
        # FRAME LOG TERMINAL
        # ==========================================
        frame_log = ttk.LabelFrame(self.root, text=" Log Aktivitas ")
        frame_log.pack(padx=10, pady=5, fill="x")

        self.txt_log = scrolledtext.ScrolledText(frame_log, wrap=tk.WORD, height=8)
        self.txt_log.pack(padx=5, pady=5, fill="both")

    def build_tab(self, parent, type_key, has_send):
        """Fungsi helper untuk me-render UI secara dinamis pada tab tertentu"""
        # 1. Bagian Monitoring (Polling)
        fm = ttk.LabelFrame(parent, text=f" Monitoring {type_key} (Polling 500ms) ")
        fm.pack(padx=10, pady=10, fill="x")
        
        for i in range(self.NUM_POLL):
            ttk.Label(fm, text=f"Alamat {i}:").grid(row=i, column=0, padx=5, pady=2, sticky="e")
            
            ent = ttk.Entry(fm, width=10)
            ent.grid(row=i, column=1, padx=5, pady=2)
            ent.insert(0, str(i)) 
            self.ui_refs[type_key]['poll_addr'].append(ent)
            
            ttk.Label(fm, text="Val:").grid(row=i, column=2, padx=5, pady=2, sticky="e")
            
            lbl = ttk.Label(fm, text="0", font=("Arial", 10, "bold"), foreground="blue")
            lbl.grid(row=i, column=3, padx=5, pady=2)
            self.ui_refs[type_key]['poll_val'].append(lbl)

        # 2. Bagian Send Data (Hanya untuk Coils & Holding Reg)
        if has_send:
            fs = ttk.LabelFrame(parent, text=f" Send Data {type_key} ")
            fs.pack(padx=10, pady=10, fill="x")
            
            for i in range(self.NUM_SEND):
                ttk.Label(fs, text=f"Alamat {i}:").grid(row=i, column=0, padx=5, pady=2, sticky="e")
                
                ent_a = ttk.Entry(fs, width=10)
                ent_a.grid(row=i, column=1, padx=5, pady=2)
                # Ubah menjadi seperti ini agar dimulai dari 0:
                ent_a.insert(0, str(i))
                self.ui_refs[type_key]['send_addr'].append(ent_a)

                ttk.Label(fs, text="Val:").grid(row=i, column=2, padx=5, pady=2, sticky="e")
                
                ent_v = ttk.Entry(fs, width=10)
                ent_v.grid(row=i, column=3, padx=5, pady=2)
                ent_v.insert(0, "1" if type_key == 'COIL' else "100") # Default nilai coil adalah bit (0/1)
                self.ui_refs[type_key]['send_val'].append(ent_v)

                # Tombol Send memanggil fungsi execute_send beserta Tipe-nya
                btn = ttk.Button(fs, text="SEND", state="disabled", 
                                command=lambda t=type_key, idx=i: self.execute_send(t, idx))
                btn.grid(row=i, column=4, padx=5, pady=2)
                self.ui_refs[type_key]['send_btn'].append(btn)

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
                # [REVISI]: Tambahkan timeout=0.1 agar HMI tidak beku (lag) jika ada alamat error
                self.client = ModbusSerialClient(port=port, baudrate=115200, timeout=0.1)
                if self.client.connect():
                    self.btn_connect.config(text="Disconnect")
                    self.cb_ports.config(state="disabled")
                    self.set_send_buttons_state("normal")
                    self.log_message(f"Terhubung ke {port} via Modbus RTU.")
                    
                    self.stop_monitor = False
                    self.monitor_thread = threading.Thread(target=self.live_monitor_loop, daemon=True)
                    self.monitor_thread.start()
                else:
                    messagebox.showerror("Error", "Gagal membuka port serial!")
        else:
            self.stop_monitor = True
            time.sleep(0.5) 
            self.client.close()
            
            self.btn_connect.config(text="Connect")
            self.cb_ports.config(state="normal")
            self.set_send_buttons_state("disabled")
            self.log_message("Koneksi diputus.")

    def set_send_buttons_state(self, state):
        """Helper untuk mengaktifkan/mematikan semua tombol SEND di semua Tab"""
        for t_key in ['HR', 'COIL']:
            for btn in self.ui_refs[t_key]['send_btn']:
                btn.config(state=state)

    # =========================================================
    # FUNGSI PEMANTAUAN (POLLING 4 TIPE MODBUS)
    # =========================================================
    def live_monitor_loop(self):
        interval = 0.5 # Target kecepatan polling: 500ms
        
        while not self.stop_monitor:
            start_time = time.time() # 1. Catat waktu mulai
            
            if self.client and self.client.is_socket_open():
                # Baca keempat tipe register
                self.poll_group('HR', self.client.read_holding_registers, is_bit=False)
                self.poll_group('COIL', self.client.read_coils, is_bit=True)
                self.poll_group('IR', self.client.read_input_registers, is_bit=False)
                self.poll_group('DI', self.client.read_discrete_inputs, is_bit=True)
                
            # 2. Hitung berapa lama proses baca Modbus barusan
            elapsed_time = time.time() - start_time
            
            # 3. Tidur hanya dari sisa waktu (Misal baca memakan waktu 0.1s, maka tidur 0.4s)
            sleep_time = interval - elapsed_time
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                # Jika proses baca sedang berat dan melebihi 500ms, jangan tidur lama-lama
                time.sleep(0.01)

    def poll_group(self, type_key, read_function, is_bit):
        """Membaca daftar alamat dari GUI secara efisien"""
        for i in range(self.NUM_POLL):
            if self.stop_monitor: return
            try:
                addr_str = self.ui_refs[type_key]['poll_addr'][i].get()
                
                # [REVISI]: Abaikan (Skip) jika kotak alamat dibiarkan kosong oleh user
                if not addr_str.strip():
                    continue 
                    
                address = int(addr_str)
                res = read_function(address=address, count=1)
                
                if not res.isError():
                    val = res.bits[0] if is_bit else res.registers[0]
                    val_str = str(int(val)) if is_bit else str(val) 
                    
                    # [REVISI SUPER PENTING]: Hanya paksa Tkinter update JIKA nilai berubah!
                    # Ini mencegah CPU spamming yang membuat HMI lag.
                    current_text = self.ui_refs[type_key]['poll_val'][i].cget("text")
                    if current_text != val_str:
                        self.root.after(0, self.ui_refs[type_key]['poll_val'][i].config, {"text": val_str})
                        
            except Exception:
                pass

    # =========================================================
    # FUNGSI SEND DATA DINAMIS
    # =========================================================
    def execute_send(self, type_key, idx):
        if self.client and self.client.is_socket_open():
            try:
                address = int(self.ui_refs[type_key]['send_addr'][idx].get())
                value = int(self.ui_refs[type_key]['send_val'][idx].get())
                
                if type_key == 'HR':
                    self.client.write_register(address, value)
                elif type_key == 'COIL':
                    # Modbus Write Coil butuh nilai Boolean
                    bool_val = True if value > 0 else False
                    self.client.write_coil(address, bool_val)
                    
                self.log_message(f"SEND [{type_key}] -> Target Alamat: {address}, Value: {value}")
                
            except ValueError:
                messagebox.showerror("Error", "Format angka pada Address atau Value salah!")
            except Exception as e:
                self.log_message(f"ERROR PENGIRIMAN [{type_key}]: {str(e)}")

    def log_message(self, msg):
        self.txt_log.insert(tk.END, msg + "\n")
        self.txt_log.see(tk.END)

if __name__ == "__main__":
    root = tk.Tk()
    app = ModbusTestHMI(root)
    root.mainloop()