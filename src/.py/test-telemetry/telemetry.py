import serial
import time
import struct
import csv
import os
import msvcrt
from datetime import datetime
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.console import Console, Group
from rich import box
from rich.text import Text

# --- 設定 ---
SERIAL_PORT = 'COM11'
BAUD_RATE = 38400
LOG_DIR = "telemetry_logs"

# --- LogManagerクラス ---
class LogManager:
    def __init__(self):
        if not os.path.exists(LOG_DIR):
            os.makedirs(LOG_DIR)
        self.file = None
        self.writer = None
        self.current_filename = None
        self.create_new_log()

    def _get_header(self):
        return ["PC_Timestamp", "Protocol", "LQI", "dBm", "Rocket_Millis", "Latitude", "Longitude", "Pressure", "Raw_Message"]

    def create_new_log(self, user_suffix=None):
        """
        新規ログファイルを作成する
        user_suffixがある場合: log_YYYYMMDD_HHMMSS_suffix.csv
        ない場合: log_YYYYMMDD_HHMMSS.csv
        """
        if self.file: self.file.close()
        
        # 共通のタイムスタンプ
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        if user_suffix:
            # 入力に拡張子が含まれていたら取り除く
            base_name = user_suffix
            if base_name.lower().endswith('.csv'):
                base_name = base_name[:-4]
            
            # 日時 + 指定名
            name = f"log_{timestamp}_{base_name}.csv"
        else:
            # 日時のみ
            name = f"log_{timestamp}.csv"
            
        self.current_filename = os.path.join(LOG_DIR, name)
        self.file = open(self.current_filename, 'w', newline='', encoding='utf-8')
        self.writer = csv.writer(self.file)
        self.writer.writerow(self._get_header())
        self.file.flush()
        return self.current_filename

    def rename_current_log(self, new_name):
        # renameの場合はユーザーが完全に名前を制御したい場合もあるため、
        # ここでは日時の強制付与は行わず、指定した名前に変更します。
        # (必要であればここも変更可能です)
        if not self.file: return "No active log file."
        self.file.close()
        
        if not new_name.endswith('.csv'): new_name += '.csv'
        
        old_path = self.current_filename
        new_path = os.path.join(LOG_DIR, new_name)
        
        try:
            os.rename(old_path, new_path)
            self.current_filename = new_path
            self.file = open(self.current_filename, 'a', newline='', encoding='utf-8')
            self.writer = csv.writer(self.file)
            return f"Renamed to {new_name}"
        except OSError as e:
            self.file = open(old_path, 'a', newline='', encoding='utf-8')
            self.writer = csv.writer(self.file)
            return f"Rename failed: {e}"

    def write_data(self, row_data):
        if self.writer:
            self.writer.writerow(row_data)
            self.file.flush()
            
    def close(self):
        if self.file: self.file.close()

# --- パケット解析 ---
def parse_packet(ser):
    try:
        head = ser.read(1)
        if not head: return None, None, None
        if head == b';':
            line = ser.readline()
            try:
                full_str = (b';' + line).decode('utf-8', errors='ignore').strip()
                parts = full_str.split(';')
                if len(parts) > 6: return int(parts[5]), parts[7], "Extended (Mode E)"
            except: pass
        elif head == b':':
            line = ser.readline()
            full_line = b':' + line
            try:
                data_str = full_line[1:].decode('utf-8').strip()
                if len(data_str) >= 8:
                    lqi = int(data_str[2:4], 16)
                    payload = bytes.fromhex(data_str[6:-2]).decode('utf-8', errors='ignore')
                    return lqi, payload, "ASCII (Mode A)"
            except: pass
        elif head == b'\xA5':
            head2 = ser.read(1)
            if head2 == b'\x5A':
                len_bytes = ser.read(2)
                if len(len_bytes) == 2:
                    length = struct.unpack('>H', len_bytes)[0] & 0x7FFF
                    body = ser.read(length)
                    if len(body) == length:
                        return body[1], body[3:-1].decode('utf-8', errors='ignore'), "BINARY (Mode B)"
        else:
            line = head + ser.readline()
            return 0, line.decode('utf-8', errors='ignore').strip(), "RAW TEXT"
    except: pass
    return None, None, None

# --- UI生成 ---
def generate_interface(data, log_filename, is_paused, status_msg=""):
    table = Table(box=box.ROUNDED, expand=True, show_header=False, show_lines=True)
    
    table.add_column("Key", style="bold white", width=18, justify="right")
    table.add_column("Value", style="bold cyan", justify="left")

    lqi = data.get("lqi", 0)
    msg = data.get("msg", "")
    mode = data.get("mode", "-")
    
    parts = msg.split(',') if msg else []
    val_millis = parts[0] if len(parts) > 0 else "-"
    val_lat    = parts[1] if len(parts) > 1 else "-"
    val_lon    = parts[2] if len(parts) > 2 else "-"
    val_press  = parts[3] if len(parts) > 3 else "-"

    dbm = (7 * lqi - 1970) / 20 if lqi > 0 else -999

    if is_paused:
        status_text = "[bold white on red] PAUSED (Not Writing) [/]"
    else:
        status_text = "[bold white on green] ● RECORDING [/]"
    
    table.add_row("REC STATUS", status_text)
    table.add_row("Protocol Mode", mode)
    table.add_row("Rocket Uptime", f"{val_millis} ms")
    table.add_row("Latitude", val_lat)
    table.add_row("Longitude", val_lon)
    table.add_row("Pressure", f"{val_press} hPa")
    
    lqi_color = "bright_green" if lqi > 150 else ("bright_yellow" if lqi > 100 else "bright_red")
    table.add_row("Signal (LQI)", f"[bold {lqi_color}]{lqi}[/]  [dim white]({dbm:.1f} dBm)[/]")
    
    file_display = os.path.basename(log_filename)
    table.add_row("Log File", f"[u white]{file_display}[/]")
    
    if status_msg:
        table.add_row("COMMAND MSG", f"[bold yellow] {status_msg} [/]")

    help_table = Table(box=box.ROUNDED, expand=True, title="[italic]Keyboard Controls[/]", border_style="dim white")
    help_table.add_column("Command", style="bold green", width=20)
    help_table.add_column("Description", style="white")

    help_table.add_row("pause", "Stop writing to CSV")
    help_table.add_row("resume", "Resume writing to CSV")
    help_table.add_row("new", "Restart (log_Time.csv)")
    help_table.add_row("new name", "Restart (log_Time_name.csv)")
    help_table.add_row("exit", "Quit program")

    content = Group(table, Text(" ", style="dim"), help_table)
    return Panel(content, title="🚀 Rocket Telemetry System", border_style="blue")

def main():
    console = Console()
    logger = LogManager()
    
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        console.print(f"[bold green]Connected to {SERIAL_PORT}[/]")
    except Exception as e:
        console.print(f"[bold red]Connection Error: {e}[/]")
        return

    telemetry_data = {"lqi": 0, "msg": "", "mode": "Waiting..."}
    status_message = ""
    is_paused = False

    try:
        with Live(generate_interface(telemetry_data, logger.current_filename, is_paused), refresh_per_second=10, auto_refresh=True) as live:
            while True:
                if msvcrt.kbhit():
                    key = msvcrt.getch().decode().lower()
                    if key == 'c':
                        live.stop()
                        console.print("\n[bold yellow on blue]--- Command Mode ---[/]")
                        console.print("Commands: pause, resume, new, new [name], exit")
                        
                        try:
                            raw_input = input(">> ").strip()
                            cmd_input = raw_input.split()
                            
                            if len(cmd_input) > 0:
                                cmd = cmd_input[0].lower()
                                arg = " ".join(cmd_input[1:]).strip("'\"") if len(cmd_input) > 1 else None

                                if cmd == 'pause':
                                    is_paused = True
                                    status_message = "Recording PAUSED."
                                    console.print("[yellow]Paused recording.[/]")

                                elif cmd == 'resume':
                                    is_paused = False
                                    status_message = "Recording RESUMED."
                                    console.print("[green]Resumed recording.[/]")

                                elif cmd == 'new':
                                    new_path = logger.create_new_log(arg)
                                    is_paused = False 
                                    status_message = f"New Log: {os.path.basename(new_path)}"
                                    console.print(f"[green]Started new log: {new_path}[/]")

                                elif cmd == 'exit':
                                    break
                                else:
                                    console.print(f"[red]Unknown: {cmd}[/]")
                            time.sleep(1)
                        except Exception as e:
                            console.print(f"[red]Error: {e}[/]")
                        
                        live.start()

                if ser.in_waiting > 0:
                    lqi, payload, mode = parse_packet(ser)
                    if payload is not None:
                        telemetry_data["lqi"] = lqi
                        telemetry_data["msg"] = payload
                        telemetry_data["mode"] = mode
                        
                        live.update(generate_interface(telemetry_data, logger.current_filename, is_paused, status_message))
                        
                        if not is_paused:
                            parts = payload.split(',')
                            csv_data = [
                                datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
                                mode, lqi, f"{(7 * lqi - 1970) / 20 if lqi > 0 else -999:.1f}"
                            ]
                            csv_data.extend(parts[:4] + [""] * (4 - len(parts[:4])))
                            csv_data.append(payload)
                            logger.write_data(csv_data)

                time.sleep(0.005)

    except KeyboardInterrupt:
        console.print("\n[bold red]Stopped logging.[/]")
    finally:
        ser.close()
        logger.close()
        console.print(f"[bold green]Logs saved in {LOG_DIR}[/]")

if __name__ == "__main__":
    main()