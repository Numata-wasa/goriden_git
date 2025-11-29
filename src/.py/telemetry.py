import serial
import time
import math
import random
import csv
import os
from datetime import datetime
from rich.live import Live
from rich.layout import Layout
from rich.panel import Panel
from rich.table import Table
from rich.console import Console
from rich import box

# --- 設定 ---
# Windowsなら 'COMx', Macなら '/dev/tty.usbmodem...'
SERIAL_PORT = 'COM3'  
BAUD_RATE = 115200
USE_DUMMY_DATA = True  # テスト時はTrue, 実機接続時はFalse
LOG_DIR = "telemetry_logs" 

# キャリブレーション設定 (起動直後の気圧を基準0mとする)
CALIBRATION_SAMPLES = 20

# --- 高度計算ロジック ---
def calculate_altitude(current_pressure, base_pressure):
    if base_pressure is None or base_pressure == 0:
        return 0.0
    try:
        # 国際標準大気モデルによる相対高度計算
        altitude = 44330 * (1.0 - math.pow(current_pressure / base_pressure, 0.1903))
        return altitude
    except:
        return 0.0

# --- ログ管理クラス ---
class TelemetryLogger:
    def __init__(self):
        if not os.path.exists(LOG_DIR):
            os.makedirs(LOG_DIR)
        
        filename = datetime.now().strftime("log_%Y%m%d_%H%M%S.csv")
        self.filepath = os.path.join(LOG_DIR, filename)
        
        # ヘッダー (Excel等で開いたときにわかりやすいように)
        with open(self.filepath, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                "MCU_Millis", "PC_Timestamp", 
                "Latitude", "Longitude", "Pressure(hPa)", "Temp(C)", 
                "AccX", "AccY", "AccZ", 
                "GyrX", "GyrY", "GyrZ", 
                "MagX", "MagY", "MagZ", 
                "RSSI(dBm)", "Calc_Altitude(m)"
            ])
            
    def log(self, millis_val, data_list, rssi_val, altitude):
        with open(self.filepath, mode='a', newline='') as f:
            writer = csv.writer(f)
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            # data_listにはセンサー値(13個)が入っている
            row = [millis_val, timestamp] + data_list + [rssi_val, altitude]
            writer.writerow(row)

# --- ダミーデータ生成 (テスト用) ---
class DummySerial:
    def __init__(self):
        self.start_time = time.time()
        self.base_press = 1013.25

    def readline(self):
        time.sleep(0.1) # 10Hz
        elapsed = time.time() - self.start_time
        millis = int(elapsed * 1000)
        
        lat = 35.6895
        lon = 139.6917
        
        # 気圧変化 (5秒後に上昇開始)
        if elapsed < 5:
            press = self.base_press + random.uniform(-0.05, 0.05)
        else:
            press = self.base_press - (elapsed - 5) * 0.5 + random.uniform(-0.1, 0.1)

        temp = 25.0
        ax, ay, az = 0.1, 0.2, 9.8
        gx, gy, gz = 0.0, 0.0, 0.0
        mx, my, mz = 30.0, 10.0, -40.0
        
        # RSSI (-60dBmくらいから距離に応じて下がるシミュレーション)
        rssi = -60 - int(elapsed * 0.5) + random.randint(-2, 2)
        
        # Picoが送ってくる形式と同じCSV文字列を作成
        # millis, lat, lon, press, temp, ax, ay, az, gx, gy, gz, mx, my, mz, rssi
        data_str = f"{millis},{lat},{lon},{press},{temp},{ax},{ay},{az},{gx},{gy},{gz},{mx},{my},{mz},{rssi}\n"
        return data_str.encode('utf-8')
    
    def close(self):
        pass

# --- UIパーツ作成 ---
def make_header(is_calibrating, log_path, millis_val, rssi_val):
    grid = Table.grid(expand=True)
    grid.add_column(justify="left", ratio=1)
    grid.add_column(justify="right")
    
    # ステータス表示
    if is_calibrating:
        status_text = "[bold yellow]CALIBRATING (Do not move)...[/bold yellow]"
    else:
        status_text = "[bold green]RECORDING[/bold green]"
    
    # 時間表示
    mcu_time = f"{millis_val / 1000.0:.1f}s" if millis_val is not None else "--"
    
    # RSSIの色分け
    if rssi_val > -80:
        rssi_style = "bold green"
    elif rssi_val > -100:
        rssi_style = "bold yellow"
    else:
        rssi_style = "bold red"
        
    rssi_disp = f"RSSI: [{rssi_style}]{rssi_val} dBm[/{rssi_style}]"

    grid.add_row(
        f"[b white]GS TELEMETRY[/b white] | {status_text}",
        f"Time: [cyan]{mcu_time}[/cyan] | {rssi_disp}"
    )
    grid.add_row(
        f"[dim]Log: {os.path.basename(log_path)}[/dim]",
        f"[dim]{datetime.now().strftime('%H:%M:%S')}[/dim]"
    )
    return Panel(grid, style="white on blue")

def make_env_panel(press, temp, alt, base_press):
    table = Table(box=None, expand=True)
    table.add_column("Sensor", style="magenta")
    table.add_column("Value", justify="right", style="yellow")
    
    base_disp = f"{base_press:.1f}" if base_press else "Wait"
    
    table.add_row("Pressure", f"{press:.1f} hPa")
    table.add_row("Base Press", f"{base_disp} hPa")
    table.add_row("Temp", f"{temp:.1f} °C")
    table.add_row("Altitude", f"[bold white on red]{alt:.1f} m[/bold white on red]")
    
    return Panel(table, title="Environment", border_style="yellow")

def make_gps_panel(lat, lon):
    table = Table(box=None, expand=True)
    table.add_column("Type", style="cyan")
    table.add_column("Value", justify="right", style="green")
    table.add_row("Lat", f"{lat:.5f}")
    table.add_row("Lon", f"{lon:.5f}")
    return Panel(table, title="GPS", border_style="green")

def make_imu_panel(acc, gyro, mag):
    table = Table(box=box.SIMPLE, expand=True)
    table.add_column("Axis", style="white")
    table.add_column("Acc(g)", justify="right", style="red")
    table.add_column("Gyr(d/s)", justify="right", style="blue")
    table.add_column("Mag(uT)", justify="right", style="magenta")
    
    axes = ["X", "Y", "Z"]
    for i in range(3):
        table.add_row(
            axes[i], 
            f"{acc[i]:.2f}", 
            f"{gyro[i]:.1f}", 
            f"{mag[i]:.1f}"
        )
    return Panel(table, title="IMU (9-Axis)", border_style="blue")

# --- メイン処理 ---
def main():
    console = Console()
    layout = Layout()

    # レイアウト定義
    layout.split(Layout(name="header", size=4), Layout(name="main", ratio=1))
    layout["main"].split_row(Layout(name="left"), Layout(name="right"))
    layout["left"].split_column(Layout(name="gps"), Layout(name="env"))
    layout["right"].split_column(Layout(name="imu"))

    # 1. シリアル接続
    try:
        if USE_DUMMY_DATA:
            ser = DummySerial()
            console.print("[yellow]Running in DUMMY mode[/yellow]")
        else:
            ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            console.print(f"[green]Connected to {SERIAL_PORT}[/green]")
            time.sleep(2) # Arduinoのリセット待ち
    except Exception as e:
        console.print(f"[bold red]Connection Error:[/bold red] {e}")
        return

    # 2. ロガー作成
    logger = TelemetryLogger()
    console.print(f"Logging to: {logger.filepath}")
    time.sleep(1)

    # 変数初期化
    base_pressure = None
    calibration_buffer = []
    is_calibrating = True
    
    # 初期画面描画 (青い枠線対策)
    layout["header"].update(make_header(True, logger.filepath, 0, -99))
    layout["gps"].update(make_gps_panel(0, 0))
    layout["env"].update(make_env_panel(0, 0, 0, 0))
    layout["imu"].update(make_imu_panel([0,0,0], [0,0,0], [0,0,0]))

    try:
        with Live(layout, refresh_per_second=10, screen=True) as live:
            while True:
                try:
                    # 1行読み込み
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if not line: continue

                    parts = line.split(',')
                    
                    # millis(1) + data(13) + rssi(1) = 15個
                    if len(parts) >= 15:
                        # データをパース
                        current_millis = int(parts[0])
                        # vals = [lat, lon, press, temp, ax...mz] (13個)
                        vals = [float(x) for x in parts[1:14]]
                        current_rssi = int(float(parts[14]))

                        # データを個別の変数へ
                        lat, lon = vals[0], vals[1]
                        press, temp = vals[2], vals[3]
                        acc, gyro, mag = vals[4:7], vals[7:10], vals[10:13]

                        # 高度計算 & キャリブレーション
                        if is_calibrating:
                            calibration_buffer.append(press)
                            if len(calibration_buffer) >= CALIBRATION_SAMPLES:
                                base_pressure = sum(calibration_buffer) / len(calibration_buffer)
                                is_calibrating = False
                            altitude = 0.0
                        else:
                            altitude = calculate_altitude(press, base_pressure)

                        # ログ保存
                        logger.log(current_millis, vals, current_rssi, altitude)

                        # UI更新
                        layout["header"].update(make_header(is_calibrating, logger.filepath, current_millis, current_rssi))
                        layout["gps"].update(make_gps_panel(lat, lon))
                        layout["env"].update(make_env_panel(press, temp, altitude, base_pressure))
                        layout["imu"].update(make_imu_panel(acc, gyro, mag))

                except ValueError:
                    continue
                except KeyboardInterrupt:
                    break
                    
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        console.print("\n[bold green]Session Finished.[/bold green]")
        console.print(f"Log saved: {logger.filepath}")

if __name__ == "__main__":
    main()