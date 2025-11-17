

import serial
import serial.tools.list_ports
import argparse
import time
import pyfiglet  # テキストをAAに変換するライブラリ
from rich.live import Live  # 画面をリアルタイムに更新
from rich.panel import Panel  # 枠線
from rich.text import Text  # テキスト（中央揃えなどに使用）

def parse_args():
    """引数を解析する"""
    parser = argparse.ArgumentParser(description="シリアルで受信した数値を大きく表示します。")
    parser.add_argument('-p', '--port', help='シリアルポートのパス (例: /dev/ttyUSB0 や COM3)')
    parser.add_argument('-b', '--baud', help='ボーレート', type=int, default=115200)
    return parser.parse_args()

def open_serial(port: str = None, baud: int = 115200) -> serial.Serial:
    """
    シリアルポートを開く（元のコードを参考に簡略化）
    ポート指定がない場合は自動検出を試みる
    """
    if port:
        try:
            ser = serial.Serial(port, baudrate=baud, timeout=1.0)
            if not ser.isOpen():
                raise ConnectionError(f'Failed to open serial port: {port}')
            print(f"Connected to {port} at {baud} baud.")
            return ser
        except serial.SerialException as e:
            print(f"Error opening port {port}: {e}")
            raise e
    else:
        ports = serial.tools.list_ports.comports(include_links=False)
        if not ports:
            raise FileNotFoundError('No serial port detected. Specify a port or file name manually.')
        
        # 利用可能なポートをすべて試す
        for p in reversed(ports): # 逆順（新しいものが先に来ることが多い）から試す
            try:
                ser = serial.Serial(p.device, baudrate=baud, timeout=1.0)
                if ser.isOpen():
                    print(f"Auto-detected and connected to {p.device} at {baud} baud.")
                    return ser
            except serial.SerialException:
                print(f"Could not connect to {p.device}...")
                continue
        
        # すべてダメだった場合
        raise ConnectionError('Failed to auto-detect and open any serial port.')

def main():
    args = parse_args()
    
    try:
        ser = open_serial(args.port, args.baud)
    except Exception as e:
        print(f"Error: {e}")
        input("error")
        return

    # Figlet（AA）の準備
    # 'larry3d' や 'standard', 'doom' など色々なフォントがあります
    fig = pyfiglet.Figlet(font='doom', width=400)
    
    last_display_text = "Waiting..."
    
    # rich.Live を使ってTUIを開始
    try:
        with Live(refresh_per_second=10, screen=True, transient=True) as live:
            while True:
                if not ser.isOpen():
                    last_display_text = "Disconnected"
                    panel = Panel(Text(fig.renderText(last_display_text), justify="center"),
                                  title="Serial Monitor", border_style="bold red")
                    live.update(panel)
                    break
                
                try:
                    # 1行読み取る (改行コードまで)
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    
                    # 何かデータがあれば更新
                    if line:
                        last_display_text = line
                    
                    # 常に最新のテキストでパネルを再描画
                    # last_display_text が空でも、最後に受信した値で表示し続ける
                    panel = Panel(
                        Text(fig.renderText(last_display_text), justify="center"),
                        title=f"Serial Monitor [{ser.name}]",
                        border_style="bold green"
                    )
                    live.update(panel)
                    
                except serial.SerialException:
                    last_display_text = "Error"
                    panel = Panel(Text(fig.renderText(last_display_text), justify="center"),
                                  title="Serial Monitor", border_style="bold red")
                    live.update(panel)
                    break
                    
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        if ser and ser.isOpen():
            ser.close()
            print(f"Serial port {ser.name} closed.")

if __name__ == "__main__":
    main()