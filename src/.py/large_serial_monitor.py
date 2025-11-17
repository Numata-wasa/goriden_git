import tkinter as tk
# scrolledtextは不要になったので削除
import serial
import threading
import time

# --- 設定項目 (ここを環境に合わせて変更してください) ---
SERIAL_PORT = "COM3"  # ArduinoやESP32が接続されているCOMポート
BAUD_RATE = 115200    # ボーレート
FONT_FAMILY = "Yu Gothic UI" # お好きなフォント名 (メイリオ, MSゴシックなど)
FONT_SIZE = 48        # ★★★ ここの数字を大きくします ★★★
# -----------------------------------------------------

# シリアルポートの接続
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"{SERIAL_PORT}に接続しました。")
except serial.SerialException as e:
    print(f"エラー: {e}")
    print(f"{SERIAL_PORT}に接続できませんでした。ポート名を確認してください。")
    exit()

# シリアルポートから継続的に読み込むための関数 (別スレッドで実行)
def read_serial_port():
    while not stop_thread:
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    # メインスレッドでGUIを安全に更新
                    root.after(0, update_text_widget, line)
            else:
                time.sleep(0.01)
        except Exception as e:
            print(f"読み取りエラー: {e}")
            break
    ser.close()
    print("シリアルポートを閉じました。")

# ★変更: テキストウィジェットを更新する関数
def update_text_widget(line):
    # StringVarの値を更新するだけで、Labelの表示が自動的に変わる
    latest_line_var.set(line)

# ウィンドウが閉じられたときの処理
def on_closing():
    global stop_thread
    stop_thread = True
    root.destroy()

# --- GUIのセットアップ ---
root = tk.Tk()
root.title(f"最新1行シリアルモニタ ({SERIAL_PORT})")
root.geometry("800x200") # 1行なので縦は小さめに

# ★フォントの設定
custom_font = (FONT_FAMILY, FONT_SIZE, "bold") # 少し太字にしても見やすいです

# ★変更: 1行表示用のStringVarを作成
# この変数の値を変えると、関連付けられたLabelの表示も変わる
latest_line_var = tk.StringVar()
latest_line_var.set("--- 待機中 ---") # 初期テキスト

# ★変更: ScrolledTextの代わりにLabelを使う
# textvariableに先ほどのStringVarを関連付ける
display_label = tk.Label(
    root, 
    textvariable=latest_line_var, 
    font=custom_font,
    anchor="w",     # 文字列を左寄せ (west)
    justify=tk.LEFT # 複数行になった場合（基本ないが）の左寄せ
)
display_label.pack(padx=20, pady=20, fill=tk.BOTH, expand=True)

# スレッド停止用のフラグ
stop_thread = False

# シリアル読み取りスレッドを開始
thread = threading.Thread(target=read_serial_port, daemon=True)
thread.start()

# ウィンドウを閉じるイベントに関数を紐付け
root.protocol("WM_DELETE_WINDOW", on_closing)

# GUIのメインループを開始
root.mainloop()