import struct
import csv
import os # ファイル操作のために追加
from datetime import datetime # 日付操作のために追加

# ESP32の「struct LogEntry」と全く同じ構造
# 修正版: クォータニオン + オイラー角 + サーボ角度 + 誤差Q値 + accelTrust + 制御フラグ
#        + 積分高度 + 積分速度 + Z軸自由落下加速度 + 発射フラグ
LOG_FORMAT = '<I ffff fff fff hhh iifB HBB BBBB B ffff fff ff fff f B ff f B'
LOG_SIZE = struct.calcsize(LOG_FORMAT)

# CSVのヘッダー (修正版)
CSV_HEADER = [
    'timestamp_ms',
    'temp', 'pres', 'alt', 'hum',
    'ax', 'ay', 'az',
    'gx', 'gy', 'gz',
    'cx', 'cy', 'cz',
    'lat_deg', 'lng_deg', 'gps_alt_m', 'sats',
    'year', 'month', 'day',
    'hour', 'min', 'sec', 'cs',
    'gps_updated',
    'q0', 'q1', 'q2', 'q3',  # クォータニオン
    'roll', 'pitch', 'yaw',   # オイラー角 [rad]
    'servo1_angle', 'servo2_angle',  # サーボ角度 [deg]
    'q_err_x', 'q_err_y', 'q_err_z',  # 誤差クォータニオン
    'accelTrust_value',  # 加速度補正信頼度 [0.0-1.0]
    'control_enabled',  # 制御有効フラグ
    'integrated_altitude',  # 積分高度 [m]
    'integrated_vz',  # 積分速度 [m/s]
    'az_freefall',  # Z軸自由落下加速度 [m/s²]
    'launch_detected'  # 発射検出フラグ
]

IN_FILE = 'fulldata.bin'
# 出力ファイル名は連番にするので、ベース名だけ定義
OUT_FILE_BASE = 'testflight-3'

# ★★★ 追加: ファイル分割設定 ★★★
SPLIT_INTERVAL_MS = 5 * 60 * 1000 # 5分 (ミリ秒)
# ★★★★★★★★★★★★★★★★★


def convert_and_split_csv(bin_file, out_base):
    print(f"Struct size: {LOG_SIZE} bytes")
    print(f"Opening binary file: {bin_file}")
    print(f"Splitting CSV every {SPLIT_INTERVAL_MS} ms")

    # ★★★ 追加: 現在の日付を取得してファイル名に含める ★★★
    current_date_str = datetime.now().strftime('%Y%m%d')
    out_base_with_date = f"{current_date_str}_{out_base}"
    print(f"Output file base name will be: {out_base_with_date}")
    # ★★★★★★★★★★★★★★★★★

    try:
        with open(bin_file, 'rb') as f_in:

            # --- 変数を初期化 ---
            f_out = None       # 現在の出力ファイルオブジェクト
            writer = None      # 現在のCSVライター
            file_index = 0     # ファイル番号 (001から始める)
            start_timestamp = 0 # 現在のファイルの最初のタイムスタンプ
            first_entry = True # 最初のデータかどうか

            count_total = 0
            count_current_file = 0

            while True:
                chunk = f_in.read(LOG_SIZE)
                if not chunk or len(chunk) < LOG_SIZE:
                    break # ファイルの終わり

                try:
                    data = struct.unpack(LOG_FORMAT, chunk)
                except struct.error as e:
                    print(f"Unpack error at entry {count_total}: {e}. File might be corrupt.")
                    break

                current_timestamp = data[0]

                # --- 新しいファイルを開く必要があるかチェック ---
                # 1. 最初のデータの場合
                # 2. または、現在のタイムスタンプが分割時間を超えた場合
                if first_entry or (current_timestamp >= start_timestamp + SPLIT_INTERVAL_MS):

                    # 既にファイルが開いていれば閉じる
                    if f_out:
                        f_out.close()
                        print(f"  -> File {file_index:03d} closed ({count_current_file} entries).")

                    # 新しいファイルを開く
                    file_index += 1
                    # ★★★ 修正: 日付付きのベース名を使用 ★★★
                    out_file_name = f"{out_base_with_date}_{file_index:03d}.csv" # 例: 20251028_converted_log_001.csv
                    print(f"Opening new output file: {out_file_name}")
                    f_out = open(out_file_name, 'w', newline='')
                    writer = csv.writer(f_out)
                    writer.writerow(CSV_HEADER) # ヘッダーを書き込み

                    # 最初のタイムスタンプを更新
                    start_timestamp = current_timestamp
                    first_entry = False
                    count_current_file = 0


                # --- データを処理して書き込む ---
                # GPSの緯度経度を float に戻す
                csv_row = list(data)
                csv_row[13] = data[13] / 1e6 # lat (インデックス13)
                csv_row[14] = data[14] / 1e6 # lng (インデックス14)

                writer.writerow(csv_row)
                count_total += 1
                count_current_file += 1

            # --- 最後のファイルを閉じる ---
            if f_out:
                f_out.close()
                print(f"  -> File {file_index:03d} closed ({count_current_file} entries).")

            print(f"\nSuccessfully converted {count_total} log entries into {file_index} file(s).")

    except FileNotFoundError:
        print(f"Error: Input file '{bin_file}' not found.")
    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    convert_and_split_csv(IN_FILE, OUT_FILE_BASE)
