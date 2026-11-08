"""
serial_to_csv.py
----------------
Membaca data serial dari Arduino MPU6050 dan menyimpannya ke file CSV.
Dapat digunakan sebagai bridge sebelum data diolah oleh program C++ host.

Persyaratan:
    pip install pyserial

Penggunaan (Windows):
    python serial_to_csv.py --port COM3 --baud 115200 --output drone_data.csv

Penggunaan (Linux):
    python serial_to_csv.py --port /dev/ttyACM0 --baud 115200 --output drone_data.csv
"""

import serial
import argparse
import sys
import time
import csv

def main():
    parser = argparse.ArgumentParser(description="Arduino MPU6050 Serial to CSV")
    parser.add_argument("--port", required=True, help="Port serial, contoh COM3 atau /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default 115200)")
    parser.add_argument("--output", default="drone_data.csv", help="Nama file CSV output")
    parser.add_argument("--duration", type=int, default=0, help="Durasi capture dalam detik (0 = tanpa batas, tekan Ctrl+C untuk berhenti)")
    args = parser.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except serial.SerialException as e:
        print(f"Gagal membuka port {args.port}: {e}")
        sys.exit(1)

    print(f"Membuka {args.port} pada {args.baud} baud...")
    print(f"Menyimpan ke {args.output}")
    print("Tekan Ctrl+C untuk berhenti.\n")

    # Tunggu Arduino reset & kirim header
    time.sleep(2)
    ser.reset_input_buffer()

    start_time = time.time()
    count = 0

    try:
        with open(args.output, "w", newline="") as f:
            writer = csv.writer(f)
            while True:
                # Cek durasi
                if args.duration > 0 and (time.time() - start_time) >= args.duration:
                    print(f"\nDurasi {args.duration} detik tercapai.")
                    break

                line = ser.readline().decode("utf-8", errors="replace").strip()
                if not line:
                    continue

                # Lewati baris teks non-CSV (print statement Arduino)
                if not line[0].isdigit() and line[0] != '-':
                    continue

                parts = line.split(",")
                if len(parts) != 11:
                    continue

                writer.writerow(parts)
                count += 1
                if count % 100 == 0:
                    print(f"\rSampel tersimpan: {count}", end="", flush=True)

    except KeyboardInterrupt:
        print("\n\nDihentikan oleh pengguna.")
    finally:
        ser.close()
        print(f"\nTotal {count} sampel disimpan ke {args.output}")

if __name__ == "__main__":
    main()