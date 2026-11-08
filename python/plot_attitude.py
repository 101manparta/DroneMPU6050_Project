"""
plot_attitude.py
----------------
Membaca file CSV hasil capture dan membuat grafik attitudo drone (roll, pitch, yaw)
secara real-time / offline menggunakan matplotlib.

Persyaratan:
    pip install matplotlib pandas
"""

import argparse
import sys
import pandas as pd
import matplotlib.pyplot as plt

def main():
    parser = argparse.ArgumentParser(description="Plot drone attitude from CSV")
    parser.add_argument("--input", default="drone_data.csv", help="File CSV input")
    args = parser.parse_args()

    try:
        df = pd.read_csv(args.input, header=None,
                         names=["timestamp_ms", "roll", "pitch", "yaw",
                                "ax", "ay", "az", "gx", "gy", "gz", "temp_c"])
    except Exception as e:
        print(f"Gagal membaca {args.input}: {e}")
        sys.exit(1)

    if df.empty:
        print("File CSV kosong.")
        sys.exit(1)

    # Konversi timestamp ke detik relatif
    t = (df["timestamp_ms"] - df["timestamp_ms"].iloc[0]) / 1000.0

    plt.figure(figsize=(12, 8))

    # Plot Roll, Pitch, Yaw
    plt.subplot(3, 1, 1)
    plt.plot(t, df["roll"], label="Roll", color="tab:red")
    plt.ylabel("Roll (deg)")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.title("Attitudo Drone dari MPU6050")

    plt.subplot(3, 1, 2)
    plt.plot(t, df["pitch"], label="Pitch", color="tab:green")
    plt.ylabel("Pitch (deg)")
    plt.grid(True, alpha=0.3)
    plt.legend()

    plt.subplot(3, 1, 3)
    plt.plot(t, df["yaw"], label="Yaw", color="tab:blue")
    plt.xlabel("Waktu (s)")
    plt.ylabel("Yaw (deg)")
    plt.grid(True, alpha=0.3)
    plt.legend()

    plt.tight_layout()
    plt.savefig("attitude_plot.png", dpi=150)
    print("Grafik disimpan ke attitude_plot.png")
    plt.show()

if __name__ == "__main__":
    main()