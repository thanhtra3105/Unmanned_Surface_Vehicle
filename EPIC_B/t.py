import serial
import time

# Mở cổng serial (serial0 trên Raspberry Pi)
ser = serial.Serial("/dev/ttyAMA0", baudrate=57600, timeout=1)

print("✅ UART opened on /dev/serial0 at 57600 baud")

try:
    while True:
        # Gửi 1 chuỗi ra UART
        ser.write(b"Hello from Raspberry Pi!\n")
        print("📤 Sent: Hello from Raspberry Pi!")

        # Đọc dữ liệu (nếu có)
        data = ser.readline().decode(errors="ignore").strip()
        if data:
            print("📥 Received:", data)

        time.sleep(1)

except KeyboardInterrupt:
    print("⏹️ Stopped by user")
finally:
    ser.close()
    print("🔌 UART closed")
