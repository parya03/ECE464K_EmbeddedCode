import time
import threading
import serial
import robot_pb2 # generated from handdatainates.proto

# ---------------- Configuration ----------------
SERIAL_PORT = "/dev/tty.usbmodem101"  # Change as needed (e.g. COM3 on Windows)
BAUD_RATE = 115200
SEND_INTERVAL = 0.08  # 20 ms

# ---------------- Serial Setup ----------------
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)

# ---------------- Serial Reader Thread ----------------
def serial_reader():
    """Continuously read and print incoming serial data."""
    while True:
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            print(data.decode(errors='ignore'), end='')

reader_thread = threading.Thread(target=serial_reader, daemon=True)
reader_thread.start()

# ---------------- Main Loop ----------------
def send_handdatainates_loop():
    handdata = robot_pb2.HandData() 

    x, y, z = 0.0, 10.0, 10.0
    while True:
        # Update coordinates here however you like:
        x += 1
        y += 0
        z += 0

        if(x >= 35):
            x = -35

        # Populate and serialize protobuf message
        handdata.x = x
        handdata.y = y
        handdata.z = z
        handdata.timestamp = time.time()
        handdata.openness = 0
        handdata.pitch = 0
        encoded = handdata.SerializeToString()

        # Optional: prefix with length for easier parsing on receiver
        length_prefix = len(encoded).to_bytes(2, byteorder='big')
        ser.write(encoded)

        time.sleep(SEND_INTERVAL)

# ---------------- Start Sending ----------------
if __name__ == "__main__":
    try:
        send_handdatainates_loop()
    except KeyboardInterrupt:
        print("\nExiting.")
        ser.close()

