import serial
import threading
import time

ser = serial.Serial('/dev/ttyACM0', 9600)

def send_data(command):
    i = 0
    while i < 3:
        try:
            ser.write(command.encode()) 
            print("Command sent to Arduino:", command)
            i += 1
        except Exception as e:
            print(f"Error sending data to Arduino: {e}")

def receive_data_from_arduino():
    while True:
        while ser.inWaiting() == 0:
            pass
        try:
            data = ser.readline().decode().strip()
            print("Command received from Arduino:", data)
        except Exception as e:
            print(f"Error receiving data from Arduino: {e}")

def main():
    send_thread = threading.Thread(target=send_data, args=("Hey Arduino!!",))
    receive_thread = threading.Thread(target=receive_data_from_arduino)

    send_thread.start()
    receive_thread.start()

    send_thread.join()
    receive_thread.join()

if __name__ == "__main__":
    main()
