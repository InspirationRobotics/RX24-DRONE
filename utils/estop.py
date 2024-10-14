import serial
import time
import keyboard

arduino = serial.Serial(port='COM3',baudrate=9600,timeout=1)

def send_stop_signal():
    arduino.write(b'STOP\n')
    print("EMERGENCY STOP sent")

def main():
    print("Press 's' for e-stop")

    while True:

        if keyboard.is_pressed('z'):
            send_stop_signal()
            time.sleep(1)
        time.sleep(0.1)

if __name__ == "main":
    main()