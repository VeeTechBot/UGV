import keyboard  # For keyboard events
import serial    # For serial communication
import time      # For delays

# --- Serial Communication Setup ---
try:      
    teenzycmd = serial.Serial('COM11', 57600, timeout=0.1)
    print("Serial port opened successfully.")
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
    print("Please ensure the Teensy is connected and the COM port is correct.")
    exit()

# --- Main Loop for Keyboard Input ---
while True:
    # ---- UGV Drive Control ----
    if keyboard.is_pressed('w'):
        teenzycmd.write(b'w\n')
        print('Forward')
    elif keyboard.is_pressed('s'):
        teenzycmd.write(b's\n')
        print('Backward')
    elif keyboard.is_pressed('a'):
        teenzycmd.write(b'a\n')
        print('Left Turn')
    elif keyboard.is_pressed('d'):
        teenzycmd.write(b'd\n')
        print('Right Turn')

    # ---- Auger Stepper Control ----
    elif keyboard.is_pressed('u') and keyboard.is_pressed('1'):
        teenzycmd.write(b'u1\n')  # Both steppers UP
        print('Auger UP (both steppers)')
        time.sleep(0.2)

    elif keyboard.is_pressed('u') and keyboard.is_pressed('2'):
        teenzycmd.write(b'u2\n')  # Both steppers DOWN
        print('Auger DOWN (both steppers)')
        time.sleep(0.2)

    # ---- Auger Relay Control ----
    elif keyboard.is_pressed('e') and keyboard.is_pressed('o'):
        teenzycmd.write(b'eo\n')  # Relay ON
        print('Auger Relay ON')
        time.sleep(0.2)

    elif keyboard.is_pressed('e') and keyboard.is_pressed('f'):
        teenzycmd.write(b'ef\n')  # Relay OFF
        print('Auger Relay OFF')
        time.sleep(0.2)

    # ---- Mechanism Motors (unchanged from your code) ----
    elif keyboard.is_pressed('1') and keyboard.is_pressed('f'):
        teenzycmd.write(b'1f\n')
        print('Mechanism 1 Forward (Timed)')
        time.sleep(0.1)
    elif keyboard.is_pressed('1') and keyboard.is_pressed('r'):
        teenzycmd.write(b'1r\n')
        print('Mechanism 1 Reverse (Timed)')
        time.sleep(0.1)

    elif keyboard.is_pressed('2') and keyboard.is_pressed('f'):
        teenzycmd.write(b'2f\n')
        print('Mechanism 2 Forward (Timed)')
        time.sleep(0.1)
    elif keyboard.is_pressed('2') and keyboard.is_pressed('r'):
        teenzycmd.write(b'2r\n')
        print('Mechanism 2 Reverse (Timed)')
        time.sleep(0.1)

    elif keyboard.is_pressed('3') and keyboard.is_pressed('f'):
        teenzycmd.write(b'3f\n')
        print('Mechanism 3 Forward (Timed)')
        time.sleep(0.1)
    elif keyboard.is_pressed('3') and keyboard.is_pressed('r'):
        teenzycmd.write(b'3r\n')
        print('Mechanism 3 Reverse (Timed)')
        time.sleep(0.1)

    elif keyboard.is_pressed('4') and keyboard.is_pressed('f'):
        teenzycmd.write(b'4f\n')
        print('Toggle Mechanism Motor 4')
        time.sleep(0.2)

    # ---- Camera Control ----
    elif keyboard.is_pressed('c') and keyboard.is_pressed('1'):
        teenzycmd.write(b'c1\n')
        print('Switching to Camera 1')
        time.sleep(0.2)
    elif keyboard.is_pressed('c') and keyboard.is_pressed('2'):
        teenzycmd.write(b'c2\n')
        print('Switching to Camera 2')
        time.sleep(0.2)

    # ---- Default: Stop UGV ----
    else:
        teenzycmd.write(b'x\n')

    time.sleep(0.05)
