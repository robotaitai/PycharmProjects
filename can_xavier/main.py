# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.
import time

import can

def print_hi():
    # Use a breakpoint in the code line below to debug your script.
    motor_socket = can.interface.Bus(channel='can0', bustype='socketcan_native')
    data = b'\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFC'

    msg = can.Message(arbitration_id=0x08, data=data, is_extended_id=False)
    motor_socket.send(msg, timeout=0.001)

    time.sleep(0.0001)

    message = motor_socket.recv(timeout=0.5)
    print(message)
    time.sleep(10)
    print('socket is about to shut down')
    time.sleep(1)
    motor_socket.shutdown()

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    print_hi()

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
