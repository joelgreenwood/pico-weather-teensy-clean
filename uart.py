# uart_tx.py (Pico)
from machine import UART, Pin
import utime

# Pico UART0: TX=GP0, RX=GP1 (change if you prefer another UART/pins)
uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))

def send_temp_uart(temp_f):
    # Send tenths of a degree as an integer in a simple line protocol: T=599\n
    msg = "T={}\n".format(int(temp_f * 10))
    uart.write(msg)

# Demo: send every 5s
def demo():
    temp_f = 59.9
    while True:
        send_temp_uart(temp_f)
        print("UART sent:", temp_f)
        utime.sleep(5)

# demo()
