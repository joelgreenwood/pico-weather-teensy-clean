from machine import UART, Pin
import utime

uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))  # GP0=TX, GP1=RX

def send_temp_uart(temp_f):
    msg = "T={}\n".format(int(temp_f * 10))
    uart.write(msg)
    print("Sent:", msg.strip())

# Demo: send every 5s
while True:
    send_temp_uart(72.5)  # example value
    utime.sleep(5)
