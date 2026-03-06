import machine
import utime
import struct
from commsbin import CommsBin

# Onboard LED to visualize data reception
led = machine.Pin("LED", machine.Pin.OUT)

# Initialize Binary Comms (using UART 1, TX=Pin 4, RX=Pin 5)
comms = CommsBin(uart_id=1, tx_pin=4, rx_pin=5, baudrate=115200)

print("Pico UART Test Node is running...")

while True:
    # 1. Check for incoming Command Packet (ID 0x01)
    # Expected payload: 2 floats (8 bytes)
    if comms.receive_commands():
        led.toggle()
        
        # Get received values
        val_l = comms.target_v_l
        val_r = comms.target_v_r
        
        # 2. Process data (Multiply by 2 as a test)
        response_l = val_l * 2.0
        response_r = val_r * 2.0
        fake_battery = 12.6
        
        # 3. Send Telemetry Packet back to Pi 4 (ID 0x02)
        # Payload: 3 floats (12 bytes)
        comms.send_telemetry(response_l, response_r, fake_battery)
        
        print(f"Received: {val_l}, {val_r} | Sent: {response_l}, {response_r}")

    utime.sleep_ms(10)
