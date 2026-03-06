import struct
from machine import UART, Pin
import utime

class CommsBin:
    def __init__(self, uart_id=1, tx_pin=4, rx_pin=5, baudrate=115200):
        self.uart = UART(uart_id, baudrate=baudrate, tx=Pin(tx_pin), rx=Pin(rx_pin))
        self.header = b'\xAA\x55'
        self.footer = b'\x0A'
        
        # Paylaşılan Durumlar
        self.target_v_l = 0.0
        self.target_v_r = 0.0
        self.last_heartbeat = utime.ticks_ms()

    def _calculate_checksum(self, data):
        """Tüm byte'lara XOR uygular."""
        cs = 0
        for b in data:
            cs ^= b
        return cs

    def send_telemetry(self, v_l, v_r, battery):
        """
        Pico'dan Pi'ye veri yollar.
        Format: 3 tane float (f) = 12 byte payload
        """
        msg_id = 0x02
        # 'fff' -> 3 tane 32-bit float (Little-endian)
        payload = struct.pack('<fff', v_l, v_r, battery)
        cs = self._calculate_checksum(payload)
        
        packet = self.header + struct.pack('<B', msg_id) + payload + struct.pack('<B', cs) + self.footer
        self.uart.write(packet)

    def receive_commands(self):
        """
        Pi'den gelen komutları okur.
        Format: 2 tane float (f) = 8 byte payload
        """
        if self.uart.any() >= 13: # 2(H) + 1(ID) + 8(Data) + 1(CS) + 1(F) = 13 Byte
            if self.uart.read(2) == self.header:
                msg_id = struct.unpack('<B', self.uart.read(1))[0]
                
                if msg_id == 0x01: # Hareket Komutu
                    raw_payload = self.uart.read(8)
                    checksum = struct.unpack('<B', self.uart.read(1))[0]
                    footer = self.uart.read(1)
                    
                    if self._calculate_checksum(raw_payload) == checksum:
                        # Veriyi float'lara geri çevir
                        self.target_v_l, self.target_v_r = struct.unpack('<ff', raw_payload)
                        self.last_heartbeat = utime.ticks_ms()
                        return True
        return False

    def check_failsafe(self, timeout_ms=500):
        """Veri gelmezse hızı sıfırlar."""
        if utime.ticks_diff(utime.ticks_ms(), self.last_heartbeat) > timeout_ms:
            self.target_v_l = 0.0
            self.target_v_r = 0.0
            return True
        return False
