import utime
import struct
import math
from machine import Pin, ADC, UART, PWM

# --- 1. BATTERY MODULE ---
class Battery:
    def __init__(self, battery_pin, R1=100.0, R2=47.0, ref_voltage=3.3):
        self.battery_adc = ADC(Pin(battery_pin))
        self.ratio = (R1 + R2) / R2
        self.ref_voltage = ref_voltage
        self.current_voltage = 0.0

    def get_voltage(self, samples=10):
        raw_sum = 0
        for _ in range(samples):
            raw_sum += self.battery_adc.read_u16()
        avg_raw = raw_sum / samples
        self.current_voltage = (avg_raw / 65535) * self.ref_voltage * self.ratio
        return self.current_voltage

# --- 2. COMMSBIN MODULE (Binary Protocol) ---
class CommsBin:
    def __init__(self, uart_id=1, tx_pin=4, rx_pin=5, baudrate=115200):
        self.uart = UART(uart_id, baudrate=baudrate, tx=Pin(tx_pin), rx=Pin(rx_pin))
        self.header = b'\xAA\x55'
        self.footer = b'\x0A'
        self.target_v_l = 0.0
        self.target_v_r = 0.0
        self.last_heartbeat = utime.ticks_ms()

    def _calculate_checksum(self, data):
        cs = 0
        for b in data: cs ^= b
        return cs

    def send_telemetry(self, v_l, v_r, battery):
        msg_id = 0x02
        payload = struct.pack('<fff', v_l, v_r, battery)
        cs = self._calculate_checksum(payload)
        packet = self.header + struct.pack('<B', msg_id) + payload + struct.pack('<B', cs) + self.footer
        self.uart.write(packet)

    def receive_commands(self):
        if self.uart.any() >= 13:
            raw = self.uart.read(13)
            if raw[:2] == self.header:
                msg_id = raw[2]
                if msg_id == 0x01:
                    payload = raw[3:11]
                    checksum = raw[11]
                    if self._calculate_checksum(payload) == checksum:
                        self.target_v_l, self.target_v_r = struct.unpack('<ff', payload)
                        self.last_heartbeat = utime.ticks_ms()
                        return True
        return False

    def check_failsafe(self, timeout_ms=500):
        if utime.ticks_diff(utime.ticks_ms(), self.last_heartbeat) > timeout_ms:
            self.target_v_l, 0.0; self.target_v_r = 0.0
            return True
        return False

# --- 3. ENCODER MODULE ---
class Encoder:
    def __init__(self, pin_x, pin_y, scale=1):
        self.scale = scale
        self.forward = True
        self.pin_x = pin_x
        self.pin_y = pin_y
        self._x = pin_x()
        self._y = pin_y()
        self._pos = 0
        self.last_pos = 0
        self.last_time = utime.ticks_ms()

        try:
            self.x_interrupt = pin_x.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self.x_callback, hard=True)
            self.y_interrupt = pin_y.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self.y_callback, hard=True)
        except TypeError:
            self.x_interrupt = pin_x.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self.x_callback)
            self.y_interrupt = pin_y.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self.y_callback)

    def x_callback(self, pin_x):
        if (x := pin_x()) != self._x:
            self._x = x
            self.forward = x ^ self.pin_y()
            self._pos += 1 if self.forward else -1

    def y_callback(self, pin_y):
        if (y := pin_y()) != self._y:
            self._y = y
            self.forward = y ^ self.pin_x() ^ 1
            self._pos += 1 if self.forward else -1

    def velocity(self):
        current_time = utime.ticks_ms()
        current_pos = self._pos
        dt = utime.ticks_diff(current_time, self.last_time) / 1000.0
        if dt > 0:
            vel = (current_pos - self.last_pos) / dt
            self.last_pos = current_pos
            self.last_time = current_time
            return vel * self.scale
        return 0.0

# --- 4. PID MODULE ---
class PID:
    def __init__(self, kp, ki, kd, setpoint=0.0, output_limits=(None, None)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self._min_out, self._max_out = output_limits
        self._last_error = 0
        self._integral = 0
        self._last_time = utime.ticks_ms()

    def update(self, measurement):
        now = utime.ticks_ms()
        dt = utime.ticks_diff(now, self._last_time) / 1000.0
        if dt <= 0: return 0
        error = self.setpoint - measurement
        p_term = self.kp * error
        self._integral += error * dt
        i_term = self.ki * self._integral
        d_term = self.kd * (error - self._last_error) / dt
        output = p_term + i_term + d_term
        if self._min_out is not None and self._max_out is not None:
            output = max(self._min_out, min(output, self._max_out))
        self._last_error = error
        self._last_time = now
        return output

    def reset(self):
        self._integral = 0; self._last_error = 0; self._last_time = utime.ticks_ms()

# --- 5. TWOWHEEL MODULE (Motor Control) ---
class TwoWheel:
    def __init__(self, motor1_pins=(6, 7), motor2_pins=(20, 19), freq=1000, scale=1.0):
        self.motor1_pin2 = PWM(Pin(motor1_pins[0], Pin.OUT))
        self.motor1_pin1 = PWM(Pin(motor1_pins[1], Pin.OUT))
        self.motor2_pin2 = PWM(Pin(motor2_pins[0], Pin.OUT))
        self.motor2_pin1 = PWM(Pin(motor2_pins[1], Pin.OUT))
        for p in [self.motor1_pin1, self.motor1_pin2, self.motor2_pin1, self.motor2_pin2]:
            p.freq(freq)

    def motor1_write(self, duty_cycle, direction):
        if direction:
            self.motor1_pin1.duty_u16(duty_cycle); self.motor1_pin2.duty_u16(0)
        else:
            self.motor1_pin1.duty_u16(0); self.motor1_pin2.duty_u16(duty_cycle)

    def motor2_write(self, duty_cycle, direction):
        if direction:
            self.motor2_pin1.duty_u16(duty_cycle); self.motor2_pin2.duty_u16(0)
        else:
            self.motor2_pin1.duty_u16(0); self.motor2_pin2.duty_u16(duty_cycle)
