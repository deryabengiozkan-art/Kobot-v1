import _thread
import utime
from machine import Pin
from encoder_portable import Encoder
from twowheel import TwoWheel
from PID import PID
from commsbin import CommsBin
from battery import Battery

# --- Donanım Tanımları ---
robot = TwoWheel(motor1_pins=(6, 7), motor2_pins=(20, 19))
enc_left  = Encoder(Pin(2,  Pin.IN), Pin(3,  Pin.IN))
enc_right = Encoder(Pin(10, Pin.IN), Pin(11, Pin.IN))
battery = Battery(battery_pin=26)
comms = CommsBin(uart_id=1, tx_pin=4, rx_pin=5)

# PID Katsayıları (Pulse/s dünyasında sayılar büyük olduğu için Kp'yi düşük tut)
# 600 PPR için 1000-2000 pals/s normal hızlardır.
pid_left  = PID(kp=2.2, ki=1.4, kd=0.08, output_limits=(0, 65535))
pid_right = PID(kp=2.2, ki=1.4, kd=0.08, output_limits=(0, 65535))

# --- CORE 1: HAM PID KONTROLÜ ---
def control_loop():
    while True:
        # Pi 4'ten gelen ham Pulse/s hedefleri
        target_p_l = comms.target_v_l
        target_p_r = comms.target_v_r
        
        # Encoder'dan gelen ham Pulse/s geri beslemesi
        curr_p_l = enc_left.velocity()
        curr_p_r = enc_right.velocity()
        
        if abs(target_p_l) < 10 and abs(target_p_r) < 10:
            robot.motor1_write(0, True); robot.motor2_write(0, True)
            pid_left.reset(); pid_right.reset()
        else:
            pid_left.setpoint = abs(target_p_l)
            pid_right.setpoint = abs(target_p_r)
            
            pwm_l = pid_left.update(abs(curr_p_l))
            pwm_r = pid_right.update(abs(curr_p_r))
            
            # Motor 1 ters kablolama düzeltmesi (!dir)
            robot.motor1_write(int(pwm_l), not (target_p_l > 0)) 
            robot.motor2_write(int(pwm_r), target_p_r > 0)
            
        utime.sleep_ms(10) # 100Hz Kontrol Döngüsü

# --- CORE 0: HABERLEŞME ---
_thread.start_new_thread(control_loop, ())
last_tel = utime.ticks_ms()

while True:
    comms.receive_commands() # Binary paketleri çöz
    comms.check_failsafe(500) # 0.5s veri gelmezse dur
    
    if utime.ticks_diff(utime.ticks_ms(), last_tel) >= 50:
        # Telemetriyi ham pals/s olarak Pi 4'e yolla
        v_l_raw = enc_left.velocity()
        v_r_raw = enc_right.velocity()
        v_bat = battery.get_voltage()
        
        comms.send_telemetry(v_l_raw, v_r_raw, v_bat)
        last_telemetry = utime.ticks_ms()
    
    utime.sleep_ms(2)
