#Import the main library
import novapi
import time
import math
from mbuild.encoder_motor import encoder_motor_class    #Import encoder motors class
from mbuild import power_expand_board                   #Import Power expansion Module class
from mbuild import gamepad                              #Import Bluetooth controller class
from mbuild import power_manage_module
import time

# Instantiate Each encoder_motor wheels
Front_left = encoder_motor_class("M1", "INDEX1")
Front_right = encoder_motor_class("M2", "INDEX1")
Back_left = encoder_motor_class("M3", "INDEX1")
Back_right = encoder_motor_class("M4", "INDEX1")

MAX_SPEED = 255
SPEED_MULTIPLIER = 2.1
PID_SPEED_MULTIPLIER = 0.6


class PID:
    def __init__(self, Kp,  Ki, Kd, setpoint=0):
        self.Kp = Kp                # Proportional gain
        self.Ki = Ki                # Integral gain
        self.Kd = Kd                # Derivative gain
        self.setpoint = setpoint    # Desired value (target)
        self.integral = 0           # Sum of errors over time
        self.previous_error = 0     # Previous error (used for derivative)

    def update(self, current_value):
        # Calculate the error (setpoint - current value)
        error = self.setpoint - current_value
        
        # Proportional term
        P = self.Kp * error
        
        # Integral term
        self.integral += error
        I = self.Ki * self.integral

        # Derivative term
        derivative = error - self.previous_error
        D = self.Kd * derivative

        # Calculate the output
        output = P + I + D

        # Save the current error for the next update
        self.previous_error = error

        return output

    def set_setpoint(self, setpoint):
        self.setpoint = setpoint
        self.integral = 0           # Reset the integral to avoid wind-up
        self.previous_error = 0     # Reset previous error to avoid a large derivative spike


class motors:
    
    def drive(lf: int, lb: int, rf: int, rb: int):
        Back_left.set_speed(lb)       # left back 
        Back_right.set_speed(-rb)     # RIGHT BACK  
        Front_right.set_speed(-(rf))  # RIGHT FORWARD
        Front_left.set_speed(lf)      # LEFT BACK
    
    def stop():
        motors.drive(0, 0, 0, 0)
        
class util:
    def restrict(val, minimum, maximum):
        return max(min(val, maximum), minimum)

class holonomic:    

    pids = {
        "lf": PID(Kp=1, Ki=0, Kd=0),
        "lb": PID(Kp=1, Ki=0, Kd=0),
        "rf": PID(Kp=0.9, Ki=0, Kd=0),
        "rb": PID(Kp=0.9, Ki=0, Kd=0),
    }

    # motor tune
    tune = {
        "fl": 1,
        "fr": 1,
        "bl": 1,
        "br": 1,
    }

    def drive(vx, vy, wL, deadzone=25, pid=False):
        global SPEED_MULTIPLIER, PID_SPEED_MULTIPLIER
        if math.fabs(vx) < math.fabs(deadzone):
            vx = 0
        if math.fabs(vy) < math.fabs(deadzone):
            vy = 0
        if math.fabs(wL) < math.fabs(deadzone):
            wL = 0

        # Ensure the correct speed multiplier
        multiplier = PID_SPEED_MULTIPLIER if pid else SPEED_MULTIPLIER
            
        # Calculation for the wheel speed
        vFL = (vx + (vy * 1.2) + wL) * multiplier
        vFR = (-(vx) + (vy * 1.2) - wL) * multiplier
        vBL = (-(vx) + (vy * 1.2) + wL) * multiplier
        vBR = (vx + (vy * 1.2) - wL) * multiplier
        
        # Sliding check to not interfere with the normal movement, incase of tuning specific power
        if math.fabs(vx) > math.fabs(vy) and vx > 0:
            vFL *= holonomic.tune["fl"] # หน้าซ้าย
            vFL *= holonomic.tune["fr"] # หน้าขวา
            vBL *= holonomic.tune["bl"] # หลังซ้าย
            vBR *= holonomic.tune["br"] # หลังขวา
        if pid:            
            # Left Forward
            holonomic.pids["lf"].set_setpoint(vFL)
            vFL = holonomic.pids["lf"].update(-left_forward_wheel.get_value("speed"))
            # Left Back
            holonomic.pids["lb"].set_setpoint(vBL)
            vBL = holonomic.pids["lb"].update(-left_back_wheel.get_value("speed"))
            # Right Forward
            holonomic.pids["rf"].set_setpoint(vFR)
            vFR = holonomic.pids["rf"].update(right_forward_wheel.get_value("speed"))
            # Right Back
            holonomic.pids["rb"].set_setpoint(vBR)
            vBR = holonomic.pids["rb"].update(right_back_wheel.get_value("speed"))

        # Velocity
        vFL = util.restrict(vFL, -MAX_SPEED, MAX_SPEED)
        vFR = util.restrict(vFR, -MAX_SPEED, MAX_SPEED)
        vBL = util.restrict(vBL, -MAX_SPEED, MAX_SPEED)
        vBR = util.restrict(vBR, -MAX_SPEED, MAX_SPEED)
        # Drive motor
        motors.drive(vFL, vBL, vFR, vBR)
        
    def move_forward(power):
        holonomic.drive(0, power, 0)
        
    def move_backward(power):
        holonomic.drive(0, -power, 0)
        
    def slide_right(power):
        holonomic.drive(power, 0, 0)
        
    def slide_left(power):
        holonomic.drive(-power, 0, 0)
        
    def turn_right(power):
        holonomic.drive(0, 0, power)
        
    def turn_left(power):
        holonomic.drive(0, 0, -power)
        
class runtime:
    # Define control mode
    CTRL_MODE = 0
    
    # Robot state
    ENABLED = True
    def move_normal():
        if math.fabs(gamepad.get_joystick("Lx")) > 20 or math.fabs(gamepad.get_joystick("Ly")) > 20 or math.fabs(gamepad.get_joystick("Rx")) > 20:
            holonomic.drive(-gamepad.get_joystick("Lx"), gamepad.get_joystick("Ly"), -gamepad.get_joystick("Rx"), pid=True)
        else:
            motors.drive(0,0,0,0)

    def move_reversed():
        if math.fabs(gamepad.get_joystick("Lx")) > 20 or math.fabs(gamepad.get_joystick("Ly")) > 20 or math.fabs(gamepad.get_joystick("Rx")) > 20:
            holonomic.drive(gamepad.get_joystick("Lx"), -gamepad.get_joystick("Ly"), -gamepad.get_joystick("Rx"), pid=True)
        else:
            motors.drive(0,0,0,0)
    
    def change_mode():
        if novapi.timer() > 0.9:
            entrance_feed.off()
            feeder.off()
            conveyer.off()
            front_input.off()
            if runtime.CTRL_MODE == 0:
                runtime.CTRL_MODE = 1
            else:
                runtime.CTRL_MODE = 0
            novapi.reset_timer()
            
while True:
    if gamepad.is_key_pressed("L2") and gamepad.is_key_pressed("R2"):
            runtime.change_mode()
    else:
        if runtime.CTRL_MODE == 0:
            runtime.move_normal()
        else:
            runtime.move_reversed()