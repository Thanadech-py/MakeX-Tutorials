# Import necessary modules
import novapi
import time
import math
from mbuild.encoder_motor import encoder_motor_class
from mbuild import power_expand_board
from mbuild import gamepad
from mbuild import power_manage_module
import time

BL_POWER = 100 #Control the Brushless Motors Power

class dc_motor:
    # Default DC port
    dc_port = "DC1"
    # Default direction (not reversed)
    reverse = False
    
    # Initialize DC motor with a specific port
    def __init__(self, port: str) -> None:
        self.dc_port = port
        
    # Method to set the direction of the motor
    def set_reverse(self, rev: bool) -> None:
        self.reverse = rev
        
    # Method to turn on the DC motor
    def on(self, Power: int) -> None:
        power = -Power if self.reverse else Power
        power_expand_board.set_power(self.dc_port, power)
        
    # Method to turn off the DC motor
    def off(self) -> None:
        power_expand_board.stop(self.dc_port)
        
class brushless_motor:
    # Default brushless motor port
    bl_port = "BL1"
    
    # Initialize brushless motor with a specific port
    def __init__(self, port: str) -> None:
        self.bl_port = port
        
    # Method to turn on the brushless motor
    def on(self) -> None:
        power_expand_board.set_power(self.bl_port, BL_POWER)
        
    # Method to turn off the brushless motor
    def off(self) -> None:
        power_expand_board.stop(self.bl_port)   
        
# Instantiate DC motors
Feedder = dc_motor("DC1")

# Instantiate Blushless motors
bl_1 = brushless_motor("BL1")

while True:
    #The Way to control DC_Motor 
    Feedder.set_reverse(True) #Set te rotation of the motor
    Feedder.on(100)           #Turn On the Motor and control the power 
    
    bl_1.on()                 #Turn On the Blushless Motor
    
    
