#!/bin/python3
# Xbox 360 Controller Support for Python
import pygame
from time import sleep

pygame.init()
pygame.joystick.init()

j = pygame.joystick.Joystick(0)
j.init()

while True:
    pygame.event.pump()
    
    for i in range(j.get_numbuttons()):
        b= j.get_button(i)
        print("Button", i, "State:", b)

    for i in range(j.get_numaxes()):
        a= j.get_axis(i)
        print("Axis", i, "Value:", a)

    r= j.get_button(5)  # Right Bumper
    if r:
        print("Right Bumper Pressed")
        j.rumble(0, 0.7, 500)  # Rumble for 500 ms
        
    print("-----")
    print("")
    sleep(0.2)
