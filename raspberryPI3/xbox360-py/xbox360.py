#!/bin/python3

# Xbox 360 Controller Support for Python

from inputs import get_gamepad

while True:
    events = get_gamepad()
    for event in events:
        print(event.ev_type, event.code, event.state)