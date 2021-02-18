#!/bin/bash

keyboard()
{
	rosrun teleop_twist_keyboard teleop_twist_keyboard.py
}

send_serial()
{
    rosrun send_serial send_serial.py
}
