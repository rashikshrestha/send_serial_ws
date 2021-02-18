#!/bin/bash

keyboard()
{
	rosrun teleop_twist_keyboard teleop_twist_keyboard.py
}

send_serial()
{
    rosrun send_serial send_serial.py
}

show_cmd_vel()
{
    rostopic echo /cmd_vel
}
