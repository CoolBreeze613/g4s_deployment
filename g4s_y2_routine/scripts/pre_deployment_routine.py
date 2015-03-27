#!/usr/bin/env python

import rospy

from datetime import time, date
from dateutil.tz import tzlocal

from routine_behaviours.robot_routine import RobotRoutine
    

if __name__ == '__main__':
    rospy.init_node("g4s_routine")

    # start and end times -- all times should be in local timezone
    localtz = tzlocal()
    start = time(8,30, tzinfo=localtz)
    end = time(20,00, tzinfo=localtz)

    # how long to stand idle before doing something
    idle_duration=rospy.Duration(20)

    routine = RobotRoutine(daily_start=start, daily_end=end, 
        idle_duration=idle_duration)    
    
    # set tasks and start execution
    routine.create_routine()
    routine.start_routine()
    
    rospy.spin()
