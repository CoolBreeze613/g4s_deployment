#!/usr/bin/env python

import rospy

from datetime import time, date, timedelta
from dateutil.tz import tzlocal

from g4s_y2.routine import G4SRoutine
from strands_executive_msgs.msg import Task    
from strands_executive_msgs import task_utils



if __name__ == '__main__':
    rospy.init_node("g4s_routine")

    # start and end times -- all times should be in local timezone
    localtz = tzlocal()
    start = time(8,30, tzinfo=localtz)
    end = time(00,00, tzinfo=localtz)

    # how long to stand idle before doing something
    idle_duration=rospy.Duration(10)

    # create the overall routine
    routine = G4SRoutine(daily_start=start, daily_end=end, idle_duration=idle_duration)

    routine.random_nodes = ['WayPoint7', 'WayPoint10', 'WayPoint11', 'WayPoint12', 'WayPoint13', 'WayPoint16', 'ChargingPoint']

    # routine.runner.add_day_off('Saturday')
    # routine.runner.add_day_off('Sunday')

    routine.start_routine()
    
    rospy.spin()
