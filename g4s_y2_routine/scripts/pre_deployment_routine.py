#!/usr/bin/env python

import rospy

from datetime import time, date, timedelta
from dateutil.tz import tzlocal

from routine_behaviours.robot_routine import RobotRoutine
from strands_executive_msgs.msg import Task    
from strands_executive_msgs import task_utils

def create_wait_task(waypoint_name, duration=rospy.Duration(30)):
    patrol_task = Task(action='wait_action', start_node_id=waypoint_name, end_node_id=waypoint_name, max_duration=duration)
    task_utils.add_time_argument(patrol_task, rospy.Time())
    task_utils.add_duration_argument(patrol_task, duration)       
    return patrol_task


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

    wait_wps = ['WayPoint3', 'WayPoint4', 'WayPoint5', 'WayPoint9', 'ChargingPoint']#, 'WayPoint13']#, 'WayPoint12']
    tasks = map(create_wait_task, wait_wps)
    
    # set tasks and start execution
    routine.create_task_routine(tasks, repeat_delta=timedelta(seconds=(15 * 60)))

    # routine.runner.add_day_off('Saturday')
    routine.runner.add_day_off('Sunday')

    routine.start_routine()
    
    rospy.spin()
