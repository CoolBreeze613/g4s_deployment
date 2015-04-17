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

def create_metric_map_task(waypoint_name, duration=rospy.Duration(120)):
    task = Task(action='ptu_pan_tilt_metric_map', max_duration=duration, start_node_id=waypoint_name)
    task_utils.add_int_argument(task, '-160')
    task_utils.add_int_argument(task, '20')
    task_utils.add_int_argument(task, '160')
    task_utils.add_int_argument(task, '-30')
    task_utils.add_int_argument(task, '30')
    task_utils.add_int_argument(task, '30')
    return task


if __name__ == '__main__':
    rospy.init_node("g4s_routine")

    # start and end times -- all times should be in local timezone
    localtz = tzlocal()
    #the timezone changed to CET for soem reason
    start = time(7,30, tzinfo=localtz)
    end = time(21,00, tzinfo=localtz)

    # how long to stand idle before doing something
    idle_duration=rospy.Duration(20)

    routine = RobotRoutine(daily_start=start, daily_end=end, 
        idle_duration=idle_duration)    

    wait_wps = ['WayPoint3', 'WayPoint4', 'WayPoint5', 'WayPoint9']#, 'ChargingPoint']#, 'WayPoint13']#, 'WayPoint12']
    wait_tasks = map(create_wait_task, wait_wps)
    
    metric_wps=['WayPoint13', 'WayPoint18', 'WayPoint9']
    metric_tasks=map(create_wait_task, metric_wps)
    
    
    # set tasks and start execution
    routine.create_task_routine(wait_tasks, repeat_delta=timedelta(seconds=(15 * 60)))
    routine.create_task_routine(metric_tasks, repeat_delta=timedelta(seconds=(60 * 60)))
    

    routine.runner.add_day_off('Saturday')
    routine.runner.add_day_off('Sunday')

    routine.start_routine()
    
    rospy.spin()
