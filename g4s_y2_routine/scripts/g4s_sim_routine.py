#!/usr/bin/env python

import rospy

from datetime import time, date, timedelta
from dateutil.tz import tzlocal

from g4s_y2.routine import G4SRoutine
from strands_executive_msgs.msg import Task    
from strands_executive_msgs import task_utils

def create_ptu_callibrate_task():
    #TODO FILL THIS
    return None

def create_ptu_reset_task():
    #TODO FILL THIS
    return None
    
def create_novelty_learning_task():
    #TODO FILL THIS
    return None

def create_travel_times_learning_task():
    #TODO BRUNO WILL FILL THIS ONCE JAIME CREATES AN ACTION SERVER FOR IT
    return None

def create_datacentre_task(to_replicate, delete_after_move=True):
    task = Task()
    # no idea, let's say 2 hours for now -- this action server can't be preempted though, so this is cheating
    task.max_duration = rospy.Duration(60 * 60 * 2)
    task.action = 'move_mongodb_entries'

    # add arg for collectionst o replication
    collections = StringList(to_replicate)
    msg_store = MessageStoreProxy()
    object_id = msg_store.insert(collections)
    task_utils.add_object_id_argument(task, object_id, StringList)
    # move stuff over 24 hours old
    task_utils.add_duration_argument(task, rospy.Duration(60 * 60 *24))
    # and delete afterwards
    task_utils.add_bool_argument(task, delete_after_move)
    return task


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

    #routine.random_nodes = ['WayPoint7', 'WayPoint10', 'WayPoint11', 'WayPoint12', 'WayPoint13', 'WayPoint16', 'ChargingPoint']

    # routine.runner.add_day_off('Saturday')
    # routine.runner.add_day_off('Sunday')
    
    
    
    # NIGHT TASKS
    # this is how you add something for when the robot is charging, but these tasks aren't allowed a location   
    ptu_reset_task=create_ptu_reset_task()
    #routine.add_night_task(ptu_reset_task)
    
    ptu_callibrate_task=create_ptu_callibrate_task()
    #routine.add_night_task(ptu_callibrate_task)
    
    offline_learning_task=create_novelty_learning_task()
    #routine.add_night_task(offline_novelty_learning_task)
    
    travel_times_learning_task=create_travel_times_learning_task()
   
    
    #LAST YEAR: collections_to_replicate=['heads',
                              #'metric_map_data',
                              #'rosout_agg',
                              #'robot_pose',
                              #'task_events',
                              #'scheduling_problems',
                              #'ws_observations',
                              #'monitored_nav_events']
    
    #CURRENT MONGO COLLECTIONS, COMMENTS ONES THAT NEED TO STAY IN THE MAIN DB FOREVER
    collections_to_replicate=['door_checks',
                              'monitored_nav_events',
                              #'nav_stats', NEEDED FOR LEARNING OF TRAVEL TIMES, SHOULD STAY IN THE MAIN DB
                              'people_perception',
                              'people_trajectory',
                              'region_knowledge',
                              'relational_episodes',
                              'scheduling_problems',
                              'soma',
                              'soma_roi',
                              'task_events',
                              #'topological_maps' , NEEDED FOR TOP NAV 
                              ]

    clear_datacentre_task = create_datacentre_task(collections_to_replicate)
    #routine.add_night_task(clear_datacentre_task)



    routine.start_routine()
    
    rospy.spin()
