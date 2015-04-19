#!/usr/bin/env python

import rospy
from routine_behaviours.patrol_routine import PatrolRoutine
from task_executor.task_routine import unix_time, delta_between, time_less_than, time_greater_than
from dateutil.tz import *
from datetime import *
import random

from strands_executive_msgs import task_utils
from strands_executive_msgs.msg import Task

def create_metric_map_task(waypoint_name, duration=rospy.Duration(120)):
    task = Task(action='ptu_pan_tilt_metric_map', max_duration=duration, start_node_id=waypoint_name)
    task_utils.add_int_argument(task, '-160')
    task_utils.add_int_argument(task, '20')
    task_utils.add_int_argument(task, '160')
    task_utils.add_int_argument(task, '-30')
    task_utils.add_int_argument(task, '30')
    task_utils.add_int_argument(task, '30')
    return task

def create_object_learn_task(waypoint_name, duration=rospy.Duration(240)):
    task = Task(action='OBJECT_LEARN_ACTION_SERVER', max_duration=duration, start_node_id=waypoint_name)
    return task

def create_object_search_task(waypoint_name, duration=rospy.Duration(480)):
    task = Task(action='OBJECT_SEARCH_ACTION_SERVER', max_duration=duration, start_node_id=waypoint_name)
    return task

def create_door_check_task(waypoint_name, duration=rospy.Duration(5)):
    task = Task(action='door_check', max_duration=duration, start_node_id=waypoint_name)
    return task

def create_wait_task(waypoint_name, duration=rospy.Duration(120)):
    task = Task(action='wait_action', start_node_id=waypoint_name, end_node_id=waypoint_name, max_duration=duration)
    task_utils.add_time_argument(task, rospy.Time())
    task_utils.add_duration_argument(task, duration)       
    return task

class G4SRoutine(PatrolRoutine):

    def __init__(self, daily_start, daily_end, tour_duration_estimate=None, idle_duration=rospy.Duration(5), charging_point = 'ChargingPoint'):
        super(G4SRoutine, self).__init__(daily_start=daily_start, daily_end=daily_end, tour_duration_estimate=tour_duration_estimate, idle_duration=idle_duration, charging_point=charging_point)        

    def set_random_task_time(self, task, start_date, start_time, window_size):
        if task.max_duration is None:
            raise Exception('Max duration is not set')
            
        # create a random window between start_time and start_time + window_size that's at least max_duration in size

        # subtract max duration from window size to give a range to pick from
        duration_secs = task.max_duration.secs
        window_secs = window_size.total_seconds()
        assert window_secs > duration_secs

        max_start = window_secs - duration_secs

        start = random.randrange(0, max_start)
        end = random.randrange(start, window_secs)

        # rospy.loginfo(start)
        # rospy.loginfo(end)

        release_date = datetime.combine(start_date, start_time) + timedelta(seconds=start)
        end_date = datetime.combine(start_date, start_time) + timedelta(seconds=end)
     
        task.start_after = rospy.Time(unix_time(release_date))
        task.end_before = rospy.Time(unix_time(end_date))


    def extra_tasks_for_today(self):
        """
        Return a list of extra tasks for the day ahead. Called every morning before the routine day starts.
        """
        localtz = tzlocal()
        datetime_today = datetime.fromtimestamp(rospy.get_rostime().to_sec(), tz=localtz)
        day_today = datetime_today.strftime("%A")
        date_today = datetime_today.date()
        rospy.loginfo('Looking for daily tasks for %s, %s' % (day_today, date_today))
       
        eight_forty_five= time(8,45, tzinfo=localtz)
        eleven_thirty= time(11,30, tzinfo=localtz)
        fourteen_thirty=time(14,30, tzinfo=localtz)
        seventeen_fifteen= time(17,15, tzinfo=localtz)
        
        metric_wps=['WayPoint13', 'WayPoint18', 'WayPoint9','WayPoint11','WayPoint5','WayPoint3']        
        object_learn_wps=['WayPoint13', 'WayPoint18', 'WayPoint9', 'WayPoint11']        
        object_search_wps=['WayPoint1', 'WayPoint2', 'WayPoint3']
        door_wps=['WayPoint7', 'WayPoint4']
        
        morning_start = eight_forty_five
        morning_duration = delta_between(eleven_thirty, morning_start)
        
        lunch_start = eleven_thirty
        lunch_duration = delta_between(fourteen_thirty, lunch_start)

        afternoon_start = fourteen_thirty
        afternoon_duration = delta_between(seventeen_fifteen, afternoon_start)

        tasks = []
        for i in range(4):
            #morning
            task=create_metric_map_task(random.choice(metric_wps))
            self.set_random_task_time(task, date_today, morning_start, morning_duration)
            tasks.append(task)
            
            task=create_door_check_task(random.choice(door_wps))
            self.set_random_task_time(task, date_today, morning_start, morning_duration)
            tasks.append(task)
            
            if i<3:
                task=create_object_learn_task(random.choice(object_learn_wps))
                self.set_random_task_time(task, date_today, morning_start, morning_duration)
                tasks.append(task)
                
                task=create_object_search_task(random.choice(object_search_wps))
                self.set_random_task_time(task, date_today, morning_start, morning_duration)
                tasks.append(task)
                
            #lunch (less tasks because we want the robot mostly learning people tracks)
            if i<1:
                task=create_metric_map_task(random.choice(metric_wps))
                self.set_random_task_time(task, date_today, lunch_start, lunch_duration)
                tasks.append(task)
            
                task=create_door_check_task(random.choice(door_wps))
                self.set_random_task_time(task, date_today,   lunch_start, lunch_duration)
                tasks.append(task)
            
                task=create_object_learn_task(random.choice(object_learn_wps))
                self.set_random_task_time(task, date_today,  lunch_start, lunch_duration)
                tasks.append(task)
                
                task=create_object_search_task(random.choice(object_search_wps))
                self.set_random_task_time(task, date_today,   lunch_start, lunch_duration)
                tasks.append(task)
            
                
            #afternoon
            task=create_metric_map_task(random.choice(metric_wps))
            self.set_random_task_time(task, date_today, afternoon_start, afternoon_duration)
            tasks.append(task)
            
            task=create_door_check_task(random.choice(door_wps))
            self.set_random_task_time(task, date_today, afternoon_start, afternoon_duration)
            tasks.append(task)
            
            if i<3:
                task=create_object_learn_task(random.choice(object_learn_wps))
                self.set_random_task_time(task, date_today, afternoon_start, afternoon_duration)
                tasks.append(task)
                
                task=create_object_search_task(random.choice(object_search_wps))
                self.set_random_task_time(task, date_today, afternoon_start, afternoon_duration)
                tasks.append(task)
        return tasks


    def on_idle(self):
        """
        Called when the robot is idle
        """
        rospy.loginfo('G4SRoutine.on_idle')
        # generate a random waypoint visit on idle
        people_track_wps=['WayPoint13', 'WayPoint18', 'WayPoint9','WayPoint11','WayPoint5','WayPoint3', 'WayPoint14', 'WayPoint10', 'WayPoint6']
        people_track_wps_lunch=['WayPoint14', 'WayPoint10', 'WayPoint6']
        
        localtz = tzlocal()
        current_time = datetime.fromtimestamp(rospy.get_rostime().to_sec(), tz=localtz).time()
        eleven_thirty = time(11,30, tzinfo=localtz)
        fourteen_thirty = time(14,30, tzinfo=localtz)
        
        if time_greater_than(current_time, eleven_thirty) and time_less_than(current_time, fourteen_thirty):
            task = create_wait_task(random.choice(people_track_wps_lunch))
        else:
            task = create_wait_task(random.choice(people_track_wps))
             
        self.add_tasks([task])
        
        
          

