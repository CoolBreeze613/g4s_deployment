#!/usr/bin/env python

import rospy
from routine_behaviours.patrol_routine import PatrolRoutine
from task_executor.task_routine import unix_time
from dateutil.tz import *
from datetime import *
import random

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

        tasks = []

        if day_today == 'Friday':
            # create a task which must have max_duration set 
            task = self.create_patrol_task('WayPoint10', rospy.Duration(30))

            # give it a random window based on the provided information
            self.set_random_task_time(task, date_today, time(22,30, tzinfo=localtz), timedelta(hours=1))

            tasks.append(task)

        return tasks


    def on_idle(self):
        """
        Called when the robot is idle
        """
        # generate a random waypoint visit on idle
        PatrolRoutine.on_idle(self)    

