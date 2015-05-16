#!/usr/bin/env python

""" home_status.py - Version 0.1 2015-02-28

    Home automation

    Created for the N.E.S.T.O.R. project
    Copyright (c) 2015 Joffrey Kriegel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.htmlPoint
      
"""

import rospy
import actionlib
from smach import State, StateMachine, Concurrence, Container, UserData
from smach_ros import MonitorState, ServiceState, SimpleActionState, IntrospectionServer
from actionlib import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PoseStamped
from std_msgs.msg import Int32, Empty, String
from random import randrange
from tf.transformations import quaternion_from_euler
from math import  pi
from collections import OrderedDict
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
#from common_smart_ai.srv import GetObjective, UpdatePriority




class PreparingShower(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.heat_pub = rospy.Publisher('/HOME/showerHeatON', Empty)
        self.french_pub = rospy.Publisher('/NESTOR/french_voice', String)
	pass

    def execute(self, userdata):
        rospy.loginfo("Heating...")
	my_string = String()
	my_string.data = "La salle de bain est en train de chauffer. Vous pourrez l'utiliser dans cinq minutes."
        self.french_pub.publish(my_string)
	self.heat_pub.publish(Empty())
        rospy.sleep(300)
        return 'succeeded'

class GoShower(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.french_pub = rospy.Publisher('/NESTOR/french_voice', String)
        pass

    def execute(self, userdata):
        rospy.loginfo("Going into shower...")
	my_string = String()
	my_string.data = "La salle de bain est chaude. Vous pouvez aller vous doucher."
        self.french_pub.publish(my_string)
        rospy.sleep(600)
        return 'succeeded'

class StopShower(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.heat_pub = rospy.Publisher('/HOME/showerHeatOFF', Empty)
        self.french_pub = rospy.Publisher('/NESTOR/french_voice', String)
        pass

    def execute(self, userdata):
        rospy.loginfo("Stop shower...")
        my_string = String()
        my_string.data = "Il est temps de sortir de la douche."
        self.french_pub.publish(my_string)
	self.heat_pub.publish(Empty())
        rospy.sleep(1)
        return 'succeeded'

class LightEntry(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded'], input_keys=['day_mode'], output_keys=['day_mode'])
        self.ON_pub = rospy.Publisher('/MILIGHT/light3ON', Empty)
        self.white_pub = rospy.Publisher('/MILIGHT/light3White', Empty)
        self.color_pub = rospy.Publisher('/MILIGHT/light3Color', Int32)
        self.brightness_pub = rospy.Publisher('/MILIGHT/light3Brightness', Int32)

    def execute(self, userdata):
        is_day = userdata.day_mode
	out_int = Int32()

        if(is_day == 1): # Day = full white power
                self.ON_pub.publish(Empty())
		rospy.sleep(0.01)
                self.white_pub.publish(Empty())
		out_int.data = 19
		rospy.sleep(0.01)
                self.brightness_pub.publish(out_int)
		rospy.sleep(0.01)
        else: # Night = low red light
                self.ON_pub.publish(Empty())
                out_int.data = 170  
                rospy.sleep(0.01)
                self.color_pub.publish(out_int)
                out_int.data = 6  
                rospy.sleep(0.01)
                self.brightness_pub.publish(out_int)
                rospy.sleep(0.01)

        return 'succeeded'

class DarkEntry(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded'], input_keys=['day_mode'], output_keys=['day_mode'])
        self.OFF_pub = rospy.Publisher('/MILIGHT/light3OFF', Empty)

    def execute(self, userdata):
        is_day = userdata.day_mode

        self.OFF_pub.publish(Empty())
        rospy.sleep(0.01)

        return 'succeeded'

class WakingUp(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded'])
        self.ON_pub = rospy.Publisher('/MILIGHT/light3ON', Empty)
        self.ON2_pub = rospy.Publisher('/MILIGHT/light2ON', Empty)
	self.heatON_pub = rospy.Publisher('/HOME/showerHeatON', Empty)
        self.heatOFF_pub = rospy.Publisher('/HOME/showerHeatOFF', Empty)
        self.weather_pub = rospy.Publisher('/NESTOR/weather', Empty)
        self.french_pub = rospy.Publisher('/NESTOR/french_voice', String)
	self.music_pub = rospy.Publisher('/NESTOR/radio_jap', Empty)

    def execute(self, userdata):

        #self.ON_pub.publish(Empty())
        rospy.sleep(120)

	my_string=String()
	my_string.data="Il est l'heure de se lever. Debout les feignants."
	self.french_pub.publish(my_string)

        rospy.sleep(480)
	self.ON2_pub.publish(Empty())

        rospy.sleep(120)
	my_string.data="Chauffage de la salle de bain en cours."
	self.french_pub.publish(my_string)
        rospy.sleep(0.01)
	self.heatON_pub.publish(Empty())
        rospy.sleep(0.01)
	self.music_pub.publish(Empty())

        rospy.sleep(60)
	self.weather_pub.publish(Empty())

        rospy.sleep(240)
	self.heatOFF_pub.publish(Empty())

        return 'succeeded'


class GoingSleep(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded'])
        self.ON1_pub = rospy.Publisher('/MILIGHT/light1ON', Empty)
	self.color1_pub = rospy.Publisher('/MILIGHT/light1Color', Int32)
        self.brightness1_pub = rospy.Publisher('/MILIGHT/light1Brightness', Int32)
        self.ON2_pub = rospy.Publisher('/MILIGHT/light2ON', Empty)
	self.color2_pub = rospy.Publisher('/MILIGHT/light2Color', Int32)
        self.brightness2_pub = rospy.Publisher('/MILIGHT/light2Brightness', Int32)
        self.ON3_pub = rospy.Publisher('/MILIGHT/light3ON', Empty)
	self.color3_pub = rospy.Publisher('/MILIGHT/light3Color', Int32)
        self.brightness3_pub = rospy.Publisher('/MILIGHT/light3Brightness', Int32)
	self.french_pub = rospy.Publisher('/NESTOR/french_voice', String)

    def execute(self, userdata):

	tosay = String()
	myint = Int32()
	tosay.data = "Il est l'heure d'aller dormir."
	self.french_pub.publish(tosay)
        rospy.sleep(0.01)
        self.ON3_pub.publish(Empty())
        rospy.sleep(0.01)
        self.ON2_pub.publish(Empty())
        rospy.sleep(0.01)
        self.ON1_pub.publish(Empty())
        rospy.sleep(0.01)
	myint.data = 175
	self.color1_pub.publish(myint)
        rospy.sleep(0.01)
	self.color3_pub.publish(myint)
        rospy.sleep(0.01)
	myint.data = 2
	self.brightness1_pub.publish(myint)
        rospy.sleep(0.01)
	myint.data = 10
	self.brightness2_pub.publish(myint)
        rospy.sleep(0.01)
	myint.data = 9
	self.brightness3_pub.publish(myint)
        rospy.sleep(0.01)

        return 'succeeded'

class InBed(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded'])
        self.OFF1_pub = rospy.Publisher('/MILIGHT/light1OFF', Empty)
        self.brightness1_pub = rospy.Publisher('/MILIGHT/light1Brightness', Int32)
        self.OFF2_pub = rospy.Publisher('/MILIGHT/light2OFF', Empty)
        self.brightness2_pub = rospy.Publisher('/MILIGHT/light2Brightness', Int32)
        self.OFF3_pub = rospy.Publisher('/MILIGHT/light3OFF', Empty)
        self.brightness3_pub = rospy.Publisher('/MILIGHT/light3Brightness', Int32)
        self.heatOFF_pub = rospy.Publisher('/HOME/showerHeatOFF', Empty)
	self.french_pub = rospy.Publisher('/NESTOR/french_voice', String)

    def execute(self, userdata):

        tosay = String()
        myint = Int32()
        tosay.data = "Bonne nuit."
        self.french_pub.publish(tosay)
        rospy.sleep(0.01)

	myint.data = 1
        self.brightness1_pub.publish(myint)
        rospy.sleep(0.01)
        myint.data = 6
        self.brightness2_pub.publish(myint)
        rospy.sleep(0.01)
        myint.data = 6
        self.brightness3_pub.publish(myint)
        rospy.sleep(0.3)

	# Shutdown everything
        self.OFF3_pub.publish(Empty())
        rospy.sleep(0.01)
        self.OFF2_pub.publish(Empty())
        rospy.sleep(0.01)
        self.OFF1_pub.publish(Empty())
        rospy.sleep(0.01)
	self.heatOFF_pub.publish(Empty())
        rospy.sleep(0.01)

        return 'succeeded'

class TimeoutToBed(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'],input_keys=[],output_keys=[])
        self.stop=0
	pass

    def execute(self, userdata):
	for bla in range(1,1800):
        	rospy.sleep(1) #30min
		if self.stop == 1:
			break
	self.stop=0
        return 'succeeded'

    def request_preempt(self):
        """Overload the preempt request method just to spew an error."""
	self.stop=1
        State.request_preempt(self)
        rospy.logwarn("Preempted!")



class Stop(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])

	self.cmd_vel_pub = rospy.Publisher('/PETIT/cmd_vel', Twist)
        pass

    def execute(self, userdata):
	self.cmd_vel_pub.publish(Twist())
        rospy.loginfo("Shutting down the state machine")
        return 'succeeded'

class Pause(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        pass

    def execute(self, userdata):
        rospy.loginfo("Thinking...")
        rospy.sleep(0.5)   
        return 'succeeded'


class SMACHAI():
    def __init__(self):
        rospy.init_node('HOME_automation_smach', anonymous=False)
        
        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)
        

        # Create a list to hold the target quaternions (orientations)
        quaternions = list()

        # First define the corner orientations as Euler angles
        euler_angles = (pi/2, pi, 3*pi/2, 0)

        # Then convert the angles to quaternions
        for angle in euler_angles:
            q_angle = quaternion_from_euler(0, 0, angle, axes='sxyz')
            q = Quaternion(*q_angle)
            quaternions.append(q)

        # Create a list to hold the waypoint poses
        self.waypoints = list()

	self.square_size = 1.0

        # Append each of the four waypoints to the list.  Each waypoint
        # is a pose consisting of a position and orientation in the map frame.
        self.waypoints.append(Pose(Point(0.0, 0.0, 0.0), quaternions[3]))
        self.waypoints.append(Pose(Point(self.square_size, 0.0, 0.0), quaternions[0]))
        self.waypoints.append(Pose(Point(self.square_size, self.square_size, 0.0), quaternions[1]))
        self.waypoints.append(Pose(Point(0.0, self.square_size, 0.0), quaternions[2]))



	# State machine for light entry 
        self.sm_light_entry = StateMachine(outcomes=['succeeded','aborted','preempted'])
        self.sm_light_entry.userdata.day_mode = 1;

        with self.sm_light_entry:
            StateMachine.add('LOOK_ENTRY', MonitorState("/HOME/entry_move", Empty, self.empty_cb),
					  transitions={'valid':'LOOK_ENTRY',
					  'invalid':'LIGHT_UP'})
            StateMachine.add('LIGHT_UP', LightEntry(),
                             transitions={'succeeded':'succeeded'})

        # State machine for dark entry 
        self.sm_dark_entry = StateMachine(outcomes=['succeeded','aborted','preempted'])
        self.sm_dark_entry.userdata.day_mode = 1;

        with self.sm_dark_entry:
            StateMachine.add('LOOK_ENTRY_OFF', MonitorState("/HOME/entry_noOne", Empty, self.empty_cb),
                                          transitions={'valid':'LOOK_ENTRY_OFF',
                                          'invalid':'LIGHT_DOWN'})
            StateMachine.add('LIGHT_DOWN', DarkEntry(),
                             transitions={'succeeded':'succeeded'})


	# State machine for day mode
        self.sm_day_mode = Concurrence(outcomes=['succeeded','aborted','preempted','go_shower','go_sleep','go_eat','go_out'],
					default_outcome='succeeded',
                                        child_termination_cb=self.daymode_child_termination_cb,
                                        outcome_cb=self.daymode_outcome_cb)
	self.sm_day_mode.userdata.day_mode = 1;

        with self.sm_day_mode:
	    Concurrence.add('LOOK_SHOWER', MonitorState("/HOME/go_shower", Empty, self.empty_cb))
	    Concurrence.add('LOOK_LEAVING', MonitorState("/HOME/leaving_home", Empty, self.empty_cb))
	    Concurrence.add('LOOK_SLEEP', MonitorState("/HOME/go_sleep", Empty, self.empty_cb))
	    Concurrence.add('LOOK_EAT', MonitorState("/HOME/go_eat", Empty, self.empty_cb))
	    Concurrence.add('LOOK_ENTRY', self.sm_light_entry)
	    Concurrence.add('LOOK_ENTRY_OFF', self.sm_dark_entry)


        # State machine for leaving home
        self.sm_leaving_home = StateMachine(outcomes=['succeeded','aborted','preempted'])

        with self.sm_leaving_home:
            StateMachine.add('LEAV', Pause(),
                             transitions={'succeeded':'succeeded',
                                          'aborted':'aborted'})

        # State machine for going to sleep
        self.sm_going_sleep = StateMachine(outcomes=['succeeded','aborted','preempted'])

        with self.sm_going_sleep:
            StateMachine.add('GOING_SLEEP', GoingSleep(),
                             transitions={'succeeded':'succeeded'})


	# State machine for day mode
        self.sm_wait_bed = Concurrence(outcomes=['succeeded','aborted','preempted'],
                                        default_outcome='succeeded',
                                        child_termination_cb=self.useless_child_termination_cb,
                                        outcome_cb=self.useless_outcome_cb)

        with self.sm_wait_bed:
            Concurrence.add('LOOK_INBED', MonitorState("/METAWATCH/button2", Empty, self.empty_cb))
            Concurrence.add('TIMEOUT', TimeoutToBed())


	# State machine for in bed
        self.sm_in_bed = StateMachine(outcomes=['succeeded','aborted','preempted'])

        with self.sm_in_bed:
            StateMachine.add('IN_BED', InBed(),
                             transitions={'succeeded':'succeeded'})



	# State machine for light entry 
        self.sm_lightn_entry = StateMachine(outcomes=['succeeded','aborted','preempted'])
        self.sm_lightn_entry.userdata.day_mode = 0;

        with self.sm_lightn_entry:
            StateMachine.add('LOOK_ENTRY', MonitorState("/HOME/entry_move", Empty, self.empty_cb),
                                          transitions={'valid':'LOOK_ENTRY',
                                          'invalid':'LIGHT_UP'})
            StateMachine.add('LIGHT_UP', LightEntry(),
                             transitions={'succeeded':'succeeded'})


	# State machine for night mode
        self.sm_night_mode = Concurrence(outcomes=['succeeded','aborted','preempted','wake_up'],
                                        default_outcome='succeeded',
                                        child_termination_cb=self.nightmode_child_termination_cb,
                                        outcome_cb=self.nightmode_outcome_cb)
        self.sm_night_mode.userdata.day_mode = 0;

        with self.sm_night_mode:
            Concurrence.add('LOOK_WAKE', MonitorState("/HOME/wake_up", Empty, self.empty_cb))
            Concurrence.add('LOOK_ENTRY', self.sm_lightn_entry)
            Concurrence.add('LOOK_ENTRY_OFF', self.sm_dark_entry)


        # State machine for night mode
        #self.sm_night_mode = StateMachine(outcomes=['succeeded','aborted','preempted'])
	#self.sm_night_mode.userdata.day_mode = 0;

        #with self.sm_night_mode:
        #    StateMachine.add('NIGHT_MOD', Pause(),
        #                     transitions={'succeeded':'succeeded',
        #                                  'aborted':'aborted'})

        # State machine for waking up 
        self.sm_waking_up = StateMachine(outcomes=['succeeded','aborted','preempted'])
        self.sm_waking_up.userdata.day_mode = 0;

        with self.sm_waking_up:
            StateMachine.add('WAKING_UP', WakingUp(),
                             transitions={'succeeded':'succeeded'})

	# State machine for waking up 
        self.sm_going_eat = StateMachine(outcomes=['succeeded','aborted','preempted'])
        self.sm_going_eat.userdata.day_mode = 1;

        with self.sm_going_eat:
            StateMachine.add('EATTTTTTTT', Pause(),
                             transitions={'succeeded':'succeeded',
                                          'aborted':'aborted'})





	# State machine for home
        self.sm_home = StateMachine(outcomes=['succeeded','aborted','preempted','going_out'])

        with self.sm_home:
            StateMachine.add('DAY_MODE', self.sm_day_mode,
                             transitions={'succeeded':'DAY_MODE',
					  'go_shower':'PREPARING_SHOWER',
                                          'go_sleep':'GOING_SLEEP',
                                          'go_eat':'GOING_EAT',
                                          'go_out':'LEAVING_HOME',
                                          'aborted':'aborted'})
            StateMachine.add('PREPARING_SHOWER', PreparingShower(),
                             transitions={'succeeded':'GO_SHOWER',
                                          'aborted':'aborted'})
            StateMachine.add('GO_SHOWER', GoShower(),
                             transitions={'succeeded':'STOP_SHOWER',
                                          'aborted':'aborted'})
            StateMachine.add('STOP_SHOWER', StopShower(),
                             transitions={'succeeded':'DAY_MODE',
                                          'aborted':'aborted'})
            StateMachine.add('LEAVING_HOME', self.sm_leaving_home,
                             transitions={'succeeded':'going_out',
                                          'aborted':'aborted'})
            StateMachine.add('GOING_SLEEP', self.sm_going_sleep,
                             transitions={'succeeded':'WAIT_TO_BED',
                                          'aborted':'aborted'})
	    StateMachine.add('WAIT_TO_BED', self.sm_wait_bed,
                             transitions={'succeeded':'IN_BED',
					  'preempted':'IN_BED',
                                          'aborted':'aborted'})
            StateMachine.add('IN_BED', self.sm_in_bed,
                             transitions={'succeeded':'NIGHT_MODE',
                                          'aborted':'aborted'})
            StateMachine.add('NIGHT_MODE', self.sm_night_mode,
                             transitions={'succeeded':'NIGHT_MODE',
					  'wake_up':'WAKING_UP',
                                          'aborted':'aborted'})
            StateMachine.add('WAKING_UP', self.sm_waking_up,
                             transitions={'succeeded':'DAY_MODE',
                                          'aborted':'aborted'})
            StateMachine.add('GOING_EAT', self.sm_going_eat,
                             transitions={'succeeded':'DAY_MODE',
                                          'aborted':'aborted'})






	# State machine for waking up 
        self.sm_guarding = StateMachine(outcomes=['succeeded','aborted','preempted'])

        with self.sm_guarding:
            StateMachine.add('GUARD', Pause(),
                             transitions={'succeeded':'succeeded',
                                          'aborted':'aborted'})



	# State machine with concurrence
        self.sm_incoming_home = Concurrence(outcomes=['succeeded', 'aborted'],
                                        default_outcome='succeeded',
                                        child_termination_cb=self.incoming_child_termination_cb,
                                        outcome_cb=self.incoming_outcome_cb)

        # Add the sm_actions machine and a battery MonitorState to the nav_patrol machine             
        with self.sm_incoming_home:
           Concurrence.add('LOOK_CONNECTION', MonitorState("/METAWATCH/connected", Empty, self.empty_cb))
           Concurrence.add('LOOK_ENTERING', MonitorState("/HOME/entry_door_open", Empty, self.empty_cb))





        # State machine for away
        self.sm_away = StateMachine(outcomes=['succeeded','aborted','preempted','entering_home'])

        with self.sm_away:
            StateMachine.add('GUARDING_MODE', self.sm_guarding,
                             transitions={'succeeded':'INCOMING_HOME',
                                          'aborted':'aborted'})
            StateMachine.add('INCOMING_HOME', self.sm_incoming_home,
                             transitions={'succeeded':'entering_home',
                                          'aborted':'aborted'})




        # Create the top level state machine
        self.sm_top = StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])

        # Add nav_patrol, sm_recharge and a Stop() machine to sm_top
        with self.sm_top:
            StateMachine.add('AT_HOME', self.sm_home, transitions={'succeeded':'AT_HOME', 'going_out':'AWAY'})
            #StateMachine.add('RECHARGE', self.sm_recharge, transitions={'succeeded':'PATROL'})
            StateMachine.add('AWAY', self.sm_away, transitions={'succeeded':'AWAY', 'entering_home':'AT_HOME'})








        # Create and start the SMACH introspection server
        intro_server = IntrospectionServer('patrol', self.sm_top, '/SM_ROOT')
        intro_server.start()
        
        # Execute the state machine
        sm_outcome = self.sm_top.execute()
        
        rospy.loginfo('State Machine Outcome: ' + str(sm_outcome))
                
        intro_server.stop()



    def empty_cb(self, userdata, msg):
	#rospy.loginfo("Empty message received.")
        return False

    # Gets called when ANY child state terminates
    def useless_child_termination_cb(self, outcome_map):
	rospy.loginfo("useless_child_termination_cb.")
        return True


    # Gets called when ALL child states are terminated
    def useless_outcome_cb(self, outcome_map):
        rospy.loginfo("useless_outcome_cb.")
        return 'succeeded'


    # Gets called when ANY child state terminates
    def daymode_child_termination_cb(self, outcome_map):
	#rospy.loginfo("daymode_child_termination_cb.")
	return True


    # Gets called when ALL child states are terminated
    def daymode_outcome_cb(self, outcome_map):
        #rospy.loginfo("daymode_outcome_cb.")
	# If the battery is below threshold, return the 'recharge' outcome
        if outcome_map['LOOK_SHOWER'] == 'invalid':
            rospy.loginfo("Going to shower ")
            return 'go_shower'
        if outcome_map['LOOK_LEAVING'] == 'invalid':
            rospy.loginfo("Leaving home")
            return 'go_out'
        if outcome_map['LOOK_SLEEP'] == 'invalid':
            rospy.loginfo("Going to sleep")
            return 'go_sleep'
        if outcome_map['LOOK_EAT'] == 'invalid':
            rospy.loginfo("Going to eat")
            return 'go_eat'
        if outcome_map['LOOK_ENTRY'] == 'succeeded':
            rospy.loginfo("Restart looking entry")
            return 'succeeded'
        if outcome_map['LOOK_ENTRY_OFF'] == 'succeeded':
            rospy.loginfo("No one in the entry")
            return 'succeeded'
        else:
            return 'aborted'


    # Gets called when ANY child state terminates
    def nightmode_child_termination_cb(self, outcome_map):
        #rospy.loginfo("daymode_child_termination_cb.")
        return True


    # Gets called when ALL child states are terminated
    def nightmode_outcome_cb(self, outcome_map):
        #rospy.loginfo("daymode_outcome_cb.")
        # If the battery is below threshold, return the 'recharge' outcome
        if outcome_map['LOOK_WAKE'] == 'invalid':
            rospy.loginfo("Wake up dude !")
            return 'wake_up'
        if outcome_map['LOOK_ENTRY'] == 'succeeded':
            rospy.loginfo("Restart looking entry")
            return 'succeeded'
        if outcome_map['LOOK_ENTRY_OFF'] == 'succeeded':
            rospy.loginfo("No one in the entry")
            return 'succeeded'
        else:
            return 'aborted'

    # Gets called when ANY child state terminates
    def incoming_child_termination_cb(self, outcome_map):
        # If the current navigation task has succeeded, return True
        if outcome_map['LOOK_CONNECTION'] == 'succeeded':
            rospy.loginfo("MW connected. Welcome back.")
            return True
        # If the MonitorState state returns False (invalid), store the current nav goal and recharge
        if outcome_map['LOOK_ENTERING'] == 'succeeded':
            rospy.loginfo("Someone entering...")
            return True
        else:
            return False


    # Gets called when ALL child states are terminated
    def incoming_outcome_cb(self, outcome_map):
        # If the battery is below threshold, return the 'recharge' outcome
        if outcome_map['LOOK_CONNECTION'] == 'succeeded':
            rospy.loginfo("MW connected. Welcome back ")
            return 'succeeded'
        if outcome_map['LOOK_ENTERING'] == 'succeeded':
            rospy.loginfo("Someone entering..")
            return 'succeeded'
        else:
            return 'aborted'







    def time_cb(self, userdata, msg):
        if msg.data < 2:
            self.stopping = True
            return False
        else:
            self.stopping = False
            return True

    def battery_cb(self, userdata, msg):
        if msg.data < 320:
            self.recharging = True
            return False
        else:
            self.recharging = False
            return True

    def objective_cb(self, userdata, response):
        #objective_response = GetObjective().Response
	userdata.waypoint_out = response.goal
	waypoint_type = response.type.data

	rospy.loginfo("goal: " + str(response.goal))

	if(waypoint_type == 1):
                return 'action1'
        if(waypoint_type == 2):
                return 'action2'
        if(waypoint_type == 3):
                return 'action3'
        if(waypoint_type == 4):
                return 'action4'
        if(waypoint_type == 5):
                return 'action5'
        if(waypoint_type == 6):
                return 'action6'
        return 'aborted'
	
    # Gets called when ANY child state terminates
    def concurrence_child_termination_cb(self, outcome_map):
        # If the current navigation task has succeeded, return True
        if outcome_map['SM_ACTIONS'] == 'succeeded':
            return True
        # If the MonitorState state returns False (invalid), store the current nav goal and recharge
        if outcome_map['MONITOR_TIME'] == 'invalid':
            rospy.loginfo("LOW TIME! NEED TO STOP...")
            return True
        if outcome_map['MONITOR_BATTERY'] == 'invalid':
            rospy.loginfo("LOW BATTERY! NEED TO STOP...")
            return True
        else:
            return False


    # Gets called when ALL child states are terminated
    def concurrence_outcome_cb(self, outcome_map):
        # If the battery is below threshold, return the 'recharge' outcome
        if outcome_map['MONITOR_TIME'] == 'invalid':
	    rospy.loginfo("TIME FINISHED !! GOING TO STOP ! ")
            return 'stop'
        if outcome_map['MONITOR_BATTERY'] == 'invalid':
            return 'stop'
        # Otherwise, if the last nav goal succeeded, return 'succeeded' or 'stop'
        elif outcome_map['SM_ACTIONS'] == 'succeeded':
            #self.patrol_count += 1
            #rospy.loginfo("FINISHED PATROL LOOP: " + str(self.patrol_count))
            # If we have not completed all our patrols, start again at the beginning
            #if self.n_patrols == -1 or self.patrol_count < self.n_patrols:
                #self.sm_nav.set_initial_state(['NAV_STATE_0'], UserData())
            return 'succeeded'
            # Otherwise, we are finished patrolling so return 'stop'
            #else:
                #self.sm_nav.set_initial_state(['NAV_STATE_4'], UserData())
                #return 'stop'
        # Recharge if all else fails
        else:
            return 'recharge'


    def shutdown(self):
        rospy.loginfo("Stopping home automation...")
        
        self.sm_day_mode.request_preempt()
        
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        SMACHAI()
    except rospy.ROSInterruptException:
        rospy.loginfo("HOME automation finished.")
