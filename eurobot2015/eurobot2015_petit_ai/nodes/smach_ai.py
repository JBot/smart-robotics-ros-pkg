#!/usr/bin/env python

""" random_patrol_smach.py - Version 1.0 2013-04-12

    Control a robot to patrol four waypoints chosen at random

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2014 Patrick Goebel.  All rights reserved.

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
from std_msgs.msg import Int32, Empty
from random import randrange
from tf.transformations import quaternion_from_euler
from math import  pi
from collections import OrderedDict
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from common_smart_ai.srv import GetObjective, UpdatePriority

class Stop(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])

	self.cmd_vel_pub = rospy.Publisher('/PETIT/cmd_vel', Twist)
        pass

    def execute(self, userdata):
	self.cmd_vel_pub.publish(Twist())
        rospy.loginfo("Shutting down the state machine")
        return 'succeeded'

class PickWaypoint(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted','preempted','action1','action2','action3','action4','action5','action6'], input_keys=['waypoints'], output_keys=['waypoint_out'])
    
    def execute(self, userdata):   
        waypoint_out = randrange(len(userdata.waypoints))
        
        userdata.waypoint_out = waypoint_out
        
        rospy.loginfo("Going to waypoint " + str(waypoint_out))
    	
	if(waypoint_out == 1):
        	return 'action1'
	if(waypoint_out == 2):
        	return 'action2'
	if(waypoint_out == 3):
        	return 'action3'
        return 'action6'

class PickObjective(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted','preempted','action1','action2','action3','action4','action5','action6'], output_keys=['waypoint_out'])

	self.get_objective = GetObjective()

    def execute(self, userdata):

	self.get_objective.send_request(GetObjectiveRequest())

	finished_within_time = self.get_objective.wait_for_response(rospy.Duration(10))

	response = self.get_objective.get_response()

	waypoint_out = response.goal
        waypoint_type = response.type
	#rospy.loginfo("Going to waypoint " + str(waypoint_out))

        if(waypoint_type.data == 1):
                return 'action1'
        if(waypoint_type.data == 2):
                return 'action2'
        if(waypoint_type.data == 3):
                return 'action3'
        if(waypoint_type.data == 4):
                return 'action4'
        if(waypoint_type.data == 5):
                return 'action5'
        if(waypoint_type.data == 6):
                return 'action6'
        return 'aborted'

class RemoveObjective(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'], input_keys=['waypoint_in'])

        self.remove_objective = rospy.Publisher('/PETIT/delet_objective', PoseStamped)

    def execute(self, userdata):

	waypoint = PoseStamped()
	waypoint = userdata.waypoint_in
	self.remove_objective.publish(waypoint)

	return 'succeeded'
        
class Nav2Waypoint(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                       input_keys=['waypoint_in'], output_keys=['waypoint_out'])
        
        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("PETIT_pathplanner", MoveBaseAction)
        
        # Wait up to 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))    
        
        rospy.loginfo("Connected to move_base action server")
        
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'petit_map'

    def execute(self, userdata):
        self.goal.target_pose = userdata.waypoint_in #userdata.waypoints[userdata.waypoint_in]
     	userdata.waypoint_out = userdata.waypoint_in

	rospy.loginfo("waypoint_in: " + str(userdata.waypoint_in))

        # Send the goal pose to the MoveBaseAction server
        self.move_base.send_goal(self.goal)
        
	i = 0
	finished_within_time = 0
	while i < 60 and not finished_within_time:
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
        
            # Allow 1 minute to get there
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(1)) 
	    #rospy.loginfo('finished ' + str(finished_within_time))
        
        # If we don't get there in time, abort the goal
        if not finished_within_time:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
            return 'aborted'
        else:
            # We made it!
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal succeeded!")
                #rospy.sleep(1)   
            return 'succeeded'

class MoveForward(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])

        self.cmd_vel_pub = rospy.Publisher('/PETIT/cmd_vel', Twist)
        self.pause_pub = rospy.Publisher('/PETIT/pause_pathwrapper', Empty)
        self.resume_pub = rospy.Publisher('/PETIT/resume_pathwrapper', Empty)
        pass

    def execute(self, userdata):
        rospy.loginfo("moving forward")
        self.pause_pub.publish(Empty())
	tmp_twist = Twist()
	tmp_twist.linear.x = 0.1
        self.cmd_vel_pub.publish(tmp_twist)
        rospy.sleep(1)   
        self.resume_pub.publish(Empty())
        return 'succeeded'


class SMACHAI():
    def __init__(self):
        rospy.init_node('petit_smach_ai', anonymous=False)
        
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

	# Publisher to manually control the robot (e.g. to stop it)
    	self.cmd_vel_pub = rospy.Publisher('/PETIT/cmd_vel', Twist)

        self.stopping = False
        self.recharging = False







	# State machine for Action1
        self.sm_action1 = StateMachine(outcomes=['succeeded','aborted','preempted'], input_keys=['waypoint_in'], output_keys=['waypoint_out'])

        with self.sm_action1:            
            StateMachine.add('NAV_WAYPOINT', Nav2Waypoint(),
                             transitions={'succeeded':'succeeded',
                                          'aborted':'aborted'})
            
	# State machine for Action2
        self.sm_action2 = StateMachine(outcomes=['succeeded','aborted','preempted'], input_keys=['waypoint_in'], output_keys=['waypoint_out'])

        with self.sm_action2:
            StateMachine.add('NAV_WAYPOINT', Nav2Waypoint(),
                             transitions={'succeeded':'succeeded',
                                          'aborted':'aborted'})

	# State machine for Action3
        self.sm_action3 = StateMachine(outcomes=['succeeded','aborted','preempted'], input_keys=['waypoint_in'], output_keys=['waypoint_out'])

        with self.sm_action3:
            StateMachine.add('NAV_WAYPOINT', Nav2Waypoint(),
                             transitions={'succeeded':'succeeded',
                                          'aborted':'aborted'})

	# State machine for Action4
        self.sm_action4 = StateMachine(outcomes=['succeeded','aborted','preempted'], input_keys=['waypoint_in'], output_keys=['waypoint_out'])

        with self.sm_action4:
            StateMachine.add('NAV_WAYPOINT', Nav2Waypoint(),
                             transitions={'succeeded':'succeeded',
                                          'aborted':'aborted'})

	# State machine for Action5
        self.sm_action5 = StateMachine(outcomes=['succeeded','aborted','preempted'], input_keys=['waypoint_in'], output_keys=['waypoint_out'])

        with self.sm_action5:
            StateMachine.add('NAV_WAYPOINT', Nav2Waypoint(),
                             transitions={'succeeded':'succeeded',
                                          'aborted':'aborted'})

	# State machine for Action6
        self.sm_action6 = StateMachine(outcomes=['succeeded','aborted','preempted'], input_keys=['waypoint_in'], output_keys=['waypoint_out'])

        with self.sm_action6:
            StateMachine.add('NAV_WAYPOINT', Nav2Waypoint(),
                             transitions={'succeeded':'succeeded',
                                          'aborted':'aborted'})



	# State machine for Actions
        self.sm_actions = StateMachine(outcomes=['succeeded','aborted','preempted'])
        self.sm_actions.userdata.waypoints = self.waypoints

        with self.sm_actions:
	    StateMachine.add('PICK_WAYPOINT', ServiceState('/PETIT/get_objective', GetObjective, response_cb=self.objective_cb,
			     output_keys=['waypoint_out'],
			     outcomes=['action1','action2','action3','action4','action5','action6','aborted','succeeded','preempted']),
                             transitions={'action1':'SM_ACTION1','action2':'SM_ACTION2','action3':'SM_ACTION3','action4':'SM_ACTION4','action5':'SM_ACTION5','action6':'SM_ACTION6','aborted':'SM_ACTION1'},
                             remapping={'waypoint_out':'patrol_waypoint'})
	    #StateMachine.add('PICK_WAYPOINT', PickWaypoint(),
            #                 transitions={'action1':'SM_ACTION1','action2':'SM_ACTION2','action3':'SM_ACTION3','action4':'SM_ACTION4','action5':'SM_ACTION5','action6':'SM_ACTION6','aborted':'SM_ACTION1'},
            #                 remapping={'waypoint_out':'patrol_waypoint'})
	    StateMachine.add('SM_ACTION1', self.sm_action1, transitions={'succeeded':'REMOVE_OBJECTIVE', 'aborted':'aborted'},
			     remapping={'waypoint_in':'patrol_waypoint',
					'waypoint_out':'remove_waypoint'})
	    StateMachine.add('SM_ACTION2', self.sm_action2, transitions={'succeeded':'REMOVE_OBJECTIVE', 'aborted':'aborted'},
			     remapping={'waypoint_in':'patrol_waypoint',
					'waypoint_out':'remove_waypoint'})
	    StateMachine.add('SM_ACTION3', self.sm_action3, transitions={'succeeded':'REMOVE_OBJECTIVE', 'aborted':'aborted'},
			     remapping={'waypoint_in':'patrol_waypoint',
					'waypoint_out':'remove_waypoint'})
	    StateMachine.add('SM_ACTION4', self.sm_action4, transitions={'succeeded':'REMOVE_OBJECTIVE', 'aborted':'aborted'},
			     remapping={'waypoint_in':'patrol_waypoint',
					'waypoint_out':'remove_waypoint'})
	    StateMachine.add('SM_ACTION5', self.sm_action5, transitions={'succeeded':'REMOVE_OBJECTIVE', 'aborted':'aborted'},
			     remapping={'waypoint_in':'patrol_waypoint',
					'waypoint_out':'remove_waypoint'})
	    StateMachine.add('SM_ACTION6', self.sm_action6, transitions={'succeeded':'REMOVE_OBJECTIVE', 'aborted':'aborted'},
			     remapping={'waypoint_in':'patrol_waypoint',
					'waypoint_out':'remove_waypoint'})
	    StateMachine.add('REMOVE_OBJECTIVE', RemoveObjective(),
                             transitions={'succeeded':'succeeded',
                                          'aborted':'aborted'},
			     remapping={'waypoint_in':'remove_waypoint'})


	# State machine with concurrence
        self.sm_concurrent = Concurrence(outcomes=['succeeded', 'stop'],
                                        default_outcome='succeeded',
                                        child_termination_cb=self.concurrence_child_termination_cb,
                                        outcome_cb=self.concurrence_outcome_cb)

        # Add the sm_actions machine and a battery MonitorState to the nav_patrol machine             
        with self.sm_concurrent:
           Concurrence.add('SM_ACTIONS', self.sm_actions)
           Concurrence.add('MONITOR_TIME', MonitorState("/GENERAL/remain", Int32, self.time_cb))
           Concurrence.add('MONITOR_BATTERY', MonitorState("/PETIT/adc", Int32, self.battery_cb))


        # Create the top level state machine
        self.sm_top = StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])

        # Add nav_patrol, sm_recharge and a Stop() machine to sm_top
        with self.sm_top:
            StateMachine.add('CONCURRENT', self.sm_concurrent, transitions={'succeeded':'CONCURRENT', 'stop':'STOP'})
            #StateMachine.add('RECHARGE', self.sm_recharge, transitions={'succeeded':'PATROL'})
            StateMachine.add('STOP', Stop(), transitions={'succeeded':''})








        # Create and start the SMACH introspection server
        intro_server = IntrospectionServer('patrol', self.sm_top, '/SM_ROOT')
        intro_server.start()
        
        # Execute the state machine
        sm_outcome = self.sm_top.execute()
        
        rospy.loginfo('State Machine Outcome: ' + str(sm_outcome))
                
        intro_server.stop()


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
        rospy.loginfo("Stopping the robot...")
        
        self.sm_actions.request_preempt()
        
        self.cmd_vel_pub.publish(Twist())
        
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        SMACHAI()
    except rospy.ROSInterruptException:
        rospy.loginfo("Petit AI finished.")
