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


class Nav2Waypoint(State):
    def __init__(self, goal):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                       input_keys=['waypoint_in'], output_keys=['waypoint_out'])

        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("CARRY_pathplanner", MoveBaseAction)

        # Wait up to 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))

        rospy.loginfo("Connected to move_base action server")

	self.my_goal = goal
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'world'

    def execute(self, userdata):
        self.goal.target_pose.pose = self.my_goal 
        self.goal.target_pose.header.frame_id = 'world'

        rospy.loginfo("waypoint_in: " + str(self.goal.target_pose))

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



class Stop(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])

	self.cmd_vel_pub = rospy.Publisher('/CARRY/cmd_vel', Twist)
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
        rospy.init_node('carry_smach_move', anonymous=False)
        
        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)
        

        # Create a list to hold the target quaternions (orientations)
        quaternions = list()

        # First define the corner orientations as Euler angles
        euler_angles = (0.0, 3*pi/2, pi/2, 3*pi/2, pi/2, 3*pi/2, 3*pi/2, 0.0, pi, 0.0, pi, 3*pi/2)

        # Then convert the angles to quaternions
        for angle in euler_angles:
            q_angle = quaternion_from_euler(0, 0, angle, axes='sxyz')
            q = Quaternion(*q_angle)
            quaternions.append(q)

        # Create a list to hold the waypoint poses
        self.waypoints = list()

        # Append each of the four waypoints to the list.  Each waypoint
        # is a pose consisting of a position and orientation in the map frame.
        self.waypoints.append(Pose(Point(0.0, -3.0, 0.0), quaternions[0]))
        self.waypoints.append(Pose(Point(2.0, -2.0, 0.0), quaternions[1]))
        self.waypoints.append(Pose(Point(2.9, 1.0, 0.0), quaternions[2]))
        self.waypoints.append(Pose(Point(3.0, 1.2, 0.0), quaternions[3]))
        self.waypoints.append(Pose(Point(4.23, 1.1, 0.0), quaternions[4]))
        self.waypoints.append(Pose(Point(4.23, 1.1, 0.0), quaternions[5]))
        self.waypoints.append(Pose(Point(4.4, -0.6, 0.0), quaternions[6]))
        self.waypoints.append(Pose(Point(2.3, 1.7, 0.0), quaternions[7]))
        self.waypoints.append(Pose(Point(2.3, 1.7, 0.0), quaternions[8]))
        self.waypoints.append(Pose(Point(-1.3, 0.7, 0.0), quaternions[9]))
        self.waypoints.append(Pose(Point(-1.3, 0.7, 0.0), quaternions[10]))
        self.waypoints.append(Pose(Point(-1.6, 0.1, 0.0), quaternions[11]))



	# State machine  
        #self.sm_out_main = StateMachine(outcomes=['succeeded','aborted','preempted'])
        #self.sm_out_main.userdata.goal = self.waypoints[2];

        #with self.sm_out_main:
	#   StateMachine.add('GO_OUT_MAIN_ROOM', Nav2Waypoint(),
        #                     transitions={'succeeded':'succeeded',
        #                                  'aborted':'aborted'}) 

	# Concurrent State machine 
        self.sm_in_main_room = Concurrence(outcomes=['succeeded','aborted','preempted','go_kitchen','go_bedroom','go_sleep'],
					default_outcome='succeeded',
                                        child_termination_cb=self.in_main_room_child_termination_cb,
                                        outcome_cb=self.in_main_room_outcome_cb)
	self.sm_in_main_room.userdata.goal = self.waypoints[1];

        with self.sm_in_main_room:
	    Concurrence.add('GO_TO_KITCHEN', MonitorState("/CARRY/go_kitchen", Empty, self.empty_cb))
	    Concurrence.add('GO_TO_BEDROOM', MonitorState("/CARRY/go_bedroom", Empty, self.empty_cb))
	    Concurrence.add('GO_TO_SLEEP', MonitorState("/CARRY/go_sleep", Empty, self.empty_cb))
	    #Concurrence.add('GO_TO_POINT', Nav2Waypoint(self.waypoints[1]))


        # Concurrent State machine 
        self.sm_sleep = Concurrence(outcomes=['succeeded','aborted','preempted','go_kitchen','go_bedroom','go_main_room'],
                                        default_outcome='succeeded',
                                        child_termination_cb=self.in_sleep_child_termination_cb,
                                        outcome_cb=self.in_sleep_outcome_cb)
        self.sm_sleep.userdata.goal = self.waypoints[0];

        with self.sm_sleep:
            Concurrence.add('GO_TO_KITCHEN', MonitorState("/CARRY/go_kitchen", Empty, self.empty_cb))
            Concurrence.add('GO_TO_BEDROOM', MonitorState("/CARRY/go_bedroom", Empty, self.empty_cb))
            Concurrence.add('GO_TO_MAIN_ROOM', MonitorState("/CARRY/go_main_room", Empty, self.empty_cb))
            #Concurrence.add('GO_TO_POINT', Nav2Waypoint(self.waypoints[0]))

	# Concurrent State machine 
        self.sm_in_kitchen = Concurrence(outcomes=['succeeded','aborted','preempted','go_main_room','go_bedroom','go_sleep'],
                                        default_outcome='succeeded',
                                        child_termination_cb=self.in_kitchen_child_termination_cb,
                                        outcome_cb=self.in_kitchen_outcome_cb)
        self.sm_in_kitchen.userdata.goal = self.waypoints[6];

        with self.sm_in_kitchen:
            Concurrence.add('GO_TO_MAIN_ROOM', MonitorState("/CARRY/go_main_room", Empty, self.empty_cb))
            Concurrence.add('GO_TO_BEDROOM', MonitorState("/CARRY/go_bedroom", Empty, self.empty_cb))
            Concurrence.add('GO_TO_SLEEP', MonitorState("/CARRY/go_sleep", Empty, self.empty_cb))
            #Concurrence.add('GO_TO_POINT', Nav2Waypoint(self.waypoints[6]))

        # Concurrent State machine 
        self.sm_in_bedroom = Concurrence(outcomes=['succeeded','aborted','preempted','go_main_room','go_kitchen','go_sleep'],
                                        default_outcome='succeeded',
                                        child_termination_cb=self.in_bedroom_child_termination_cb,
                                        outcome_cb=self.in_bedroom_outcome_cb)
        self.sm_in_bedroom.userdata.goal = self.waypoints[11];

        with self.sm_in_bedroom:
            Concurrence.add('GO_TO_MAIN_ROOM', MonitorState("/CARRY/go_main_room", Empty, self.empty_cb))
            Concurrence.add('GO_TO_KITCHEN', MonitorState("/CARRY/go_kitchen", Empty, self.empty_cb))
            Concurrence.add('GO_TO_SLEEP', MonitorState("/CARRY/go_sleep", Empty, self.empty_cb))
            #Concurrence.add('GO_TO_POINT', Nav2Waypoint(self.waypoints[11]))




	# Create the top level state machine
        self.sm_top = StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])

        # Add nav_patrol, sm_recharge and a Stop() machine to sm_top
        with self.sm_top:
            StateMachine.add('IN_MAIN_ROOM', self.sm_in_main_room, transitions={'succeeded':'IN_MAIN_ROOM', 
										'go_kitchen':'NAV_M2K_M',
										'go_sleep':'IN_SLEEP',
										'go_bedroom':'NAV_M2B_M'})
            StateMachine.add('IN_KITCHEN', self.sm_in_kitchen, transitions={'succeeded':'succeeded', 
                                                                                'go_main_room':'NAV_K2M_K',
                                                                                'go_sleep':'NAV_K2S_K',
                                                                                'go_bedroom':'NAV_K2B_K'})
            StateMachine.add('IN_BEDROOM', self.sm_in_bedroom, transitions={'succeeded':'succeeded', 
                                                                                'go_kitchen':'NAV_B2K_B',
                                                                                'go_sleep':'NAV_B2S_B',
                                                                                'go_main_room':'NAV_B2M_B'})
            StateMachine.add('IN_SLEEP', self.sm_sleep, transitions={'succeeded':'succeeded', 
                                                                                'go_kitchen':'NAV_S2K_M',
                                                                                'go_main_room':'IN_MAIN_ROOM',
                                                                                'go_bedroom':'NAV_S2B_M'})
	    StateMachine.add('NAV_M2K_M', Nav2Waypoint(self.waypoints[2]),
                             transitions={'succeeded':'NAV_M2K_K',
                                          'aborted':'aborted'})
            StateMachine.add('NAV_M2K_K', Nav2Waypoint(self.waypoints[5]),
                             transitions={'succeeded':'NAV_M2K_END',
                                          'aborted':'aborted'})
            StateMachine.add('NAV_M2K_END', Nav2Waypoint(self.waypoints[6]),
                             transitions={'succeeded':'IN_KITCHEN',
                                          'aborted':'aborted'})

            StateMachine.add('NAV_K2M_K', Nav2Waypoint(self.waypoints[4]),
                             transitions={'succeeded':'NAV_K2M_M',
                                          'aborted':'aborted'})
            StateMachine.add('NAV_K2M_M', Nav2Waypoint(self.waypoints[3]),
                             transitions={'succeeded':'NAV_K2M_END',
                                          'aborted':'aborted'})
            StateMachine.add('NAV_K2M_END', Nav2Waypoint(self.waypoints[1]),
                             transitions={'succeeded':'IN_MAIN_ROOM',
                                          'aborted':'aborted'})

            StateMachine.add('NAV_M2B_M', Nav2Waypoint(self.waypoints[2]),
                             transitions={'succeeded':'NAV_M2B_C',
                                          'aborted':'aborted'})
            StateMachine.add('NAV_M2B_C', Nav2Waypoint(self.waypoints[8]),
                             transitions={'succeeded':'NAV_M2B_B',
                                          'aborted':'aborted'})
            StateMachine.add('NAV_M2B_B', Nav2Waypoint(self.waypoints[10]),
                             transitions={'succeeded':'NAV_M2B_END',
                                          'aborted':'aborted'})
            StateMachine.add('NAV_M2B_END', Nav2Waypoint(self.waypoints[11]),
                             transitions={'succeeded':'IN_BEDROOM',
                                          'aborted':'aborted'})

            StateMachine.add('NAV_K2S_K', Nav2Waypoint(self.waypoints[4]),
                             transitions={'succeeded':'NAV_K2S_M',
                                          'aborted':'aborted'})
            StateMachine.add('NAV_K2S_M', Nav2Waypoint(self.waypoints[3]),
                             transitions={'succeeded':'NAV_K2S_END',
                                          'aborted':'aborted'})
            StateMachine.add('NAV_K2S_END', Nav2Waypoint(self.waypoints[0]),
                             transitions={'succeeded':'IN_SLEEP',
                                          'aborted':'aborted'})

            StateMachine.add('NAV_K2B_K', Nav2Waypoint(self.waypoints[4]),
                             transitions={'succeeded':'NAV_K2B_C',
                                          'aborted':'aborted'})
            StateMachine.add('NAV_K2B_C', Nav2Waypoint(self.waypoints[8]),
                             transitions={'succeeded':'NAV_K2B_B',
                                          'aborted':'aborted'})
            StateMachine.add('NAV_K2B_B', Nav2Waypoint(self.waypoints[10]),
                             transitions={'succeeded':'NAV_K2B_END',
                                          'aborted':'aborted'})
            StateMachine.add('NAV_K2B_END', Nav2Waypoint(self.waypoints[11]),
                             transitions={'succeeded':'IN_BEDROOM',
                                          'aborted':'aborted'})

            StateMachine.add('NAV_B2K_B', Nav2Waypoint(self.waypoints[9]),
                             transitions={'succeeded':'NAV_B2K_C',
                                          'aborted':'aborted'})
            StateMachine.add('NAV_B2K_C', Nav2Waypoint(self.waypoints[7]),
                             transitions={'succeeded':'NAV_B2K_K',
                                          'aborted':'aborted'})
            StateMachine.add('NAV_B2K_K', Nav2Waypoint(self.waypoints[5]),
                             transitions={'succeeded':'NAV_B2K_END',
                                          'aborted':'aborted'})
            StateMachine.add('NAV_B2K_END', Nav2Waypoint(self.waypoints[6]),
                             transitions={'succeeded':'IN_KITCHEN',
                                          'aborted':'aborted'})

            StateMachine.add('NAV_B2S_B', Nav2Waypoint(self.waypoints[9]),
                             transitions={'succeeded':'NAV_B2S_C',
                                          'aborted':'aborted'})
            StateMachine.add('NAV_B2S_C', Nav2Waypoint(self.waypoints[7]),
                             transitions={'succeeded':'NAV_B2S_M',
                                          'aborted':'aborted'})
            StateMachine.add('NAV_B2S_M', Nav2Waypoint(self.waypoints[3]),
                             transitions={'succeeded':'NAV_B2S_END',
                                          'aborted':'aborted'})
            StateMachine.add('NAV_B2S_END', Nav2Waypoint(self.waypoints[0]),
                             transitions={'succeeded':'IN_SLEEP',
                                          'aborted':'aborted'})

            StateMachine.add('NAV_B2M_B', Nav2Waypoint(self.waypoints[9]),
                             transitions={'succeeded':'NAV_B2M_C',
                                          'aborted':'aborted'})
            StateMachine.add('NAV_B2M_C', Nav2Waypoint(self.waypoints[7]),
                             transitions={'succeeded':'NAV_B2M_M',
                                          'aborted':'aborted'})
            StateMachine.add('NAV_B2M_M', Nav2Waypoint(self.waypoints[3]),
                             transitions={'succeeded':'NAV_B2M_END',
                                          'aborted':'aborted'})
            StateMachine.add('NAV_B2M_END', Nav2Waypoint(self.waypoints[1]),
                             transitions={'succeeded':'IN_MAIN_ROOM',
                                          'aborted':'aborted'})

            StateMachine.add('NAV_S2B_M', Nav2Waypoint(self.waypoints[2]),
                             transitions={'succeeded':'NAV_S2B_C',
                                          'aborted':'aborted'})
            StateMachine.add('NAV_S2B_C', Nav2Waypoint(self.waypoints[8]),
                             transitions={'succeeded':'NAV_S2B_B',
                                          'aborted':'aborted'})
            StateMachine.add('NAV_S2B_B', Nav2Waypoint(self.waypoints[10]),
                             transitions={'succeeded':'NAV_S2B_END',
                                          'aborted':'aborted'})
            StateMachine.add('NAV_S2B_END', Nav2Waypoint(self.waypoints[11]),
                             transitions={'succeeded':'IN_BEDROOM',
                                          'aborted':'aborted'})

            StateMachine.add('NAV_S2K_M', Nav2Waypoint(self.waypoints[2]),
                             transitions={'succeeded':'NAV_S2K_K',
                                          'aborted':'aborted'})
            StateMachine.add('NAV_S2K_K', Nav2Waypoint(self.waypoints[5]),
                             transitions={'succeeded':'NAV_S2K_END',
                                          'aborted':'aborted'})
            StateMachine.add('NAV_S2K_END', Nav2Waypoint(self.waypoints[6]),
                             transitions={'succeeded':'IN_KITCHEN',
                                          'aborted':'aborted'})





        # Create and start the SMACH introspection server
        intro_server = IntrospectionServer('carry_sm', self.sm_top, '/SM_CARRY_ROOT')
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
    def in_main_room_child_termination_cb(self, outcome_map):
	#rospy.loginfo("daymode_child_termination_cb.")
	return True


    # Gets called when ALL child states are terminated
    def in_main_room_outcome_cb(self, outcome_map):
        #rospy.loginfo("daymode_outcome_cb.")
        if outcome_map['GO_TO_KITCHEN'] == 'invalid':
            rospy.loginfo("Going to the kitchen ")
            return 'go_kitchen'
        if outcome_map['GO_TO_BEDROOM'] == 'invalid':
            rospy.loginfo("Going to the bedroom")
            return 'go_bedroom'
        if outcome_map['GO_TO_SLEEP'] == 'invalid':
            rospy.loginfo("Going to sleep")
            return 'go_sleep'
        if outcome_map['GO_TO_POINT'] == 'succeeded':
            rospy.loginfo("Arrived to point")
            return 'succeeded'
        else:
            return 'aborted'


    # Gets called when ANY child state terminates
    def in_sleep_child_termination_cb(self, outcome_map):
        #rospy.loginfo("daymode_child_termination_cb.")
        return True


    # Gets called when ALL child states are terminated
    def in_sleep_outcome_cb(self, outcome_map):
        #rospy.loginfo("daymode_outcome_cb.")
        if outcome_map['GO_TO_KITCHEN'] == 'invalid':
            rospy.loginfo("Going to the kitchen ")
            return 'go_kitchen'
        if outcome_map['GO_TO_BEDROOM'] == 'invalid':
            rospy.loginfo("Going to the bedroom")
            return 'go_bedroom'
        if outcome_map['GO_TO_MAIN_ROOM'] == 'invalid':
            rospy.loginfo("Going to main room")
            return 'go_main_room'
        if outcome_map['GO_TO_POINT'] == 'succeeded':
            rospy.loginfo("Arrived to point")
            return 'succeeded'
        else:
            return 'aborted'


    # Gets called when ANY child state terminates
    def in_bedroom_child_termination_cb(self, outcome_map):
        #rospy.loginfo("daymode_child_termination_cb.")
        return True


    # Gets called when ALL child states are terminated
    def in_bedroom_outcome_cb(self, outcome_map):
        #rospy.loginfo("daymode_outcome_cb.")
        if outcome_map['GO_TO_KITCHEN'] == 'invalid':
            rospy.loginfo("Going to the kitchen ")
            return 'go_kitchen'
        if outcome_map['GO_TO_MAIN_ROOM'] == 'invalid':
            rospy.loginfo("Going to the main room")
            return 'go_main_room'
        if outcome_map['GO_TO_SLEEP'] == 'invalid':
            rospy.loginfo("Going to sleep")
            return 'go_sleep'
        if outcome_map['GO_TO_POINT'] == 'succeeded':
            rospy.loginfo("Arrived to point")
            return 'succeeded'
        else:
            return 'aborted'


    # Gets called when ANY child state terminates
    def in_kitchen_child_termination_cb(self, outcome_map):
        #rospy.loginfo("daymode_child_termination_cb.")
        return True


    # Gets called when ALL child states are terminated
    def in_kitchen_outcome_cb(self, outcome_map):
        #rospy.loginfo("daymode_outcome_cb.")
        if outcome_map['GO_TO_MAIN_ROOM'] == 'invalid':
            rospy.loginfo("Going to the main room")
            return 'go_main_room'
        if outcome_map['GO_TO_BEDROOM'] == 'invalid':
            rospy.loginfo("Going to the bedroom")
            return 'go_bedroom'
        if outcome_map['GO_TO_SLEEP'] == 'invalid':
            rospy.loginfo("Going to sleep")
            return 'go_sleep'
        if outcome_map['GO_TO_POINT'] == 'succeeded':
            rospy.loginfo("Arrived to point")
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

	

    def shutdown(self):
        rospy.loginfo("Stopping carry automation...")
        
        #self.sm_day_mode.request_preempt()
        
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        SMACHAI()
    except rospy.ROSInterruptException:
        rospy.loginfo("CARRY automation finished.")
