#!/usr/bin/env python

import threading
import rospy
import actionlib
import numpy as np
from tf.transformations import quaternion_from_euler

from smach import State,StateMachine
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, Header
from sensor_msgs.msg import NavSatStatus, NavSatFix


waypoints = []
utm_init = False
gps_waypoints = np.array([[-84.40154,33.78081, 288.5, 270], [-84.40132, 33.78081, 288.5, 180], \
                     [-84.40132,33.78075, 288.5, 90], [-84.40150, 33.78075, 288.5, 90]], dtype=float)
deg2rad = 22*180/7

class FollowPath(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'], input_keys=['waypoints'])
        self.frame_id = rospy.get_param('~goal_frame_id','odom')
        # Get a move_base action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo('Connecting to move_base...')
        self.client.wait_for_server()
        rospy.loginfo('Connected to move_base.')

    def execute(self, userdata):
        global waypoints
        # Execute waypoints each in sequence
        for waypoint in waypoints:
            # Break if preempted
            if waypoints == []:
                rospy.loginfo('The waypoint queue has been reset.')
                break
            # Otherwise publish next waypoint as goal
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = self.frame_id
            goal.target_pose.pose.position = waypoint.pose.pose.position
            goal.target_pose.pose.orientation = waypoint.pose.pose.orientation
            rospy.loginfo('Executing move_base goal to position (x,y): %s, %s' %
                    (waypoint.pose.pose.position.x, waypoint.pose.pose.position.y))
            rospy.loginfo("To cancel the goal: 'rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}'")
            self.client.send_goal(goal)
            self.client.wait_for_result()
        return 'success'

def convert_PoseWithCovArray_to_PoseArray(waypoints):
    """Used to publish waypoints as pose array so that you can see them in rviz, etc."""
    poses = PoseArray()
    poses.header.frame_id = 'map'
    poses.poses = [pose.pose.pose for pose in waypoints]
    return poses

class GetPath(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'], input_keys=['waypoints'], output_keys=['waypoints'])
        # Create publsher to publish waypoints as pose array so that you can see them in rviz, etc.
        self.poseArray_publisher = rospy.Publisher('/waypoints', PoseArray, queue_size=1)

        # Start thread to listen for reset messages to clear the waypoint queue
        def wait_for_path_reset():
            """thread worker function"""
            global waypoints
            while not rospy.is_shutdown():
                data = rospy.wait_for_message('/path_reset', Empty)
                rospy.loginfo('Recieved path RESET message')
                self.initialize_path_queue()
                rospy.sleep(3) # Wait 3 seconds because `rostopic echo` latches
                               # for three seconds and wait_for_message() in a
                               # loop will see it again.
        reset_thread = threading.Thread(target=wait_for_path_reset)
        reset_thread.start()

    def initialize_path_queue(self):
        global waypoints
        waypoints = [] # the waypoint queue
        # publish empty waypoint queue as pose array so that you can see them the change in rviz, etc.
        self.poseArray_publisher.publish(convert_PoseWithCovArray_to_PoseArray(waypoints))

    def execute(self, userdata):
        global waypoints
        self.initialize_path_queue()
        self.path_ready = False

        # Start thread to listen for when the path is ready (this function will end then)
        def wait_for_path_ready():
            """thread worker function"""
            data = rospy.wait_for_message('/path_ready', Empty)
            rospy.loginfo('Recieved path READY message')
            self.path_ready = True
        ready_thread = threading.Thread(target=wait_for_path_ready)
        ready_thread.start()

        topic = "/initialpose"
        rospy.loginfo("Waiting to recieve waypoints via Pose msg on topic %s" % topic)
        rospy.loginfo("To start following waypoints: 'rostopic pub /path_ready std_msgs/Empty -1'")

        # Wait for published waypoints
        while not self.path_ready:
            try:
                pose = rospy.wait_for_message(topic, PoseWithCovarianceStamped, timeout=1)
            except rospy.ROSException as e:
                if 'timeout exceeded' in e.message:
                    continue  # no new waypoint within timeout, looping...
                else:
                    raise e
            rospy.loginfo("Recieved new waypoint")
            waypoints.append(pose)
            # publish waypoint queue as pose array so that you can see them in rviz, etc.
            self.poseArray_publisher.publish(convert_PoseWithCovArray_to_PoseArray(waypoints))

        # Path is ready! return success and move on to the next state (FOLLOW_PATH)
        return 'success'

class PathComplete(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

    def execute(self, userdata):
        rospy.loginfo('###############################')
        rospy.loginfo('##### REACHED FINISH GATE #####')
        rospy.loginfo('###############################')
        return 'success'

def utm_cb(msgIn):
    global wp_num,deg2rad        
    # recieve waypoint converted to odom
    rospy.loginfo("converted waypoint recieved.")
    posCovStamped = PoseWithCovarianceStamped()
    posCovStamped.pose = msgIn.pose
    posCovStamped.header = msgIn.header

    # add in des orientation
    q = quaternion_from_euler(0,0,gps_waypoints[wp_num][3]*deg2rad)
    posCovStamped.pose.pose.orientation.x = q[0]
    posCovStamped.pose.pose.orientation.y = q[1]
    posCovStamped.pose.pose.orientation.z = q[2]
    posCovStamped.pose.pose.orientation.w = q[3]

    # append to global waypoint array
    waypoints.append(posCovStamped)

def init_path():
    # logitude, latitute, altitute, orientation (+deg)
    gpsPub = rospy.Publisher("/fix_path_init", NavSatFix, queue_size=10)
    wpSub = rospy.Subscriber("/gps_path_init",Odometry, utm_cb)
    navStatus = NavSatStatus()
    navStatus.service = 1
    navStatus.status = 1

    # waypoints hardcoded for now, may want to spec with argument in future
    
    num_wp = gps_waypoints.shape[0]
    
    hdop = 1.5
    position_covariance_type = 1

    global wp_num

    for i in range(0,num_wp):
        # send gps waypoint for conversion
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'base_link'
        
        msgOut = NavSatFix()
        msgOut.position_covariance[0] = hdop**2
        msgOut.position_covariance[4] = hdop**2
        msgOut.position_covariance[8] = (2*hdop)**2
        msgOut.longitude = gps_waypoints[i][0]
        msgOut.latitude = gps_waypoints[i][1]
        msgOut.altitude = gps_waypoints[i][2]
        msgOut.position_covariance_type = position_covariance_type
        msgOut.status = navStatus
        msgOut.header = header
        wp_num = i
        gpsPub.publish(msgOut)

def main():
    rospy.init_node('follow_waypoints')
    init_path()
    sm = StateMachine(outcomes=['success'])

    with sm:
        # Not using this for now. 
        #StateMachine.add('GET_PATH', GetPath(),
        #                   transitions={'success':'FOLLOW_PATH'},
        #                   remapping={'waypoints':'waypoints'})
        StateMachine.add('FOLLOW_PATH', FollowPath(),
                           transitions={'success':'PATH_COMPLETE'},
                           remapping={'waypoints':'waypoints'})
        StateMachine.add('PATH_COMPLETE', PathComplete(),
                           transitions={'success':'GET_PATH'})

    outcome = sm.execute()
