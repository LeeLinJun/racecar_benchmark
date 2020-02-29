#!/usr/bin/env python
import rospy
import roslib
import actionlib
from actionlib_msgs.msg import GoalID, GoalStatus, GoalStatusArray
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PoseWithCovarianceStamped, PoseStamped, TransformStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf.broadcaster import TransformBroadcaster
import tf
import tf2_ros
import sys
import signal
from std_msgs.msg import Bool
from std_msgs.msg import String
import rospkg
import numpy as np
import math

class NucInitPose():
    def __init__(self):
        self.rospack = rospkg.RosPack()
        self.data_path = self.rospack.get_path('nuc_navigation')+'/data/'
        self.left_states = np.loadtxt(self.data_path+'LEFT.csv', delimiter=',')
        self.right_states = np.loadtxt(self.data_path+'RIGHT.csv', delimiter=',')
        
        rospy.on_shutdown(self.shutdown)
        rospy.init_node('initial_loca_pub', anonymous=True)
        rospy.loginfo("This node sets the robot's position to the red cross on the floor. It will shudown after publishing to the topic /initialpose")

        self.initpose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.OdometryCallback)

        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(60))    # Wait 60 seconds for the action server to become available
        rospy.loginfo("Connected to move base server")
        # clear costmap
        rospy.wait_for_service('/move_base/clear_costmaps')
        self.clear_costmaps_serv = rospy.ServiceProxy('move_base/clear_costmaps', Empty)

        # Goal state return values
        self.goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED', 'SUCCEEDED', 'ABORTED', 'REJECTED', 'PREEMPTING', 'RECALLING', 'RECALLED', 'LOST']
        self.current_state = 'PENDING'

    def OdometryCallback(self, odom_receive):
            self.robot_pose_now = odom_receive.pose.pose
            # rospy.loginfo(self.robot_pose_now.position.x)

    def set_init(self, left_id=0, right_id=0, left_to_right=True):
        if left_to_right:
            start = self.left_states[left_id]
            goal = self.right_states[right_id]
        else:
            start = self.right_states[right_id]
            goal = self.left_states[left_id]
        self.init_locations = dict()
        self.init_locations['INIT_L1'] = Pose(Point(start[0], start[1], start[2]), Quaternion(start[3], start[4], start[5], start[6]))
        self.goal_locations = dict()
        self.goal_locations['GOAL_L1'] = Pose(Point(goal[0], goal[1], goal[2]), Quaternion(goal[3], goal[4], goal[5], goal[6]))

        self.robot_pose_now = Pose()
        self.robot_pose_last = Pose()

    def initial_loca_pub(self, start_id):
        start_pos = PoseWithCovarianceStamped()
        start_pos.header.frame_id = "map"
        start_pos.pose.pose = self.init_locations[start_id]
        self.initpose_pub.publish(start_pos)
        rospy.sleep(1)
        self.initpose_pub.publish(start_pos)


    def goal_loca_pub(self, goal_id):
        self.clear_costmaps_serv()
        rospy.sleep(1)
       
        self.goal = MoveBaseGoal()
        self.goal.target_pose.pose = self.goal_locations[goal_id]
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.header.stamp = rospy.Time.now()
        rospy.loginfo("Going to the goal ...")
        self.move_base.send_goal(self.goal)

    def shutdown(self):
        rospy.loginfo("Stopping initial_loca_pub...")

    def quit(signum, frame):
        print ''
        print 'stop program'
        sys.exit()


if __name__ == '__main__':
    goroom = NucInitPose()
    NUM_EXP = 3
    NUM_REPEAT = 3
    data = np.zeros((NUM_EXP, NUM_REPEAT, 3))
    try:        
        loop_r = rospy.Rate(10)  # 10hz
        for i in range(NUM_EXP):
            for j in range(NUM_REPEAT):
                goroom.set_init(i, i, True)
                total_path_length = 0
                success_flag = 0

                start_id = 'INIT_L1'
                goal_id  = 'GOAL_L1'
                goroom.initial_loca_pub(start_id)
                rospy.sleep(2.0)
                goroom.goal_loca_pub(goal_id)
                START_TIME = rospy.Time.now()
                END_TIME = rospy.Time.now()

                while (not goroom.current_state == GoalStatus.SUCCEEDED) and (END_TIME-START_TIME).secs <= 60:
                    signal.signal(signal.SIGINT, quit)
                    signal.signal(signal.SIGTERM, quit)
                    goroom.current_state = goroom.move_base.get_state()
                    robot_states_now = str(goroom.goal_states[goroom.current_state])
                    # rospy.loginfo("Robot Current State : " + robot_states_now)
                    # rospy.loginfo('now  - ' + str(goroom.robot_pose_now.position.x))
                    # rospy.loginfo('last - ' + str(goroom.robot_pose_last.position.x))
                    delta_x = goroom.robot_pose_now.position.x - goroom.robot_pose_last.position.x
                    delta_y = goroom.robot_pose_now.position.y - goroom.robot_pose_last.position.y
                    delta_z = goroom.robot_pose_now.position.z - goroom.robot_pose_last.position.z
                    total_path_length = total_path_length + math.sqrt(math.pow(delta_x, 2) + math.pow(delta_y, 2) + math.pow(delta_z, 2))
                    goroom.robot_pose_last = goroom.robot_pose_now
                    END_TIME = rospy.Time.now()    
                if not goroom.current_state == GoalStatus.SUCCEEDED and (END_TIME-START_TIME).secs > 60:
                    success_flag = 0
                else:
                    success_flag = 1
                loop_r.sleep()
                data[i, j, 0] = (END_TIME-START_TIME).secs
                data[i, j, 1] = total_path_length
                data[i, j, 2] = success_flag

                rospy.loginfo("Totally time : {}\n Trajectory length: {}\n If Success:{}\n".format((END_TIME-START_TIME).secs,total_path_length, success_flag))

                goroom.current_state = 'pending'


        # loop_r.sleep()

        print(data)
    except rospy.ROSInterruptException:
        pass
