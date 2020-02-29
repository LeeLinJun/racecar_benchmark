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


class NucInitPose():
    def __init__(self):
        rospy.on_shutdown(self.shutdown)
        rospy.init_node('initial_loca_pub', anonymous=True)
        rospy.loginfo("This node sets the robot's position to the red cross on the floor. It will shudown after publishing to the topic /initialpose")

        self.initpose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(60))    # Wait 60 seconds for the action server to become available
        rospy.loginfo("Connected to move base server")
        # clear costmap
        rospy.wait_for_service('/move_base/clear_costmaps')
        self.clear_costmaps_serv = rospy.ServiceProxy('move_base/clear_costmaps', Empty)

        # Goal state return values
        self.goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED', 'SUCCEEDED', 'ABORTED', 'REJECTED', 'PREEMPTING', 'RECALLING', 'RECALLED', 'LOST']
        self.current_state = 'PENDING'

        self.init_locations = dict()
        self.init_locations['INIT_L1'] = Pose(Point(-0.653374195099, -0.895349740982, 0), Quaternion(0, 0, 0.135303399881, 0.990804213748))
        self.init_locations['INIT_L2'] = Pose(Point(-0.557947158813, 0.205744981766, 0.0 ), Quaternion(0.0, 0.0, 0.394188499164, 0.91902961167))
        self.init_locations['INIT_L3'] = Pose(Point(0.228755950928, -0.275204181671, 0.0), Quaternion(0.0, 0.0, -0.256127288561, 0.966643063418))

        # different start area
        self.init_locations['INIT_L4'] = Pose(Point(0.497625112534, 54179096222, 0.0), Quaternion(0, 0, 0.515876621565, 0.856662892463))
        self.init_locations['INIT_L5'] = Pose(Point(1.65303754807, 8.03802108765, 0), Quaternion(0.000, 0.000, -0.791846722288, 0.610719877195))
        self.init_locations['INIT_L6'] = Pose(Point(1.00688910484, 7.61973905563, 0), Quaternion(0.000, 0.000, -0.322892729631, 0.94643556841))


        self.goal_locations = dict()
        self.goal_locations['GOAL_L1'] = Pose(Point(1.26254987717, 6.82018184662, 0.0), Quaternion(0.0, 0.0, 0.737921745822, 0.674886284527))
        self.goal_locations['GOAL_L2'] = Pose(Point(0.223132610321, -0.222714424133, 0.0), Quaternion(0.0, 0.0, 0.995901562483, 0.0904437827806))
        self.goal_locations['GOAL_L3'] = Pose(Point(3.13682126999, 2.86037015915, 0.0), Quaternion(0.0, 0.0, 0.450237335203, 0.892908921441))
        self.goal_locations['GOAL_L4'] = Pose(Point(3.21567869186, 0.33133554459, 0.0), Quaternion(0.0, 0.0, 0.69441072153, 0.719578869773))



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
    try:
        goroom = NucInitPose()
        # loop_r = rospy.Rate(10)  # 10hz

        start_id = 'INIT_L1'
        goal_id  = 'GOAL_L1'

        goroom.initial_loca_pub(start_id)
        rospy.sleep(2.0)

        goroom.goal_loca_pub(goal_id)
        START_TIME = rospy.Time.now()

        while (not goroom.current_state == GoalStatus.SUCCEEDED):
            signal.signal(signal.SIGINT, quit)
            signal.signal(signal.SIGTERM, quit)
            goroom.current_state = goroom.move_base.get_state()
            robot_states_now = str(goroom.goal_states[goroom.current_state])
            rospy.loginfo("Robot Current State : " + robot_states_now)

        END_TIME = rospy.Time.now()
        rospy.loginfo('Totally time : '  + str((END_TIME-START_TIME).secs))
            

        # loop_r.sleep()


    except rospy.ROSInterruptException:
        pass
