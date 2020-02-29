#!/usr/bin/env python
''' nav_yolo.py

	version : demo before Fei leave
'''

import rospy
import actionlib
from actionlib_msgs.msg import GoalID, GoalStatus, GoalStatusArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PoseWithCovarianceStamped, PoseStamped, TransformStamped
from std_msgs.msg import String
from std_srvs.srv import Empty
from nav_msgs.msg import Path
# from goroom_test.srv import GuiGoalService, GoalStateService
import random
import numpy
import tf
import signal


class NavTest():
    def __init__(self):

        rospy.init_node('nav_test', anonymous=True)
        rospy.on_shutdown(self.shutdown)

        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base.wait_for_server(rospy.Duration(60))    # Wait 60 seconds for the action server to become available
        rospy.loginfo("Connected to move base server")

        # clear costmap
        rospy.wait_for_service('/move_base/clear_costmaps')
        self.clear_costmaps_serv = rospy.ServiceProxy('move_base/clear_costmaps', Empty)

        # ------------- GUI Services -------------
        self.gui_goal_serv = rospy.Subscriber('goal_from_gui', String, self.GuiGoalReceviedCallback)
        # self.goal_state_serv = rospy.Service('goal_state_service', GoalStateService, self.goalStateServiceCallback)

        # Publisher to manually control the robot (e.g. to stop it, queue_size=5)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux', Twist, queue_size=1)
        self.robot_states_pub = rospy.Publisher('robot_states', String, queue_size=1)
        self.language_pub = rospy.Publisher('tts', String, queue_size=1)
        # self.goal_states_pub = rospy.Publisher('goal_states', String, queue_size=1)
        # self.goal_received_pub = rospy.Publisher('goal_received', String, queue_size=1)
        # self.localplan_sub = rospy.Subscriber("move_base/DWAPlannerROS/local_plan", Path, self.localPlanCallback)

        self.cuboid_deteted_sub = rospy.Subscriber("cuboid_detected", PoseStamped, self.cuboidDetectedCallback)
        self.if_cuboid_deteted_sub = rospy.Subscriber("if_cuboid_detected", String, self.ifcuboidDetectedCallback)
        # self.gui_received_arrived_sub = rospy.Subscriber("arrived", String, self.guiReceivedArrivedCallback)

        rospy.loginfo("Starting navigation ....")
        # rospy.loginfo("*** Click the 2D Pose Estimate button in RViz to set the robot's initial pose...")
        # rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)

        # Define language
        self.lang_room_arrived = 'Sei arrivato alla destinazione.'
        self.lang_reception_arrived = 'Tornato alla posizione iniziale.'
        self.lang_introduce = 'Buongiorno, sono un robot nato a Pisa.'

        # Goal state return values
        self.goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED', 'SUCCEEDED', 'ABORTED', 'REJECTED', 'PREEMPTING', 'RECALLING', 'RECALLED', 'LOST']
        self.goal_current = ''
        self.goal_received = False
        self.go_surv_received = False

        self.if_cuboid_detection = False
        self.if_cuboid_from_yolo = False
        self.cuboid_location = []

        # self.goal_arrived = False
        # self.guiReceivedArrived = False

        # Set up the goal locations. Poses are defined in the map frame.
        self.locations = dict()
        self.locations['ROOM_LAB'] = Pose(Point(10.8466, 7.55557, 0), Quaternion(0.0, 0.0, 0.757342, -0.653018))
        self.locations['EXIT'] = Pose(Point(14.8724, 7.96083, 0), Quaternion(0, 0, 0.664482, 0.747305))
        self.locations['WORKSHOPS'] = Pose(Point(27.3687, 5.81269, 0), Quaternion(0.0, 0.0, 0.654586, 0.755988))
        self.locations['COFFEE'] = Pose(Point(22.1985, 6.74462, 0), Quaternion(0.0, 0.0, -0.748319, 0.66334))
        self.locations['ARUCO'] = Pose(Point(6.95152, 8.59522, 0), Quaternion(0, 0, -0.761218, 0.648495))
        self.locations['HOME'] = Pose(Point(0, 0, 0), Quaternion(0.000, 0.000, 0.000, 1.000))
        # self.locations['P0'] = Pose(Point(2.21285, -0.469732, 0), Quaternion(0, 0, -0.99033, 0.138728))
        # self.locations['P1'] = Pose(Point(6.56309, -1.20057, 0), Quaternion(0, 0, 0.0487893, 0.998809))
        # self.locations['P2'] = Pose(Point(10.38, 1.07452, 0), Quaternion(0, 0, -0.0350146, 0.999387))
        # self.locations['P3'] = Pose(Point(15.5545, -0.113971, 0), Quaternion(0, 0, 0.093234, 0.995644))
        # self.locations['P4'] = Pose(Point(26.7169, 0.625837, 0), Quaternion(0, 0, -0.220397, 0.97541))
        # self.locations['P5'] = Pose(Point(32.3373, 1.91218, 0), Quaternion(0, 0, 0.268397, 0.963308))
        # self.locations['P6'] = Pose(Point(34.4508, -6.39862, 0), Quaternion(0, 0, 0.778162, 0.628063))
        # self.locations['P7'] = Pose(Point(37.9245, 0.566548, 0), Quaternion(0, 0, 0.944841, 0.327529))
        # self.locations['P8'] = Pose(Point(40.6931, 8.55877, 0), Quaternion(0, 0, 0.999461, 0.0328223))

    def goalMsgCallback(self, data):
        rospy.loginfo("Recevied goal is [%s]", data.data)
        self.moveToGoal(data.data)

    def cuboidDetectedCallback(self, data):
        self.cuboid_location = data
        self.if_cuboid_detection = True

    def ifcuboidDetectedCallback(self, data):
        if data.data == 'True':
            self.if_cuboid_from_yolo = True
        else:
            self.if_cuboid_from_yolo = False

    def localPlanCallback(self, msg):
        if not msg.poses:
            rospy.loginfo("EMPTY")

    def GuiGoalReceviedCallback(self, goal_gui):
        rospy.loginfo("GUI : set to new goal [%s]", goal_gui.data)

        if goal_gui.data == 'SURVEILLIANCE':
            # self.go_surv_random()
            self.go_surv_received = True
        else:
            self.goal_current = goal_gui.data
            self.go_surv_received = False
            self.moveToGoal(goal_gui.data)

        return True

    def moveToGoal(self, goal_gui):
        # clear local costmap before move to a new goal
        self.clear_costmaps_serv()
        rospy.sleep(1)

        if self.goal_received:
            self.move_base.cancel_goal()
            rospy.loginfo("Stopping last goal")
            # rospy.sleep(1)

            # Set up the goal location
        self.goal = MoveBaseGoal()
        self.goal.target_pose.pose = self.locations[goal_gui]
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.header.stamp = rospy.Time.now()
        rospy.loginfo("Going to the goal ...")
        # Start the robot toward the next location
        self.move_base.send_goal(self.goal)
        # rospy.sleep(3)
        self.goal_received = True
        # self.goal_arrived = False  # reset the self.goal_arrived to FALSE
        # self.guiReceivedArrived = False

    def go_surv_random(self):
        self.move_base.cancel_goal()
        rospy.loginfo("Stopping last goal")
        # rospy.sleep(1)

        # clear local costmap before move to a new goal
        surv_point_num = random.randint(1, 4)
        # goal_num = {
        #     1: "P0",
        #     2: "P1",
        #     3: "P2",
        #     4: "P3",
        #     5: "P4",
        #     6: "P5",
        #     7: "P6",
        #     8: "P7",
        #     9: "P8",
        # }
        goal_num = {
            1: "ROOM_LAB",
            2: "EXIT",
        # 3: "WORKSHOPS",
            3: "COFFEE",
            4: "ARUCO",
        }
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose = self.locations[goal_num.get(surv_point_num)]

        # Start the robot toward the next location
        self.clear_costmaps_serv()
        rospy.sleep(1)
        self.move_base.send_goal(self.goal)
        rospy.loginfo("Going to the next surveillance goal no. %s", goal_num.get(surv_point_num))

    def go_to_cuboid(self):
        self.move_base.cancel_goal()
        rospy.loginfo("Stopping last goal")
     

        signal.signal(signal.SIGINT, quit)
        signal.signal(signal.SIGTERM, quit)
        # map_cuboid_trans = []
        # map_cuboid_rot = []

        self.language_pub.publish("found a bottle.")

        cuboid_in_map = []
        rospy.loginfo("header ID : " + self.cuboid_location.header.frame_id)

        listener = tf.TransformListener()
        rospy.sleep(0.1)

        time_begin = rospy.Time.now().secs
        # while not map_cuboid_trans:
        while not cuboid_in_map:
            time_duration = rospy.Time.now().secs - time_begin

            if time_duration < 10:
                try:
                    cuboid_in_map = listener.transformPose('/map', self.cuboid_location)
                    # (map_cuboid_trans, map_cuboid_rot) = listener.lookupTransform('/wrldcuboid', '/cuboid', rospy.Time(0))
                    # (map_cuboid_trans, map_cuboid_rot) = listener.lookupTransform('/map', '/wrldcuboid', rospy.Time(0))
                    rospy.loginfo("******** TF: Got CUBOID in MAP ********")
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rospy.loginfo("******** TF: No GET CUBOID in MAP ********")
                    rospy.sleep(0.1)
                    continue
            else:
                break

        if cuboid_in_map:
            self.goal = MoveBaseGoal()
            self.goal.target_pose.header.frame_id = 'map'
            self.goal.target_pose.header.stamp = rospy.Time.now()

            self.goal.target_pose.pose = cuboid_in_map.pose
            rospy.loginfo(self.goal.target_pose.pose)

            # Start the robot toward the cuboid
            self.clear_costmaps_serv()
            rospy.sleep(1)
            self.move_base.send_goal(self.goal)
            rospy.loginfo("Going to the cuboid")
            rospy.sleep(1)
            self.language_pub.publish("il robot va verso la bottiglia.")
        else:
            self.if_cuboid_detection = False
            self.count_goal_yolo = 0
            rospy.sleep(1)
            self.language_pub.publish("give up, resuming surveillance.")
            goroom.count_goal_surv == 0

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        # Cancel any active goals
        self.move_base.cancel_goal()
        rospy.sleep(3)
        # Stop the robot
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(2)

    def quit(signum, frame):
        print ''
        print 'stop program'
        sys.exit()


if __name__ == '__main__':
    try:
        goroom = NavTest()

        loop_r = rospy.Rate(1)    # 10hz
        goroom.count_goal_surv = 0
        goroom.count_goal_yolo = 0
        rospy.sleep(1)
        goroom.language_pub.publish("il robot e pronto per partire.")
        while not rospy.is_shutdown():
            if goroom.go_surv_received:
                if goroom.if_cuboid_detection:
                    if goroom.count_goal_yolo == 0:
                        goroom.go_to_cuboid()
                        goroom.count_goal_yolo = goroom.count_goal_yolo + 1
                    goroom.state = goroom.move_base.get_state()
                    robot_states_now = str(goroom.goal_states[goroom.state])
                    if goroom.state == GoalStatus.SUCCEEDED:
                        goroom.if_cuboid_detection = False
                        goroom.count_goal_yolo = 0
                        rospy.loginfo("Reached YOLO object position!")
                        rospy.sleep(1)
                        goroom.language_pub.publish("La bottiglia e stata raggiunta.")
                        # goroom.go_surv_random()

                else:
                    if goroom.count_goal_surv == 0:
                        goroom.go_surv_random()
                        goroom.count_goal_surv = goroom.count_goal_surv + 1
                    goroom.state = goroom.move_base.get_state()
                    robot_states_now = str(goroom.goal_states[goroom.state])
                    if goroom.state == GoalStatus.SUCCEEDED:
                        goroom.count_goal_surv = 0
                        rospy.loginfo("Current surveillance goal succeeded!")
                        rospy.sleep(1)
                        goroom.language_pub.publish("Current surveillance goal succeeded!")
                        # goroom.go_surv_random()
            else:
                # Check for success or failure of CHEKIN Type of Goal
                if goroom.goal_received:
                    goroom.state = goroom.move_base.get_state()
                    robot_states_now = str(goroom.goal_states[goroom.state])
                    goroom.robot_states_pub.publish(robot_states_now)

                    if goroom.state == GoalStatus.SUCCEEDED:
                        rospy.loginfo("Goal succeeded!")
                        if (not goroom.goal_current == 'HOME'):
                            goroom.language_pub.publish(goroom.lang_room_arrived)
                        else:
                            goroom.language_pub.publish(goroom.lang_reception_arrived)

                        goroom.goal_arrived = True
                        goroom.goal_received = False
                    else:
                        rospy.loginfo("Robot State : " + robot_states_now)
                else:
                    goroom.robot_states_pub.publish('WAIT_FOR_GOAL')
                    rospy.loginfo("Waiting for new Check IN Goal!")

            #
            # if (not goroom.goal_received) and (goroom.goal_arrived):
            # if (not goroom.guiReceivedArrived) and (goroom.goal_arrived):
            #     goroom.goal_states_pub.publish("SUCCEEDED")
            loop_r.sleep()

# rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("navigation test finished.")
