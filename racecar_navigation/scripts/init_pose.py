#!/usr/bin/env python
import rospy
import roslib
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
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
        rospy.init_node('initial_pos_pub', anonymous=True)
        rospy.loginfo("This node sets the robot's position to the red cross on the floor. It will shudown after publishing to the topic /initialpose")

        self.initpose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        self.aruco_init_pose_pub = rospy.Publisher('/aruco_init_pose', PoseWithCovarianceStamped, queue_size=10)
        self.reset_odom_pub = rospy.Publisher('/odom_combined_ekf', Odometry, queue_size=10)
        self.init_pose_finish_pub = rospy.Publisher('/aruco_init_pose_finish', Bool, queue_size=10)
        self.language_pub = rospy.Publisher('tts', String, queue_size=1)

        self.initpose_aruco_sub = rospy.Subscriber("/initpose_aruco", PoseWithCovarianceStamped, self.initPoseArucoMsgCallback)
        self.initpose_aruco = PoseWithCovarianceStamped()

        self.seq = 0
        self.count = 0
        self.NUM_USED_FOR_AVRG = 30
        self.MAX_SEQ_NEED = 2
        self.trans_total = [0, 0, 0]
        self.rot_total = [0, 0, 0, 0]
        self.trans_total_seq = [0, 0, 0]
        self.rot_total_seq = [0, 0, 0, 0]
        self.IF_GOT_INIT = False

        self.tf_map_to_odom_br_ = tf.TransformBroadcaster()

    def initPoseArucoMsgCallback(self, msg):
        initpose_aruco = msg

    def initial_pos_pub(self):

        # arucoCodeBroadcaster = TransformBroadcaster()
        # arucoCodeBroadcaster.sendTransform(
        #     (6.56082, 6.63258, 0.0),
        #     (-0.0635783, -0.704243, -0.704243, -0.0635782),
        #     rospy.Time.now(),
        #     "aruco_code",
        #     "map"
        # )

        # this two line is used to stop the while loop
        # when you using CTRL+C to exit the program
        signal.signal(signal.SIGINT, quit)
        signal.signal(signal.SIGTERM, quit)

        # listener.waitForTransform("/map", "/camera_init", rospy.Time(), rospy.Duration(4.0))
        # while not map_camera_trans:
        #     try:
        #         # rospy.sleep(1)
        #         # rospy.loginfo(map_camera_trans)
        #         time_now = rospy.Time.now()
        #         listener.waitForTransform("/map", "/camera_init", time_now,  rospy.Duration(4.0))
        #         # listener.waitForTransform("/map", "/camera_init", rospy.Time(0),  rospy.Duration(1.0))
        #         (map_camera_trans, map_camera_rot) = listener.lookupTransform('/map', '/camera_init', time_now)
        #         # (map_camera_trans, map_camera_rot) = listener.lookupTransform('/map', '/camera_init', rospy.Time(0))
        #     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #         # map_camera_trans = [0, 0, 0]
        #         # map_camera_rot = [0, 0, 0, 0]
        #         # traceback.print_exc()
        #         rospy.loginfo("stop!!!-----------")
        #         continue
        map_camera_trans = []
        listener = tf.TransformListener()
        rospy.sleep(0.1)
        # count = 0
        while not map_camera_trans:
            try:
                # rospy.loginfo(map_camera_trans)
                # time_now = rospy.Time.now()
                # listener.waitForTransform("/map", "/camera_init", time_now,  rospy.Duration(5.0))
                # listener.waitForTransform("/map", "/camera_init", rospy.Time(0),  rospy.Duration(4.0))
                # (map_camera_trans, map_camera_rot) = listener.lookupTransform('/map', '/camera_init', time_now)
                (map_camera_trans, map_camera_rot) = listener.lookupTransform('/map', '/camera_init', rospy.Time(0))
                rospy.loginfo("********Got Trans & Rot********")
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                # map_camera_trans = [0, 0, 0]
                # map_camera_rot = [0, 0, 0, 0]
                # traceback.print_exc()
                rospy.loginfo("********NO CODE --OR-- NO TF ********")
                rospy.sleep(0.1)
                # rospy.loginfo("stop!!!-----------")
                continue

        self.count = self.count + 1
        self.trans_total[0] = self.trans_total[0] + map_camera_trans[0] / self.NUM_USED_FOR_AVRG
        self.trans_total[1] = self.trans_total[1] + map_camera_trans[1] / self.NUM_USED_FOR_AVRG
        self.rot_total[0] = self.rot_total[0] + map_camera_rot[0] / self.NUM_USED_FOR_AVRG
        self.rot_total[1] = self.rot_total[1] + map_camera_rot[1] / self.NUM_USED_FOR_AVRG
        self.rot_total[2] = self.rot_total[2] + map_camera_rot[2] / self.NUM_USED_FOR_AVRG
        self.rot_total[3] = self.rot_total[3] + map_camera_rot[3] / self.NUM_USED_FOR_AVRG
        # rospy.loginfo(map_camera_trans[0])
        # rospy.loginfo(self.trans_total[0])

        if self.count >= self.NUM_USED_FOR_AVRG:
            rospy.loginfo(self.count)
            self.trans_total_seq[0] = self.trans_total_seq[0] + self.trans_total[0] / self.MAX_SEQ_NEED
            self.trans_total_seq[1] = self.trans_total_seq[1] + self.trans_total[1] / self.MAX_SEQ_NEED
            self.rot_total_seq[0] = self.rot_total_seq[0] + self.rot_total[0] / self.MAX_SEQ_NEED
            self.rot_total_seq[1] = self.rot_total_seq[1] + self.rot_total[1] / self.MAX_SEQ_NEED
            self.rot_total_seq[2] = self.rot_total_seq[2] + self.rot_total[2] / self.MAX_SEQ_NEED
            self.rot_total_seq[3] = self.rot_total_seq[3] + self.rot_total[3] / self.MAX_SEQ_NEED
            self.trans_total = [0, 0, 0]
            self.rot_total = [0, 0, 0, 0]
            self.seq = self.seq + 1
            self.count = 0

        if self.seq >= self.MAX_SEQ_NEED:

            # ---------------------------------------------------------------------
            # The following is going to update the /map->/odom TF frame transform
            # https://robot-ros.com/robot/35127.html

            # map_odom_tf = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(
            #     self.trans_total_seq), tf.transformations.quaternion_matrix(self.rot_total_seq))
            # inversed_map_odom_tf = tf.transformations.inverse_matrix(map_odom_tf)
            #
            # inversed_trans_map_odom = tf.transformations.translation_from_matrix(inversed_map_odom_tf)
            # inversed_rot_map_odom = tf.transformations.quaternion_from_matrix(inversed_map_odom_tf)

            # self.tf_map_to_odom_br_.sendTransform(
            #     self.trans_total_seq,
            #     self.rot_total_seq,
            #     # inversed_trans_map_odom,
            #     # inversed_rot_map_odom,
            #     rospy.Time.now(),
            #     "map",
            #     "odom"
            # )
            # rospy.loginfo("Published /map->/odom TF frame transform!")
            # rospy.sleep(5)
            # ---------------------------------------------------------------------

            start_pos = PoseWithCovarianceStamped()
            # start_pos = initpose_aruco
            # filling header with relevant information
            start_pos.header.frame_id = "map"
            start_pos.header.seq = self.seq

            # start_pos.header.stamp = rospy.Time.now()
            # filling payload with relevant information gathered from subscribing
            # to initialpose topic published by RVIZ via rostopic echo initialpose
            start_pos.pose.pose.position.x = self.trans_total_seq[0]
            # rospy.loginfo(self.trans_total[0])
            start_pos.pose.pose.position.y = self.trans_total_seq[1]
            start_pos.pose.pose.position.z = 0.0
            start_pos.pose.pose.orientation.x = self.rot_total_seq[0]
            start_pos.pose.pose.orientation.y = self.rot_total_seq[1]
            start_pos.pose.pose.orientation.z = self.rot_total_seq[2]
            start_pos.pose.pose.orientation.w = self.rot_total_seq[3]
            # start_pos.pose.pose.position.x=map_camera_trans[0]
            # start_pos.pose.pose.position.y=map_camera_trans[1]
            # start_pos.pose.pose.position.z=0.0
            # start_pos.pose.pose.orientation.x=map_camera_rot[0]
            # start_pos.pose.pose.orientation.y=map_camera_rot[1]
            # start_pos.pose.pose.orientation.z=map_camera_rot[2]
            # start_pos.pose.pose.orientation.w=map_camera_rot[3]
            # start_pos.pose.pose.position.x = 5.0
            # start_pos.pose.pose.position.y = 10.0
            # start_pos.pose.pose.position.z = 0.0
            # start_pos.pose.pose.orientation.x = 0.0
            # start_pos.pose.pose.orientation.y = 0.0
            # start_pos.pose.pose.orientation.z = -0.694837665627
            # start_pos.pose.pose.orientation.w = 0.719166613815
            # start_pos.pose.covariance[0] = 0.01
            # start_pos.pose.covariance[7] = 0.01
            # start_pos.pose.covariance[1:7] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            # start_pos.pose.covariance[8:34] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            #                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            # start_pos.pose.covariance[35] = 0.001

            # rospy.loginfo(start_pos)
            self.initpose_pub.publish(start_pos)
            rospy.sleep(1)
            self.initpose_pub.publish(start_pos)
            # self.trans_total=[0,0,0]
            # self.rot_total=[0,0,0,0]
            self.IF_GOT_INIT = True

        # rospy.sleep(2)
        # self.initpose_pub.publish(start_pos)

        # reset_odom = Odometry()
        # reset_odom.header.stamp = rospy.Time.now()
        # reset_odom.header.frame_id = "odom"
        # reset_odom.child_frame_id = "base_link"
        # self.reset_odom_pub.publish(reset_odom)

    def shutdown(self):
        rospy.loginfo("Stopping initial_pos_pub...")

    def quit(signum, frame):
        print ''
        print 'stop program'
        sys.exit()


if __name__ == '__main__':
    try:
        nuc_init = NucInitPose()
        loop_r = rospy.Rate(10)  # 10hz

        init_pose_finish = False

        # while not rospy.is_shutdown():
        while (not nuc_init.IF_GOT_INIT) | (nuc_init.seq < nuc_init.MAX_SEQ_NEED):
            nuc_init.initial_pos_pub()
            loop_r.sleep()

        init_pose_finish = True
        nuc_init.init_pose_finish_pub.publish(init_pose_finish)
        rospy.sleep(1)
        # nuc_init.init_pose_finish_pub.publish(init_pose_finish)
        nuc_init.language_pub.publish("L'inizializzazione e fatta. Si prega di impostare la destinazione.")

    except rospy.ROSInterruptException:
        pass
