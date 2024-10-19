import asyncio
import logging
import json
import numpy as np
from scipy.spatial.transform import Rotation as R

from go2_webrtc_driver.webrtc_driver import Go2WebRTCConnection, WebRTCConnectionMethod
from go2_webrtc_driver.constants import RTC_TOPIC, SPORT_CMD

import threading
import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
import sys
import signal
# Enable logging for debugging
logging.basicConfig(level=logging.FATAL)

# goal_position = [0.0, 0.0, 0.0]



class go2_base:
    def __init__(self):
        self.odom_pub = rospy.Publisher('/go2_odom', Odometry, queue_size=10)
        rospy.Subscriber('navigation/waypoints', PoseStamped, self.waypoint_callback)
        self.current_state = None
        self.goal_pos = [0.0, 0.0, 0.0] # x, y, yaw
        self.reach_tolerance = 0.2
        self.new_waypoint_received = False

        # Connect to the WebRTC service
        self.conn = Go2WebRTCConnection(WebRTCConnectionMethod.Remote, serialNumber="B42D4000O1GBDIG2", username="fy.ya20026@gmail.com", password="Ykk1234?")
        self.conn.connect()
        self.conn.datachannel.pub_sub.subscribe(RTC_TOPIC['LF_SPORT_MOD_STATE'], self.sportmodestatus_callback)

        # initialize motion
        self.init_motion()

        # start odom thread
        self.odom_thread = threading.Thread(target=self.publish_odometry)
        self.odom_thread.daemon = True  # Daemon thread to exit when the main program exits
        self.odom_thread.start()

        # move server

    def sportmodestatus_callback(self, message):  
        self.current_state = message['data']

    def publish_odometry(self):
        if self.current_state is None:
            return  # Avoid running if current_state is not set yet

        odom_msg = Odometry()
        position = self.current_state['position']
        
        # Set position data
        odom_msg.pose.pose.position = Point(position[0], position[1], position[2])
        
        # Example: converting yaw from IMU to quaternion
        yaw = self.current_state['imu_state']['rpy'][2]  # Roll-Pitch-Yaw from IMU
        q = quaternion_from_euler(0, 0, yaw)
        odom_msg.pose.pose.orientation = Quaternion(*q)
        odom_msg.header.frame_id = 'map'
        
        # Publish odometry message to ROS
        self.odom_pub.publish(odom_msg)

    def waypoint_callback(self, msg):
        xgoal = msg.pose.position.x
        ygoal = msg.pose.position.y
        zgoal = msg.pose.position.z

        x = msg.pose.orientation.x
        y = msg.pose.orientation.y
        z = msg.pose.orientation.z
        w = msg.pose.orientation.w

        quat_norm = np.linalg.norm([x, y, z, w])
        if quat_norm == 0:
            yawgoal = self.current_state['imu_state']['rpy'][2]
        else:
            quat = R.from_quat([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
            yawgoal = quat.as_euler('xyz')[2]

        print(f"Received waypoint: x={xgoal}, y={ygoal}, z={zgoal}, yaw={yawgoal}")
        self.new_waypoint_received = True

    def init_motion(self):
        ####### NORMAL MODE ########
        print("Checking current motion mode...")

        # Get the current motion_switcher status
        response = self.conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["MOTION_SWITCHER"], 
            {"api_id": 1001}
        )

        if response['data']['header']['status']['code'] == 0:
            data = json.loads(response['data']['data'])
            current_motion_switcher_mode = data['name']
            print(f"Current motion mode: {current_motion_switcher_mode}")

         # Switch to "normal" mode if not already
        if current_motion_switcher_mode != "normal":
            print(f"Switching motion mode from {current_motion_switcher_mode} to 'normal'...")
            self.conn.datachannel.pub_sub.publish_request_new(
                RTC_TOPIC["MOTION_SWITCHER"], 
                {
                    "api_id": 1002,
                    "parameter": {"name": "normal"}
                }
            )

        print("StandUp ...")
        self.conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["SPORT_MOD"], 
            {
                #"api_id": SPORT_CMD["StandUp"] # StandUp is lock
                "api_id": SPORT_CMD["RecoveryStand"] # StandUp is lock
            }
        )

        if self.current_state != None:
            print(self.current_state['position'])
            imu_state = self.current_state['imu_state']
            w,x,y,z = imu_state['quaternion']
            r = R.from_quat([x,y,z,w])
            print(r.as_euler('xyz'))

        print("Stop ... ")
        self.conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["SPORT_MOD"], 
            {
                "api_id": SPORT_CMD["StopMove"],
            }
        )

    def move_server(self):
        rate = rospy.Rate(30)
        cycle = 0
        while not rospy.is_shutdown():
            cycle += 1
            print(f"Cycle: {cycle}, current position: {self.current_state['position']}, current yaw: {self.current_state['imu_state']['rpy'][2]}")

            if self.current_state != None:
                print("Start position ...")
                print(self.current_state['position'])
                imu_state = self.current_state['imu_state']
                w,x,y,z = imu_state['quaternion']
                r = R.from_quat([x,y,z,w])
                print(r.as_euler('xyz'))

            if abs(self.current_state['position'][0] - goal_pos[0]) < tol and abs(self.current_state['position'][1] - goal_pos[1]) < tol:
                print("Reach goal, Stop ... ")
                self.conn.datachannel.pub_sub.publish_request_new(
                    RTC_TOPIC["SPORT_MOD"], 
                    {
                        "api_id": SPORT_CMD["StopMove"],
                    }
                )
                break
            else:
                print("Trajectory ...")
                trajectory = genTrajectory(goal_pos)
                # for i in range(len(trajectory)):
                #     print(f"Trajectory point {i}: x={trajectory[i]['x']}, y={trajectory[i]['y']}, yaw={trajectory[i]['yaw']}, vx={trajectory[i]['vx']}, vy={trajectory[i]['vy']}, vyaw={trajectory[i]['vyaw']}")
                self.conn.datachannel.pub_sub.publish_request_new(
                    RTC_TOPIC["SPORT_MOD"], 
                    {
                        "api_id": SPORT_CMD["TrajectoryFollow"],
                        "parameter": trajectory
                    }
                )

            # await asyncio.sleep(3)
            print("finish one loop")
            if self.current_state != None:
                print("End position ...")
                print(self.current_state['position'])
                imu_state = self.current_state['imu_state']
                w,x,y,z = imu_state['quaternion']
                r = R.from_quat([x,y,z,w])
                print(r.as_euler('xyz'))
            # await asyncio.sleep(3)
            
            rate.sleep()  # Maintain 10Hz loop


if __name__ == "__main__":
    rospy.init_node('go2_robot_controller', anonymous=True)
    go2_base_agent = go2_base()