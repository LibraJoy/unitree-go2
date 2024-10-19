import asyncio
import logging
import json
import numpy as np
import math
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

class go2_base:
    def __init__(self):
        self.odom_pub = rospy.Publisher('/go2_odom', Odometry, queue_size=10)
        # rospy.Subscriber('navigation/waypoints', PoseStamped, self.waypoint_callback)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_pos_click_callback)
        self.current_state = None
        self.goal_pos = [0.0, 0.0, 0.0] # x, y, yaw
        self.reach_tolerance = 0.1
        self.new_waypoint_received = False

        # start odom thread
        # self.odom_thread = threading.Thread(target=self.publish_odometry)
        # self.odom_thread.daemon = True  # Daemon thread to exit when the main program exits
        # self.odom_thread.start()

    async def async_init(self):
        # Connect to the WebRTC service
        self.conn = Go2WebRTCConnection(WebRTCConnectionMethod.Remote, serialNumber="B42D4000O1GBDIG2", username="fy.ya20026@gmail.com", password="Ykk1234?")
        await self.conn.connect()
        self.conn.datachannel.pub_sub.subscribe(RTC_TOPIC['LF_SPORT_MOD_STATE'], self.sportmodestatus_callback)

        # initialize motion
        await self.init_motion()
        asyncio.create_task(self.publish_odometry())

    def sportmodestatus_callback(self, message):
        self.current_state = message['data']

    async def publish_odometry(self):
        while not rospy.is_shutdown():
            if self.current_state is None:
                await asyncio.sleep(0.1)  # Sleep for a short duration to avoid busy-waiting
                continue  # Avoid running if current_state is not set yet

            odom_msg = Odometry()
            position = self.current_state['position']

            # Set position data
            odom_msg.pose.pose.position = Point(position[0], position[1], position[2])
            yaw = self.current_state['imu_state']['rpy'][2]  # Roll-Pitch-Yaw from IMU
            q = quaternion_from_euler(0, 0, yaw)
            odom_msg.pose.pose.orientation = Quaternion(*q)
            odom_msg.header.frame_id = 'map'

            # Publish odometry message to ROS
            self.odom_pub.publish(odom_msg)
            await asyncio.sleep(0.1)

    def waypoint_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        # z = msg.pose.position.z

        self.goal_pos = [x, y, 0]
        # self.move_server()
        asyncio.run_coroutine_threadsafe(self.move_server(),self.loop)

    def goal_pos_click_callback(self, msg):
        xgoal = msg.pose.position.x
        ygoal = msg.pose.position.y
        # zgoal = msg.pose.position.z

        self.new_waypoint_received = True

        self.goal_pos = [xgoal, ygoal, 0]
        print(f"New goal position received: {self.goal_pos}")
        # self.move_server()
        asyncio.run_coroutine_threadsafe(self.move_server(),self.loop)

    def get_desired_heading(self, dx, dy):
        position, quaternion = self.get_location()
        current_x = position.x
        current_y = position.y
        current_rot = R.from_quat([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        current_yaw = current_rot.as_euler('xyz', degrees=False)[2]

        x = dx - current_x
        y = dy - current_y
        goal_yaw = math.atan2(y, x)
        print(f"current yaw: {current_yaw}, goal yaw: {goal_yaw}")

        diff = abs(goal_yaw - current_yaw)
        if diff > math.pi:
            diff = 2 * math.pi - diff
        print(f"diff: {diff}")
        if math.pi / 8 < diff < 3 * math.pi / 4:
            dyaw = goal_yaw
        else:
            dyaw = current_yaw
        print(f"desired heading: {dyaw}")
        return current_yaw, current_x, current_y, dyaw

    async def init_motion(self):
        ####### NORMAL MODE ########
        print("Checking current motion mode...")

        # Get the current motion_switcher status
        response = await self.conn.datachannel.pub_sub.publish_request_new(
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
            await self.conn.datachannel.pub_sub.publish_request_new(
                RTC_TOPIC["MOTION_SWITCHER"],
                {
                    "api_id": 1002,
                    "parameter": {"name": "normal"}
                }
            )

        print("StandUp ...")
        await self.conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["SPORT_MOD"],
            {
                #"api_id": SPORT_CMD["StandUp"] # StandUp is lock
                "api_id": SPORT_CMD["RecoveryStand"] # StandUp is lock
            }
        )

        if self.current_state is not None:
            print(self.current_state['position'])
            imu_state = self.current_state['imu_state']
            w, x, y, z = imu_state['quaternion']
            r = R.from_quat([x, y, z, w])
            print(r.as_euler('xyz'))

        print("Stop ... ")
        await self.conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["SPORT_MOD"],
            {
                "api_id": SPORT_CMD["StopMove"],
            }
        )

    def genTrajectory(self, goal_pos):
        x0, y0, yaw = self.current_state['position'][0], self.current_state['position'][1], self.current_state['imu_state']['rpy'][2]
        x1, y1 = goal_pos[0], goal_pos[1]
        dir_ray = [x1 - x0, y1 - y0]
        dir_ray_norm = np.linalg.norm(dir_ray)
        dir_ray = [dir_ray[0] / dir_ray_norm, dir_ray[1] / dir_ray_norm]
        dt = 0.1
        sp = 0.5 # speed
        trajectory = []
        for i in range(30):
            tt = dt * i # time from start
            data = {
                "t_from_start": tt,
                "x": sp * dir_ray[0] * tt + x0,
                "y": sp * dir_ray[1] * tt + y0,
                "yaw": yaw,
                "vx": dir_ray[0] * sp,
                "vy": dir_ray[1] * sp,
                "vyaw": 0.0
            }
            trajectory.append(data)
        return trajectory

    async def move_server(self):
        rate = rospy.Rate(30)
        cycle = 0
        while not rospy.is_shutdown():
            cycle += 1
            print(f"Cycle: {cycle}, current position: {self.current_state['position']}, current yaw: {self.current_state['imu_state']['rpy'][2]}")

            if self.current_state != None:
                print("Start position ...")
                print(self.current_state['position'])

            if abs(self.current_state['position'][0] - self.goal_pos[0]) < self.reach_tolerance and abs(self.current_state['position'][1] - self.goal_pos[1]) < self.reach_tolerance:
                print("Reach goal, Stop ... ")
                await self.conn.datachannel.pub_sub.publish_request_new(
                    RTC_TOPIC["SPORT_MOD"],
                    {
                        "api_id": SPORT_CMD["StopMove"],
                    }
                )
                break
            else:
                print("Trajectory ...")
                trajectory = self.genTrajectory(self.goal_pos)
                await self.conn.datachannel.pub_sub.publish_request_new(
                    RTC_TOPIC["SPORT_MOD"],
                    {
                        "api_id": SPORT_CMD["TrajectoryFollow"],
                        "parameter": trajectory
                    }
                )

            print("finish one loop")
            if self.current_state != None:
                print("End position ...")
                print(self.current_state['position'])
            rate.sleep()

def signal_handler(sig, frame):
    rospy.signal_shutdown('Ctrl+C pressed')
    sys.exit(0)

if __name__ == "__main__":
    rospy.init_node('go2_robot_controller', anonymous=True)
    signal.signal(signal.SIGINT, signal_handler)
    loop = asyncio.get_event_loop()
    go2_base_agent = go2_base()
    go2_base_agent.loop = loop
    loop.run_until_complete(go2_base_agent.async_init())
    loop.run_forever()
