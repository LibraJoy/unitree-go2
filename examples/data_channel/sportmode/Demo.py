import asyncio
import logging
import json
import numpy as np
from go2_webrtc_driver.webrtc_driver import Go2WebRTCConnection, WebRTCConnectionMethod
from go2_webrtc_driver.constants import RTC_TOPIC, SPORT_CMD
import rospy
from geometry_msgs.msg import PoseStamped,Point, Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
import time
import math
# Enable logging for debugging
logging.basicConfig(level=logging.FATAL)

rospy.init_node('go2_robot_controller', anonymous=True)
odom_pub = rospy.Publisher('/_odom', Odometry, queue_size=10)


def sportmodestatus_callback(message):
        global current_state
        current_state = message['data']   

# Define a function to publish odometry data to ROS.
def publish_odometry():
    odom_msg = Odometry()
    # Assume current_state['position'] gives us x, y, z
    position = current_state['position']  # Fetch current position from robot data
    
    # Set position data
    odom_msg.pose.pose.position = Point(position[0], position[1], position[2])
    
    # Example: converting yaw from IMU to quaternion
    yaw = current_state['imu_state']['rpy'][2]  # Roll-Pitch-Yaw from IMU
    q = quaternion_from_euler(0, 0, yaw)
    odom_msg.pose.pose.orientation = Quaternion(*q)
    
    # Publish odometry message to ROS
    odom_pub.publish(odom_msg)
    
    # Loop at 10 Hz for publishing odometry
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        publish_odometry()
        rate.sleep()

def waypoint_callback(msg):
    # Extract waypoint position from the received message
    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z
    
    print(f"Received waypoint: x={x}, y={y}, z={z}")
    
    # Prepare a trajectory path based on the waypoint
    trajectory = []
    for i in range(30):  # Assume we create 30 trajectory points leading to the waypoint
        t_from_start = i * 0.1  # Time increment
        trajectory.append({
            "tFromStart": t_from_start,
            "x": x * (i / 30.0),  # Scale position over time
            "y": y * (i / 30.0),
            "yaw": 0,  # Simplified for now
            "vx": 0.5,  # Example velocity in x direction
            "vy": 0.0,
            "vyaw": 0.0  # No rotation for simplicity
        })
    
    asyncio.run(conn.datachannel.pub_sub.publish_request_new(
        RTC_TOPIC["SPORT_MOD"],
        {
            "api_id": SPORT_CMD["TrajectoryFollow"],
            "parameter": {"path": trajectory}
        }
    ))
rospy.Subscriber('navigation/waypoints', PoseStamped, waypoint_callback)

async def main():
    global current_state
    current_state = None

    # Connect to the WebRTC service.
    conn = Go2WebRTCConnection(WebRTCConnectionMethod.Remote, serialNumber="B42D4000O1GBDIG2", username="fy.ya20026@gmail.com", password="Ykk1234?")
    await conn.connect()
    conn.datachannel.pub_sub.subscribe(RTC_TOPIC['LF_SPORT_MOD_STATE'], sportmodestatus_callback)

    ####### NORMAL MODE ########
    print("Checking current motion mode...")

    # Get the current motion_switcher status
    response = await conn.datachannel.pub_sub.publish_request_new(
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
        await conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["MOTION_SWITCHER"], 
            {
                "api_id": 1002,
                "parameter": {"name": "normal"}
            }
        )
        await asyncio.sleep(5)  # Wait while it stands up

    print("StandUp ...")
    await conn.datachannel.pub_sub.publish_request_new(
        RTC_TOPIC["SPORT_MOD"], 
        {
            #"api_id": SPORT_CMD["StandUp"] # StandUp is lock
            "api_id": SPORT_CMD["RecoveryStand"] # StandUp is lock
        }
    )
    print(current_state['position'])
    await asyncio.sleep(3)
    # await asyncio.sleep(3)
    # print(current_state['position'])
    # # Keep the program running for a while
    # await conn.datachannel.pub_sub.publish_request_new(
    #     RTC_TOPIC["SPORT_MOD"], 
    #     {
    #         "api_id": SPORT_CMD["StandUp"] # StandUp is lock
    #     }
    # )
    # await asyncio.sleep(1)
    
    # print("Sit ...")
    # await conn.datachannel.pub_sub.publish_request_new(
    #     RTC_TOPIC["SPORT_MOD"], 
    #     {
    #         "api_id": SPORT_CMD["Damp"]
    #     }
    # )

    try:
        while True:
            # conn.datachannel.pub_sub.subscribe(RTC_TOPIC['LF_SPORT_MOD_STATE'], sportmodestatus_callback)
            publish_odometry()
            rospy.spin()
    
    except ValueError as e:
        # Log any value errors that occur during the process.
        logging.error(f"An error occurred: {e}")


if __name__ == "__main__":
    asyncio.run(main())
