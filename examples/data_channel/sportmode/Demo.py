import asyncio
import logging
import json
import numpy as np
from go2_webrtc_driver.webrtc_driver import Go2WebRTCConnection, WebRTCConnectionMethod
from go2_webrtc_driver.constants import RTC_TOPIC, SPORT_CMD
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import math
# Enable logging for debugging
logging.basicConfig(level=logging.FATAL)
current_state = None



async def main():
    try:
        # Choose a connection method (uncomment the correct one)
        # conn = Go2WebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip="192.168.8.181")
        # conn = Go2WebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip="192.168.0.230")
        # conn = Go2WebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip="192.168.1.5")
        # conn = Go2WebRTCConnection(WebRTCConnectionMethod.LocalSTA, serialNumber="B42D2000XXXXXXXX")
        # conn = Go2WebRTCConnection(WebRTCConnectionMethod.Remote, serialNumber="B42D2000XXXXXXXX", username="email@gmail.com", password="pass")
        conn = Go2WebRTCConnection(WebRTCConnectionMethod.Remote, serialNumber="B42D4000O1GBDIG2", username="fy.ya20026@gmail.com", password="Ykk1234?")
        # conn = Go2WebRTCConnection(WebRTCConnectionMethod.LocalAP)

        # Communicate with ROS
        # odom_pub = rospy.Publisher('/spot/odom', Odometry, queue_size=10)
        # waypoint_sub = rospy.Subscriber("navigation/waypoints", PoseStamped, waypoints_callback)
        

        # Connect to the WebRTC service.
        await conn.connect()

        # Define a callback function to handle sportmode status when received.
        global current_state
        def sportmodestatus_callback(message):
            global current_state
            current_state = message['data']
        # Subscribe to the sportmode status data and use the callback function to process incoming messages.
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
        await asyncio.sleep(3)

        print(current_state['position'])
        print("Move Forward ... ")
        await send_trajectory_follow(conn)

        await asyncio.sleep(3)
        print(current_state['position'])
        # Keep the program running for a while
        await conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["SPORT_MOD"], 
            {
                "api_id": SPORT_CMD["StandUp"] # StandUp is lock
            }
        )
        await asyncio.sleep(1)
        
        print("Sit ...")
        await conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["SPORT_MOD"], 
            {
                "api_id": SPORT_CMD["Damp"]
            }
        )
        await asyncio.sleep(3600)
    
    except ValueError as e:
        # Log any value errors that occur during the process.
        logging.error(f"An error occurred: {e}")

def create_path_point(t_from_start, x, y, yaw, vx, vy, vyaw):
        return {
            "tFromStart": t_from_start,
            "x": x,
            "y": y,
            "yaw": yaw,
            "vx": vx,
            "vy": vy,
            "vyaw": vyaw
        }


async def send_trajectory_follow(conn):
    vx = 0.3  # Velocity in the x direction
    delta = 0.06  # Time increment
    count = 0  # Simulating the count that increments each cycle
    path = []

    # Generate 30 trajectory points
    for i in range(30):
        var = count + i * delta
        time_from_start = i * delta
        x = vx * var
        y = 0.6 * math.sin(math.pi * vx * var)
        yaw = 2 * 0.6 * vx * math.pi * math.cos(math.pi * vx * var)
        vy = math.pi * vx * (0.6 * math.cos(math.pi * vx * var))
        vyaw = -math.pi * vx * 2 * 0.6 * vx * math.pi * math.sin(math.pi * vx * var)

        path.append(create_path_point(time_from_start, x, y, yaw, vx, vy, vyaw))

    # Prepare the payload for sending the trajectory
    payload = {
        "api_id": SPORT_CMD["TrajectoryFollow"],
        "parameter": {
            "path": path
        }
    }

    # Send the command to follow the trajectory
    await conn.datachannel.pub_sub.publish_request_new(
        RTC_TOPIC["SPORT_MOD"],
        payload
    )
    print(current_state['position'])


if __name__ == "__main__":
    asyncio.run(main())
