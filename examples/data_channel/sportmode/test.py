import asyncio
import logging
import json
import numpy as np
from go2_webrtc_driver.webrtc_driver import Go2WebRTCConnection, WebRTCConnectionMethod
from go2_webrtc_driver.constants import RTC_TOPIC, SPORT_CMD

# Enable logging for debugging
logging.basicConfig(level=logging.FATAL)
def display_data(message):

    imu_state = message['imu_state']
    quaternion = imu_state['quaternion']
    gyroscope = imu_state['gyroscope']
    accelerometer = imu_state['accelerometer']
    rpy = imu_state['rpy']
    temperature = imu_state['temperature']

    mode = message['mode']
    progress = message['progress']
    gait_type = message['gait_type']
    foot_raise_height = message['foot_raise_height']
    position = message['position']
    body_height = message['body_height']
    velocity = message['velocity']
    yaw_speed = message['yaw_speed']
    range_obstacle = message['range_obstacle']
    foot_force = message['foot_force']
    foot_position_body = message['foot_position_body']
    foot_speed_body = message['foot_speed_body']

    # Clear the entire screen and reset cursor position to top
    sys.stdout.write("\033[H\033[J")

    # Print each piece of data on a separate line
    print("Go2 Robot Status")
    print("===================")
    print(f"Mode: {mode}")
    # print(f"Progress: {progress}")
    # print(f"Gait Type: {gait_type}")
    # print(f"Foot Raise Height: {foot_raise_height} m")
    print(f"Position: {position}")
    # print(f"Body Height: {body_height} m")
    # print(f"Velocity: {velocity}")
    # print(f"Yaw Speed: {yaw_speed}")
    # print(f"Range Obstacle: {range_obstacle}")s
    # print(f"Foot Force: {foot_force}")
    # print(f"Foot Position (Body): {foot_position_body}")
    # print(f"Foot Speed (Body): {foot_speed_body}")
    print("-------------------")
    # print(f"IMU - Quaternion: {quaternion}")
    # print(f"IMU - Gyroscope: {gyroscope}")
    # print(f"IMU - Accelerometer: {accelerometer}")
    # print(f"IMU - RPY: {rpy}")
    # print(f"IMU - Temperature: {temperature}°C")
    
    # Optionally, flush to ensure immediate output
    sys.stdout.flush()    

# def sportmodestatus_callback(message):
#     print("Callback triggered")  # Log when the callback is invoked
#     current_message = message['data']
#     display_data(current_message)


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

        # Connect to the WebRTC service.
        await conn.connect()

        ####### NORMAL MODE ########
        print("Checking current motion mode...")
        def sportmodestatus_callback(message):
            current_message = message['data']
            print(message['data']['position'])
            # display_data(current_message)
        # Subscribe to the sportmode status data and use the callback function to process incoming messages.
        # conn.datachannel.pub_sub.subscribe(RTC_TOPIC['LF_SPORT_MOD_STATE'], sportmodestatus_callback)

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

        print("Move Forward ... ")
        await conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["SPORT_MOD"], 
            {
                "api_id": SPORT_CMD["Move"],
                "parameter": {"x": 2.0, "y": 0, "z": 0}
            }
        )
        # conn.datachannel.pub_sub.subscribe(RTC_TOPIC['LF_SPORT_MOD_STATE'], sportmodestatus_callback)
        # await asyncio.sleep(2)

        # print("Rotation CCW ... ")
        # await conn.datachannel.pub_sub.publish_request_new(
        #     RTC_TOPIC["SPORT_MOD"], 
        #     {
        #         "api_id": SPORT_CMD["Move"],
        #         "parameter": {"x": 0.0, "y": 0, "z": 90.0/np.pi}
        #     }
        # )

        # await asyncio.sleep(2)

        # print("Rotation CW ...")
        # await conn.datachannel.pub_sub.publish_request_new(
        #     RTC_TOPIC["SPORT_MOD"], 
        #     {
        #         "api_id": SPORT_CMD["Move"],
        #         "parameter": {"x": 0.0, "y": 0, "z": -90.0/np.pi}
        #     }
        # )

        # await asyncio.sleep(1)

        # print("StandDown ...")
        # await conn.datachannel.pub_sub.publish_request_new(
        #     RTC_TOPIC["SPORT_MOD"], 
        #     {
        #         "api_id": SPORT_CMD["StandDown"]
        #     }
        # )

        # await asyncio.sleep(1)

        # print("StandUp ...")
        # await conn.datachannel.pub_sub.publish_request_new(
        #     RTC_TOPIC["SPORT_MOD"], 
        #     {
        #         "api_id": SPORT_CMD["RecoveryStand"]
        #     }
        # )

        # await asyncio.sleep(1)
        
        # print("Rotation CCW ... ")
        # await conn.datachannel.pub_sub.publish_request_new(
        #     RTC_TOPIC["SPORT_MOD"], 
        #     {
        #         "api_id": SPORT_CMD["Move"],
        #         "parameter": {"x": 0.0, "y": 0, "z": 90.0/np.pi}
        #     }
        # )

        # await asyncio.sleep(2)

        # print("Rotation CW ...")
        # await conn.datachannel.pub_sub.publish_request_new(
        #     RTC_TOPIC["SPORT_MOD"], 
        #     {
        #         "api_id": SPORT_CMD["Move"],
        #         "parameter": {"x": 0.0, "y": 0, "z": -90.0/np.pi}
        #     }
        # )

        # await asyncio.sleep(1)

        print("StandUp ...")
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

        await asyncio.sleep(3)
        # Keep the program running for a while
        await asyncio.sleep(60)
    
    except ValueError as e:
        # Log any value errors that occur during the process.
        logging.error(f"An error occurred: {e}")

if __name__ == "__main__":
    asyncio.run(main())
