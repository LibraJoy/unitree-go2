import asyncio
import logging
import json
import numpy as np
from go2_webrtc_driver.webrtc_driver import Go2WebRTCConnection, WebRTCConnectionMethod
from go2_webrtc_driver.constants import RTC_TOPIC, SPORT_CMD

# Enable logging for debugging
logging.basicConfig(level=logging.FATAL)
    
async def main():
    try:
        # Choose a connection method (uncomment the correct one)
        # conn = Go2WebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip="192.168.8.181")
        # conn = Go2WebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip="192.168.0.230")
        conn = Go2WebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip="192.168.1.5")
        # conn = Go2WebRTCConnection(WebRTCConnectionMethod.LocalSTA, serialNumber="B42D2000XXXXXXXX")
        # conn = Go2WebRTCConnection(WebRTCConnectionMethod.Remote, serialNumber="B42D2000XXXXXXXX", username="email@gmail.com", password="pass")
        # conn = Go2WebRTCConnection(WebRTCConnectionMethod.Remote, serialNumber="B42D4000O1GBDIG2", username="fy.ya20026@gmail.com", password="Ykk1234?")
        # conn = Go2WebRTCConnection(WebRTCConnectionMethod.LocalAP)

        # Connect to the WebRTC service.
        await conn.connect()

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

        print("RecoveryStand ...")
        await conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["SPORT_MOD"], 
            {
                #"api_id": SPORT_CMD["StandUp"] # StandUp is lock
                "api_id": SPORT_CMD["RecoveryStand"] # StandUp is lock
            }
        )

        await asyncio.sleep(3)

        print("StandDown ...")
        await conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["SPORT_MOD"], 
            {
                "api_id": SPORT_CMD["StandDown"] # StandUp is lock
            }
        )

        await asyncio.sleep(3)

        # https://support.unitree.com/home/en/developer/D1Arm_services
        print("single joint angle control of the arm")
        await conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["ARM_COMMAND"], 
            {
                "api_id" : 1002,
                "seq": 4,
                "address":1,
                "funcode":1,
                "data":{ "id":5, "angle":60, "delay_ms":0}
            }
        )
        await asyncio.sleep(3)

        print("multiple joint angle control of the arm")
        await conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["ARM_COMMAND"], 
            {
                "api_id" : 1003,
                "seq ": 4,
                "address": 1,
                "funcode": 2,
                "data": {
                    "mode": 1,
                    "angle0" : 0,
                    "angle1" : -60,
                    "angle2" : 60,
                    "angle3" : 0,
                    "angle4" : 30,
                    "angle5" : 0,
                    "angle6" : 0
                }
            }
        )
        await asyncio.sleep(3)

        print("enable/unload force control of the arm joint motor")
        await conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["ARM_COMMAND"], 
            {
                "api_id" : 1004,
                "seq ": 4,
                "address": 1,
                "funcode": 5,
                "data": {
                    "mode": 0
                }
            }
        )
        await asyncio.sleep(3)
        
        print("robotic arm position and posture return to zero")
        await conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["ARM_COMMAND"], 
            {
                "api_id" : 1005,
                "seq ": 4,
                "address": 1,
                "funcode": 7
            }
        )
        await asyncio.sleep(3)

        # Keep the program running for a while
        await asyncio.sleep(3600)
    
    except ValueError as e:
        # Log any value errors that occur during the process.
        logging.error(f"An error occurred: {e}")

if __name__ == "__main__":
    asyncio.run(main())
