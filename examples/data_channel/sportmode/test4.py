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
current_state = None
goal_position = [0.0, 0.0, 0.0]

rospy.init_node('go2_robot_controller', anonymous=True)
odom_pub = rospy.Publisher('/_odom', Odometry, queue_size=10)

# Define a function to publish odometry data to ROS.
def publish_odometry():
    if current_state is None:
        return  # Avoid running if current_state is not set yet

    odom_msg = Odometry()
    position = current_state['position']
    
    # Set position data
    odom_msg.pose.pose.position = Point(position[0], position[1], position[2])
    
    yaw = current_state['imu_state']['rpy'][2]  # Roll-Pitch-Yaw from IMU
    q = quaternion_from_euler(0, 0, yaw)
    odom_msg.pose.pose.orientation = Quaternion(*q)
    odom_msg.header.frame_id = 'map'
    
    # Publish odometry message to ROS
    odom_pub.publish(odom_msg)



# def genTrajectory(pos0):
#     print(pos0[0], pos0[1], pos0[2])
#     head_dir = [np.cos(pos0[2]), np.sin(pos0[2])]
#     dt = 0.3
#     sp = 0.5 # speed
#     ret = []
#     for i in range(31):
#         tt = dt * i # target time
#         data = {
#             "t_from_start": tt,
#             "x": sp * head_dir[0] * tt + pos0[0],
#             "y": sp * head_dir[1] * tt + pos0[1],
#             "yaw": pos0[2],
#             "vx": head_dir[0] * sp,
#             "vy": head_dir[1] * sp,
#             "vyaw": 0.0
#         }
#         #if (i % 10) == 9:
#         #    print(data)
#         ret.append(data)
#     return ret

# def genTrajectory(pos0):
#     x0 = pos0[0]
#     y0 = pos0[1]
#     yaw = current_state['imu_state']['rpy'][2]

#     # create trajectory
#     trajectory = []
#     numpts = 30
#     vx = 0.3
#     dt = 0.06
#     count = dt
#     for i in range(numpts):
#         var = count + i*dt
#         trajectory.append({
#             "t_from_start": i * dt,
#             "x": vx * var + x0,
#             "y": 0.6 * np.sin(np.math.pi * var *vx) + y0,
#             "yaw": 2 * 0.6 * vx * np.math.pi * np.cos(np.math.pi * var * vx),
#             "vx": vx,
#             "vy": 0.6 * np.math.pi * vx * np.cos(np.math.pi * var * vx),
#             "vyaw": -np.math.pi * 2 * 0.6 * vx * vx * np.math.pi * np.sin(np.math.pi * var * vx)
#         })
#     return trajectory

# def genTrajectory(pos0, dir):
#     x0 = pos0[0]
#     y0 = pos0[1]
#     yaw = current_state['imu_state']['rpy'][2]

#     # create trajectory
#     trajectory = []
#     numpts = 30
#     vx = 1.0 * dir
#     vy = 0.0
#     vyaw = 0.0
#     dt = 0.06
#     count = dt
#     for i in range(numpts):
#         time = count + i*dt
#         # print(time)
#         trajectory.append({
#             "t_from_start": i * dt,
#             "x": x0 + vx * time,
#             "y": y0 + vy * time,
#             "yaw": yaw + vyaw * time,
#             "vx": vx,
#             "vy": vy,
#             "vyaw": vyaw
#         })
#     return trajectory

# receiving the target pose, and generate the linear trajectory
def genTrajectory(target_pose):
    x0, y0, yaw = current_state['position'][0], current_state['position'][1], current_state['imu_state']['rpy'][2]
    x1, y1 = target_pose[0], target_pose[1]
    dir_ray = [x1 - x0, y1 - y0]
    dir_ray_norm = np.linalg.norm(dir_ray)
    dir_ray = [dir_ray[0] / dir_ray_norm, dir_ray[1] / dir_ray_norm]
    dt = 0.1
    sp = 0.5 # speed
    trajectory = []
    for i in range(30):
        tt = dt * i
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

# cpp example
# def genTrajectory(pos0, dir):
#     x0 = pos0[0]
#     y0 = pos0[1]
#     yaw = current_state['imu_state']['rpy'][2]  # Initial yaw

#     # Parameters for constant speed motion
#     speed = 1.0  # Constant speed for the trajectory
#     curvature_radius = 5.0  # Radius of curvature for the trajectory
#     angular_velocity = speed / curvature_radius  # Constant angular velocity for the curve
    
#     # Create trajectory
#     trajectory = []
#     numpts = 30
#     dt = 0.06  # Time step
    
#     for i in range(numpts):
#         time = i * dt
        
#         # Calculate the new positions using a circular trajectory (second-order curve)
#         # Assuming the robot moves along a circle with a radius `curvature_radius`
#         x_new = x0 + curvature_radius * np.sin(angular_velocity * time) * dir[0]
#         y_new = y0 + curvature_radius * (1 - np.cos(angular_velocity * time)) * dir[1]
#         yaw_new = yaw + angular_velocity * time * dir[0]  # Update yaw along the curve
        
#         # Constant speed in both x and y directions (since it's circular motion)
#         vx = speed * np.cos(angular_velocity * time) * dir[0]
#         vy = speed * np.sin(angular_velocity * time) * dir[1]
#         vyaw = angular_velocity * dir[0]  # Yaw rate remains constant

#         # Append the new trajectory point
#         trajectory.append({
#             "t_from_start": i * dt,
#             "x": x_new,
#             "y": y_new,
#             "yaw": yaw_new,
#             "vx": vx,
#             "vy": vy,
#             "vyaw": vyaw
#         })
    
#     return trajectory



# def genTrajectory(pos0):
#     x0 = pos0[0]
#     y0 = pos0[1]
#     yaw = current_state['imu_state']['rpy'][2]

#     # create trajectory
#     trajectory = []
#     numpts = 30
#     vx = 1.0
#     vy = 0.0
#     vyaw = 0.0
#     dt = 0.06
#     count = dt
#     for i in range(numpts):
#         time = count + i*dt
#         print(time)
#         trajectory.append({
#             "t_from_start": i * dt,
#             "x": x0 + vx * time,
#             "y": y0 + vy * time,
#             "yaw": yaw + vyaw * time,
#             "vx": vx,
#             "vy": vy,
#             "vyaw": vyaw
#         })
#     return trajectory

# def goalReacched():

# Signal handler for graceful shutdown
def signal_handler(sig, frame):
    print('🕒 Handler: Signaling State          : ⚫ closed        (11:24:48)')
    rospy.signal_shutdown('Ctrl+C pressed')
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

def sportmodestatus_callback(message):
    global current_state
    current_state = message['data']
    #display_data(current_state)

def continuous_odometry_publishing():
    rate = rospy.Rate(30) 
    while not rospy.is_shutdown():
        publish_odometry()  # Call the function to publish odometry
        rate.sleep()

def start_odometry_thread():
    print("Starting odometry thread...")
    odom_thread = threading.Thread(target=continuous_odometry_publishing)
    odom_thread.daemon = True  # Daemon thread to exit when the main program exits
    odom_thread.start()

async def main():
    global current_state
    current_state = None

    # Connect to the WebRTC service
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

    await asyncio.sleep(3)

    if current_state != None:
        print(current_state['position'])
        imu_state = current_state['imu_state']
        w,x,y,z = imu_state['quaternion']
        r = R.from_quat([x,y,z,w])
        print(r.as_euler('xyz'))

    '''
    # Switch to AI mode
    print("Switching motion mode to 'AI'...")
    await conn.datachannel.pub_sub.publish_request_new(
        RTC_TOPIC["MOTION_SWITCHER"], 
        {
            "api_id": 1002,
            "parameter": {"name": "ai"}
        }
    )
    await asyncio.sleep(10)
    '''
    # print("Move Rotate ... ")
    # await conn.datachannel.pub_sub.publish_request_new(
    #     RTC_TOPIC["SPORT_MOD"], 
    #     {
    #         "api_id": SPORT_CMD["Move"],
    #         "parameter": {"x": 0.0, "y": 0.0, "z": -1.5}
    #     }
    # )

    await asyncio.sleep(1)

    print("Stop ... ")
    await conn.datachannel.pub_sub.publish_request_new(
        RTC_TOPIC["SPORT_MOD"], 
        {
            "api_id": SPORT_CMD["StopMove"],
        }
    )

    await asyncio.sleep(3)
    
    # if current_state != None:
    #     print(current_state['position'])
    #     imu_state = current_state['imu_state']
    #     w,x,y,z = imu_state['quaternion']
    #     r = R.from_quat([x,y,z,w])
    #     print(r.as_euler('xyz'))

    # print("Trajectory ...")
    # trajectory = genTrajectory(current_state['position'])
    # # print out all steps in trajectory
    # # for j in range(6):
    # #     for i in range(len(trajectory)):
    # #         print(f"Trajectory point {i}: x={trajectory[i]['x']}, y={trajectory[i]['y']}, yaw={trajectory[i]['yaw']}, vx={trajectory[i]['vx']}, vy={trajectory[i]['vy']}, vyaw={trajectory[i]['vyaw']}")
    # #     await conn.datachannel.pub_sub.publish_request_new(
    # #         RTC_TOPIC["SPORT_MOD"], 
    # #         {
    # #             "api_id": SPORT_CMD["TrajectoryFollow"],
    # #             "parameter": trajectory
    # #         }
    # #     )
    # for i in range(len(trajectory)):
    #     print(f"Trajectory point {i}: x={trajectory[i]['x']}, y={trajectory[i]['y']}, yaw={trajectory[i]['yaw']}, vx={trajectory[i]['vx']}, vy={trajectory[i]['vy']}, vyaw={trajectory[i]['vyaw']}")
    # await conn.datachannel.pub_sub.publish_request_new(
    #     RTC_TOPIC["SPORT_MOD"], 
    #     {
    #         "api_id": SPORT_CMD["TrajectoryFollow"],
    #         "parameter": trajectory
    #     }
    # )

    # await asyncio.sleep(3)
    # if current_state != None:
    #     print(current_state['position'])
    #     imu_state = current_state['imu_state']
    #     w,x,y,z = imu_state['quaternion']
    #     r = R.from_quat([x,y,z,w])
    #     print(r.as_euler('xyz'))
    # await asyncio.sleep(3)

    tol = 0.1
    cycle = 0
    goal_position = [0.0, 0.0, 0.0]

    start_odometry_thread()

    try:
        goal_pos = [goal_position[0], goal_position[1]]
        trajectory = genTrajectory(goal_pos)
        print("Init trajectory ...")
        for i in range(len(trajectory)):
            print(f"Trajectory point {i}: x={trajectory[i]['x']}, y={trajectory[i]['y']}, yaw={trajectory[i]['yaw']}, vx={trajectory[i]['vx']}, vy={trajectory[i]['vy']}, vyaw={trajectory[i]['vyaw']}")

        rate = rospy.Rate(30)
        while not rospy.is_shutdown():

            
            cycle += 1
            print(f"Cycle: {cycle}, current position: {current_state['position']}, current yaw: {current_state['imu_state']['rpy'][2]}")

            if current_state != None:
                print("Start position ...")
                print(current_state['position'])
                imu_state = current_state['imu_state']
                w,x,y,z = imu_state['quaternion']
                r = R.from_quat([x,y,z,w])
                print(r.as_euler('xyz'))

            if abs(current_state['position'][0] - goal_pos[0]) < tol and abs(current_state['position'][1] - goal_pos[1]) < tol:
                print("Reach goal, Stop ... ")
                await conn.datachannel.pub_sub.publish_request_new(
                    RTC_TOPIC["SPORT_MOD"], 
                    {
                        "api_id": SPORT_CMD["StopMove"],
                    }
                )
                await asyncio.sleep(3)
                break
            else:
                print("Trajectory ...")
                trajectory = genTrajectory(goal_pos)
                # for i in range(len(trajectory)):
                #     print(f"Trajectory point {i}: x={trajectory[i]['x']}, y={trajectory[i]['y']}, yaw={trajectory[i]['yaw']}, vx={trajectory[i]['vx']}, vy={trajectory[i]['vy']}, vyaw={trajectory[i]['vyaw']}")
                await conn.datachannel.pub_sub.publish_request_new(
                    RTC_TOPIC["SPORT_MOD"], 
                    {
                        "api_id": SPORT_CMD["TrajectoryFollow"],
                        "parameter": trajectory
                    }
                )

            # await asyncio.sleep(3)
            print("finish one loop")
            if current_state != None:
                print("End position ...")
                print(current_state['position'])
                imu_state = current_state['imu_state']
                w,x,y,z = imu_state['quaternion']
                r = R.from_quat([x,y,z,w])
                print(r.as_euler('xyz'))
            # await asyncio.sleep(3)
            
            rate.sleep()  # Maintain 10Hz loop
    
    except ValueError as e:
        logging.error(f"An error occurred: {e}")

if __name__ == "__main__":
    asyncio.run(main())