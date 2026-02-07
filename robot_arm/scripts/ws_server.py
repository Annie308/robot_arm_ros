#!/usr/bin/env python3
import os
from fastapi import FastAPI, WebSocket
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
import uvicorn
import threading

from robot_arm_interfaces.srv import ServoAngles  # Custom interface
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import atexit
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
import array

class ArmPublisher(Node):
    def __init__(self):
        super().__init__('arm_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, 'arm_angles', 10)
        self.angles = [0,0,0,0,0,0]  # default
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def update_angles(self, new_angle, angle_index):
        self.angles[angle_index] = float(new_angle)

    def return_angles(self):
        return self.angles

    def timer_callback(self):
        msg = Float64MultiArray()
        msg.data = array.array('d', self.angles)
        
        self.publisher_.publish(msg)

        for angle in msg.data:
            self.get_logger().info('Publishing: "%lf"' % angle)


class ServoClient(Node):
    def __init__(self):
        super().__init__('servo_client')
        self.cli = self.create_client(ServoAngles, 'servo_angles')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ServoAngles.Request()

        self.angles = [0,0,0,0,0,0]  # default
    
    def update_angles(self, new_angle, angle_index):
        self.angles[angle_index] = float(new_angle)

    def send_request(self):
        self.req.angle_1 = float(self.angles[0])
        self.req.angle_2 = float(self.angles[1])
        self.req.angle_3 = float(self.angles[2])
        self.req.angle_4 = float(self.angles[3])
        self.req.angle_5 = float(self.angles[4])
        self.req.angle_6 = float(self.angles[5])

        self.get_logger().info('Sending angles to servo controller: "%s"' % self.angles)

        self.cli.call_async(self.req)


rclpy.init()
arm_pub = ArmPublisher()
servo_client = ServoClient()
executor = MultiThreadedExecutor()
executor.add_node(arm_pub)
executor.add_node(servo_client)
       
def start_ros():
    try:
        executor.spin()
    finally:
        executor.shutdown()
        arm_pub.destroy_node()
        servo_client.destroy_node()
        rclpy.shutdown()

def spin_ros(): 
    executor.spin() 

threading.Thread(target=spin_ros, daemon=True).start()

app = FastAPI()

# 🔹 Find the share directory of your package
package_share = get_package_share_directory('robot_arm')
static_dir = Path(package_share) / "static"

# Mount static files
app.mount("/static", StaticFiles(directory=static_dir), name="static")

# Serve index.html
@app.get("/")
async def get():
    return FileResponse(static_dir / "index.html")

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            data = await websocket.receive_text()
            print("Received:", data, flush=True)

            # Expect CSV string of 6 angles
            angle_index = int(data.split("joint")[1].split(":")[0]) - 1
            angles = float(data.split(":")[1]) * 3.14159 / 180.0 


            # Invert angles for joints 2, 4,5, 6 bc of RVIZ
            if angle_index == 1 or angle_index == 3 or angle_index == 4 or angle_index == 5:
                angles = -angles

            # Update publisher angles
            arm_pub.update_angles(angles, angle_index)

            if angle_index == 4:
                angles = 3.14159 - abs(angles)

            servo_client.update_angles(angles, angle_index)
            future = servo_client.send_request() 

            await websocket.send_text("Angles updated: " + str(angles))

    except Exception as e:
        print("WebSocket error:", e)
    finally:
        await websocket.close()

# -------------------- Start Uvicorn --------------------
if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
