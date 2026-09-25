import sys, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
rclpy.init(); n=Node('grab'); got=[]
n.create_subscription(CompressedImage,'/uav_top_cam/compressed',lambda m:got.append(bytes(m.data)),qos_profile_sensor_data)
import time; t=time.time()
while not got and time.time()-t<25: rclpy.spin_once(n,timeout_sec=0.5)
open('/tmp/top_frame.jpg','wb').write(got[0]) if got else print('NO FRAME')
print('frames',len(got))
