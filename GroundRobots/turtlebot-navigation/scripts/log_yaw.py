import rclpy, math, time
from rclpy.node import Node
from sensor_msgs.msg import Imu

rclpy.init()
n = Node('yaw_logger')
t0 = time.time()
f = open('/tmp/yaw_log.txt', 'w')

def cb(m):
    z, w = m.orientation.z, m.orientation.w
    yaw = math.degrees(2 * math.atan2(z, w))
    line = f"{time.time()-t0:7.2f}  {yaw:+8.2f}"
    print(line); f.write(line + "\n"); f.flush()

n.create_subscription(Imu, '/imu/data', cb, 10)
try:
    rclpy.spin(n)
except KeyboardInterrupt:
    f.close()
