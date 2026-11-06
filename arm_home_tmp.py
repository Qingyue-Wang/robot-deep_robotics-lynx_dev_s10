import rclpy, sys, time
from sensor_msgs.msg import JointState

rclpy.init()
n = rclpy.create_node('home_arm_tmp')
state = {'p': None}

def cb(m):
    state['p'] = list(m.position)

n.create_subscription(JointState, '/arm/joint_states_single', cb, 10)
t0 = time.time()
while state['p'] is None and time.time() - t0 < 5.0:
    rclpy.spin_once(n, timeout_sec=0.1)
if state['p'] is None:
    print('NO_STATE')
    sys.exit(1)
start = list(state['p'])
print('start', start)
names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper']
grip = start[6] if len(start) > 6 else 0.1
target = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, grip]
pub = n.create_publisher(JointState, '/arm/joint_command', 10)
steps = 240
dt = 0.02
for i in range(1, steps + 1):
    a = i / float(steps)
    msg = JointState()
    msg.name = names
    msg.position = [(1 - a) * start[j] + a * target[j] for j in range(7)]
    pub.publish(msg)
    rclpy.spin_once(n, timeout_sec=0)
    time.sleep(dt)
end = time.time() + 1.5
while time.time() < end:
    msg = JointState()
    msg.name = names
    msg.position = list(target)
    pub.publish(msg)
    rclpy.spin_once(n, timeout_sec=0)
    time.sleep(dt)
state['p'] = None
t0 = time.time()
while state['p'] is None and time.time() - t0 < 3.0:
    rclpy.spin_once(n, timeout_sec=0.1)
print('final', state['p'])
n.destroy_node()
rclpy.shutdown()
