"""Isolated DDS: no /ODOM or posture writes, gated gait and final velocity timeout."""
import math,os,signal,threading,time
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from drdds.msg import MotionInfo,NavCmd,Gait
from std_msgs.msg import String,UInt32
import json
from .chassis_status import gait_relay_allowed


def main():
    robot=Context();private=Context()
    rclpy.init(args=[],context=robot,domain_id=int(os.environ.get('LYNX_ROBOT_DOMAIN_ID','0')))
    rclpy.init(args=[],context=private,domain_id=int(os.environ.get('ROS_DOMAIN_ID','80')))
    rn=Node('outdoor_native_domain',context=robot);pn=Node('outdoor_private_domain',context=private)
    info=pn.create_publisher(MotionInfo,'/MOTION_INFO',10)
    native_motion=[None,-math.inf];chassis=[{},-math.inf]
    def motion(msg):native_motion[:]=[msg,time.monotonic()];info.publish(msg)
    def chassis_status(msg):
        try:
            value=json.loads(msg.data)
            if isinstance(value,dict):chassis[:]=[value,time.monotonic()]
        except (ValueError,TypeError):pass
    rn.create_subscription(MotionInfo,'/MOTION_INFO',motion,10)
    pn.create_subscription(String,'/outdoor/chassis_status',chassis_status,10)
    if os.environ.get('LYNX_ALLOW_MOTION')=='1' and os.environ.get('LYNX_ALLOW_GAIT')=='1':
        gait_output=rn.create_publisher(Gait,'/GAIT',10)
        def gait_request(msg):
            now=time.monotonic();actual,at=native_motion;body,body_at=chassis
            mode_age=body.get('usage_mode_age_s')
            if actual is None or not gait_relay_allowed(msg.data,
                    actual.data.motion_state.state,now-at,body.get('usage_mode'),mode_age,
                    now-body_at,pn.count_publishers('/outdoor/gait_request')):return
            command=Gait();command.data.gait=msg.data;gait_output.publish(command)
        pn.create_subscription(UInt32,'/outdoor/gait_request',gait_request,10)
    output=None;latest=[None,-math.inf];stop=threading.Event()
    if os.environ.get('LYNX_ALLOW_MOTION')=='1':
        output=rn.create_publisher(NavCmd,'/NAV_CMD',10)
        def receive(msg):latest[:]=[msg,time.monotonic()]
        pn.create_subscription(NavCmd,'/NAV_CMD',receive,10)
        def tick():
            message=NavCmd();raw,at=latest
            if not stop.is_set() and raw is not None and time.monotonic()-at<.35 and pn.count_publishers('/NAV_CMD')<=1:
                x,yaw=raw.data.x_vel,raw.data.yaw_vel
                if math.isfinite(x) and math.isfinite(yaw):
                    message.data.x_vel=max(0.,min(.4,x));message.data.y_vel=0.;message.data.yaw_vel=max(-.4,min(.4,yaw))
            output.publish(message)
        rn.create_timer(.05,tick)
    rex=MultiThreadedExecutor(context=robot);rex.add_node(rn)
    pex=MultiThreadedExecutor(context=private);pex.add_node(pn)
    signal.signal(signal.SIGINT,lambda *_:stop.set());signal.signal(signal.SIGTERM,lambda *_:stop.set())
    for executor in (rex,pex):threading.Thread(target=executor.spin,daemon=True).start()
    stop.wait();rex.shutdown();pex.shutdown()
    if output is not None:
        for _ in range(20):output.publish(NavCmd());time.sleep(.05)
    rn.destroy_node();pn.destroy_node();rclpy.try_shutdown(context=robot);rclpy.try_shutdown(context=private)

if __name__=='__main__':main()
