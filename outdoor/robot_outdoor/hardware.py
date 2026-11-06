"""Robonix-owned front lidar + read-only chassis feedback / explicitly armed bridge."""
import json,os,signal,subprocess,sys,time
from pathlib import Path
from robonix_api import Primitive,Ok,Err,Deferred

hardware=Primitive(id=os.environ.get('RBNX_INSTANCE_NAME','lynx_hardware'),namespace='robonix/primitive')
children=[];logs=[]


def child(args,env,label,run):
    log=(run/(label+'.log')).open('a',buffering=1);logs.append(log)
    p=subprocess.Popen(args,env=env,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    children.append(p);return p


def cleanup():
    for p in reversed(children):
        if p.poll() is None:
            os.killpg(p.pid,signal.SIGINT)
            try:p.wait(timeout=7)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=3)
    children.clear()
    for f in logs:f.close()
    logs.clear()


@hardware.on_init
def init(cfg):
    try:
        if children:return Ok()
        if type(cfg.get('allow_motion',False)) is not bool:raise ValueError('allow_motion must be boolean')
        if type(cfg.get('allow_gait',False)) is not bool:raise ValueError('allow_gait must be boolean')
        if cfg.get('allow_gait',False) and not cfg.get('allow_motion',False):raise ValueError('Gait output requires explicit motion arming')
        run=Path(cfg['run_dir']);run.mkdir(parents=True,exist_ok=True)
        env={**os.environ,'LYNX_ALLOW_MOTION':'1' if cfg.get('allow_motion',False) else '0',
            'LYNX_ALLOW_GAIT':'1' if cfg.get('allow_gait',False) else '0'}
        child([sys.executable,'-m','robot_outdoor.domain_bridge'],env,'domain_bridge',run)
        child([sys.executable,'-m','robot_outdoor.velocity_bridge'],env,'chassis_bridge',run)
        child(['ros2','run','tf2_ros','static_transform_publisher','--x','0.22341','--y','0','--z','-0.0001',
            '--roll','3.14159','--pitch','-1.5708','--yaw','0','--frame-id','base_link','--child-frame-id','airy_201',
            '--ros-args','-r','__node:=outdoor_front_lidar_tf'],env,'lidar_tf',run)
        child([cfg['lidar_executable'],'--ros-args','-p','config_path:='+cfg['lidar_config']],env,'front_lidar',run)
        import rclpy
        from rclpy.context import Context
        from sensor_msgs.msg import PointCloud2
        from rclpy.executors import SingleThreadedExecutor
        context=Context();rclpy.init(context=context)
        executor=SingleThreadedExecutor(context=context)
        node=rclpy.create_node('outdoor_lidar_sentinel',context=context);executor.add_node(node);seen=[]
        from rclpy.qos import qos_profile_sensor_data
        node.create_subscription(PointCloud2,'/airy_201/points',lambda m:seen.append(m.header.frame_id),qos_profile_sensor_data)
        deadline=time.monotonic()+10
        try:
            while not seen and time.monotonic()<deadline:
                if any(p.poll() is not None for p in children):raise RuntimeError('Hardware helper exited; inspect '+str(run))
                executor.spin_once(timeout_sec=.1)
        finally:executor.shutdown();node.destroy_node();rclpy.try_shutdown(context=context)
        if not seen:
            cleanup();return Deferred('No front lidar data; check power/UDP')
        if seen[-1]!='airy_201':raise ValueError('Unexpected front lidar frame')
        hardware.declare_ros2_topic('robonix/primitive/lidar/lidar3d','/airy_201/points',qos='reliable')
        hardware.declare_ros2_topic('robonix/primitive/chassis/twist_in','/cmd_vel',qos='reliable')
        if cfg.get('allow_gait',False):hardware.declare_ros2_topic('robonix/primitive/chassis/gait_in','/outdoor/gait_request',qos='reliable')
        return Ok()
    except Exception as e:
        import traceback
        traceback.print_exc()
        cleanup();return Err(str(e))


@hardware.on_shutdown
def shutdown():
    cleanup();return Ok()

if __name__=='__main__':hardware.run()
