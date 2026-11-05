"""Field entry point: only independent packages, explicit motion arming, no posture writes."""
import argparse,json,os,signal,subprocess,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]


def environment():
    env=dict(os.environ)
    env.update(ROBONIX_ATLAS='127.0.0.1:51551',ROS_DOMAIN_ID='80',ROS_LOCALHOST_ONLY='0',RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
        G70_PYTHON=str(ROOT/'.venv/bin/python'),WIT_PYTHON=str(ROOT/'.venv/bin/python'),
        LOCALIZATION_PYTHON=str(ROOT/'.venv/bin/python'),OUTDOOR_SERVICE_PYTHON=str(ROOT/'.venv/bin/python'),
        RBNX_CODEGEN_PYTHON=str(ROOT/'.venv/bin/python'),ROBONIX_DRIVER_INIT_TIMEOUT_S='180')
    secret=ROOT/'config/secrets.json'
    if secret.exists():
        if secret.stat().st_mode&0o077:raise RuntimeError('config/secrets.json must have mode 600')
        values=json.loads(secret.read_text());env.update(values)
    return env


def process_alive(record):
    try:
        p=Path('/proc')/str(record['pid']);args=(p/'cmdline').read_bytes().split(b'\0')
        return record['manifest'].encode() in args and (p/'stat').read_text().split()[2]!='Z'
    except (OSError,KeyError):return False


def stop():
    p=ROOT/'run/owner.json'
    if not p.exists():return
    record=json.loads(p.read_text())
    if not process_alive(record):return
    os.kill(record['pid'],signal.SIGINT)
    deadline=time.monotonic()+55
    while process_alive(record) and time.monotonic()<deadline:time.sleep(.1)
    if process_alive(record):raise RuntimeError('Stack did not stop; inspect run/boot.log')
    print('Outdoor stack stopped')


def sdk():
    nav=ROOT/'packages/service-navigation-outdoor-rbnx'
    sys.path[:0]=[str(nav),str(nav/'rbnx-build/codegen/proto_gen'),str(nav/'rbnx-build/codegen/robonix_mcp_types')]
    import robonix_contracts_pb2_grpc
    from robonix_api import ATLAS
    os.environ['ROBONIX_ATLAS']='127.0.0.1:51551'
    return ATLAS


def client():
    atlas=sdk()
    cap=atlas.find_unique_capability(contract_id='robonix/service/navigation_outdoor/follow_reference',provider_id='outdoor_execution',transport='grpc')
    channel=atlas.connect_capability(consumer_id='outdoor_field_cli',provider_id=cap.provider_id,
        contract_id='robonix/service/navigation_outdoor/follow_reference',transport='grpc')
    from outdoor_nav.client import ExecutionClient
    return ExecutionClient(channel.endpoint),channel


def status():
    from urllib.request import urlopen
    data={}
    for label,url in (('localization','http://127.0.0.1:19980/status'),('navigation','http://127.0.0.1:19985/api/status')):
        try:
            with urlopen(url,timeout=2) as response:data[label]=json.load(response)
        except OSError:data[label]={'available':False}
    try:
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from std_msgs.msg import String
        context=Context();rclpy.init(args=[],context=context,domain_id=80)
        node=rclpy.create_node('outdoor_status_readonly',context=context)
        executor=SingleThreadedExecutor(context=context);executor.add_node(node);sample=[];gnss=[]
        node.create_subscription(String,'/outdoor/chassis_status',lambda m:sample.append(json.loads(m.data)),10)
        node.create_subscription(String,'/gps/rtk_status',lambda m:gnss.append(json.loads(m.data)),10)
        deadline=time.monotonic()+2.5
        while (not sample or not gnss) and time.monotonic()<deadline:executor.spin_once(timeout_sec=.1)
        if gnss:
            raw=gnss[-1]
            data['gnss']={key:raw.get(key) for key in ('state','receiver_quality','satellites','fix_age_s','position_covariance_source')}
            data['gnss']['velocity']=raw.get('velocity')
        data['chassis']=sample[-1] if sample else {'sample_observed':False,'reason':'no sample within probe budget'}
        data['private_cmd_vel_publishers']=node.count_publishers('/cmd_vel')
        data['private_native_command_publishers']=node.count_publishers('/NAV_CMD')
        executor.shutdown();node.destroy_node();rclpy.try_shutdown(context=context)
    except Exception as error:data['chassis']={'error':str(error)}
    print(json.dumps(data,indent=2,ensure_ascii=False))


def boot(profile):
    run=ROOT/'run';run.mkdir(exist_ok=True)
    owner=run/'owner.json'
    if owner.exists() and process_alive(json.loads(owner.read_text())):raise RuntimeError('Stack already running; use ./outdoor stop first')
    manifest=ROOT/'config'/('field.yaml' if profile=='field' else 'check.yaml')
    env=environment()
    subprocess.run(['rbnx','build','-f',str(manifest)],env=env,check=True)
    log=(run/'boot.log').open('a',buffering=1)
    p=subprocess.Popen(['rbnx','boot','-f',str(manifest),'--no-update-check','-v'],env=env,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    owner.write_text(json.dumps({'pid':p.pid,'manifest':str(manifest),'profile':profile}))
    print('Started',profile,'profile. Log:',run/'boot.log',flush=True)
    print('No goal or posture command was sent. Use ./outdoor status.',flush=True)


def main():
    p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='command',required=True)
    sub.add_parser('check',help='Start real sensors and localization, without any native velocity publisher')
    up=sub.add_parser('up');up.add_argument('--enable-motion',action='store_true')
    sub.add_parser('stop');sub.add_parser('status');sub.add_parser('cancel')
    a=p.parse_args()
    if a.command=='stop':stop();return
    if a.command=='status':status();return
    if a.command in ('check','up'):
        if a.command=='up' and not a.enable_motion:raise RuntimeError('Use check for non-motion mode; field output requires up --enable-motion')
        boot('check' if a.command=='check' else 'field');return
    c,ch=client()
    try:
        if a.command=='cancel':print(json.dumps(c.cancel(),ensure_ascii=False));return
    finally:c.close();ch.close()

if __name__=='__main__':
    try:main()
    except Exception as error:print('ERROR:',error,file=sys.stderr);raise SystemExit(1)
