"""Actual Atlas, localization Driver and worker; isolated domain, no motion outputs."""
import json
import os
from pathlib import Path
import subprocess
import tempfile
import time

os.environ.update(ROBONIX_ATLAS='127.0.0.1:52551', ROS_DOMAIN_ID='189', ROS_LOCALHOST_ONLY='1',
                  ROBONIX_DRIVER_CONTRACT_ID='robonix/service/localization/driver')
from localization_outdoor import atlas_bridge as bridge
from localization_outdoor.configuration import CONTRACTS
from outdoor_nav.scene import AtlasLocalization

root=Path(__file__).resolve().parents[1]
run=Path(tempfile.mkdtemp(prefix='map-reference-smoke-'))
log=(run/'atlas.log').open('w')
atlas=subprocess.Popen(['/home/nvidia/.cargo/bin/robonix-atlas','--listen','127.0.0.1:52551',
    '--capabilities',str(root/'packages/service-localization-outdoor-rbnx/capabilities')],stdout=log,stderr=log)
try:
    from robonix_api import ATLAS
    deadline=time.monotonic()+8
    while True:
        try:
            ATLAS.find_capability(contract_id='robonix/service/localization/driver');break
        except Exception:
            if time.monotonic()>deadline:raise
            time.sleep(.05)
    bridge.service.bootstrap()
    config={'origin':{'latitude':0.,'longitude':0.},'require_map_reference':True,
        'topic_remap':{role:'/reference_smoke/'+role for role in CONTRACTS},
        'output_topics':{role:'/reference_smoke/'+role for role in ('odom','pose','status','tf','tf_static')},
        'run_dir':str(run/'localization'),'http_port':19988}
    result=bridge.init(config)
    assert bridge.worker is not None,repr(result)
    binding=AtlasLocalization(run/'localization/config.json')
    assert bridge.call('/status')['coordinate_reference']['origin_wgs84'] is None
    for city,origin in [('beijing',{'latitude':39.9042,'longitude':116.4074}),
                        ('hangzhou',{'latitude':30.22413029,'longitude':120.25979315833334})]:
        snapshot={'map_id':city+'-recording','version':'a'*16,'origin_wgs84':origin}
        binding.bind(snapshot)
        status=bridge.call('/status')
        assert status['active'] and status['coordinate_reference']['map_id']==snapshot['map_id'],status
        assert status['coordinate_reference']['origin_wgs84']==origin,status
        bridge.deactivate()
        binding.bind(snapshot)
        assert bridge.call('/status')['active']
    print(json.dumps({'passed':True,'domain':189,'beijing_and_hangzhou_bound':True,
        'same_config_reactivation':True,'robot_motion':False}))
finally:
    bridge.cleanup()
    bridge.service._teardown()
    atlas.terminate()
    atlas.wait(timeout=5)
    log.close()
