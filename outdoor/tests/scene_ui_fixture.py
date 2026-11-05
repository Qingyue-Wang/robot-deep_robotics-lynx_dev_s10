"""Actual Scene + map core + generated navigation RPC; protocol fixture, no hardware."""
from concurrent import futures
from contextlib import contextmanager
import json
from pathlib import Path
import sys
import tempfile

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'tests'),str(ROOT/'framework/robonix/system/scene'),
               str(ROOT/'packages/service-map-outdoor-rbnx'),str(ROOT/'packages/service-navigation-outdoor-rbnx'),str(ROOT/'packages/service-navigation-outdoor-rbnx/tests')]
from test_scene_wire import Worker
from outdoor_nav import atlas_bridge as bridge
from outdoor_nav.client import ExecutionClient
from outdoor_nav.scene import AtlasExecution
from robonix_api.lifecycle import resolve_servicer,bind_user_handler
import robonix_contracts_pb2_grpc as contracts
from outdoor_mapping.core import Mapping
from outdoor_mapping.storage import MapStore
from scene_service import map_clients
from scene_service.annotations import AnnotationStore
from scene_service.map_meta import MapMetaStore
from scene_service.state.object_registry import ObjectRegistry
from scene_service.web import make_app


@contextmanager
def fixture():
    with tempfile.TemporaryDirectory(prefix='scene-ui-protocol-') as temporary:
        run=Path(temporary)
        from test_scene import MAP
        value={**MAP,'map_type':'outdoor','schema_version':1,'frame_id':'map','units':'meters',
            'example_data':True,'recordings':['synthetic-session'],'network':{'nodes':[], 'roads':[
                {'id':'recorded-road','from':'a','to':'b','points':[{'x':0.,'y':0.},{'x':20.,'y':0.}],
                 'length_m':20.,'gait':'stairs','gait_code':0x1003,'grade':'fixed',
                 'low_accuracy':False,'sources':['synthetic-session'],'bidirectional':True}]}}
        store=MapStore(run/'mapping');store.save(value)
        mapping=Mapping(run/'mapping');mapping.load_map(value['map_id'])
        status={'ready':True,'localization_grade':'fixed','not_ready_reasons':[],
                'pose':{'x':0.,'y':0.,'yaw':0.,'age_s':.01},'synthetic_protocol_fixture':True,
                'coordinate_reference':{'frame_id':'map','axes':'east_north',
                   'origin_wgs84':value['origin_wgs84'],'reference_point':'base_link',
                   'map_id':value['map_id'],'version':value['version'],
                   'map_offset_xy_m':[0.,0.],'antenna_xy_m':[0.,0.]}}
        worker=Worker(status);worker.node.runtime={"gait_switching_enabled":True};previous_manager=bridge.manager;bridge.manager=worker
        server=grpc_server=__import__('grpc').server(futures.ThreadPoolExecutor(max_workers=4))
        for contract,handler,_ in bridge.service._grpc_handlers:
            if not contract.startswith('robonix/service/navigation_outdoor/'):continue
            cls,method,add,_=resolve_servicer(contract,contracts)
            add(bind_user_handler(cls,method,handler)(),server)
        port=server.add_insecure_port('127.0.0.1:0');server.start()
        class Wire(AtlasExecution):
            @contextmanager
            def connection(self,action):
                client=ExecutionClient(f'127.0.0.1:{port}')
                try:yield client
                finally:client.close()
        previous_call=map_clients.call
        def call(namespace,op,payload=None,timeout_s=20.,provider_id=''):
            payload=payload or {}
            if op=='list_maps':return {'ok':True,'maps_json':json.dumps(store.list())}
            if op=='status':return {'ok':True,**mapping.status(),'example_data':True}
            try:
                result=getattr(mapping,op)(**payload) if op in ('load_map','save_map','delete_map') else getattr(mapping,op)(payload)
                return {'ok':True,**result}
            except (ValueError,KeyError) as error:return {'ok':False,'detail':str(error)}
        map_clients.call=call
        config=json.loads((ROOT/'config/scene_navigation.json').read_text())
        config.update(run_dir=str(run/'requests'),execution=Wire(),execution_enabled=True)
        config.pop('localization_config_path',None)
        app=make_app(registry=ObjectRegistry(),anno_store=AnnotationStore(str(run/'annotations'),map_id='.live'),
                     map_meta=MapMetaStore(str(run/'meta')),map_binding={'map_type':'outdoor'},
                     map_providers={'outdoor':'synthetic-map'},outdoor_navigation=config)
        try:yield app,worker
        finally:
            map_clients.call=previous_call;bridge.manager=previous_manager
            server.stop(0).wait();mapping.close()


if __name__=='__main__':
    import uvicorn
    with fixture() as (app,worker):
        uvicorn.run(app,host='127.0.0.1',port=50119,log_level='warning')
