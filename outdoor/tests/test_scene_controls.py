from starlette.testclient import TestClient
from scene_ui_fixture import fixture


def test_actual_scene_recorded_route_execute_and_cancel():
    with fixture() as (app,worker),TestClient(app) as client:
        state=client.get('/api/outdoor/state').json()
        assert state['is_active'] and state['navigation']['ready']
        assert client.get('/outdoor').status_code==200
        assert client.get('/api/outdoor/background').status_code in (404,405)
        snapshot=state['map']
        request={'map_id':snapshot['map_id'],'version':snapshot['version'],
                 'goal':{'x':6.,'y':0.}}
        preview=client.post('/api/outdoor/plan_route',json=request).json()
        assert preview['ok'] and preview['goal']==request['goal']
        assert worker.node.tasks.snapshot() is None
        accepted=client.post('/api/outdoor/execute',json={'preview_id':preview['preview_id'],
                             'request_id':'f'*32}).json()
        assert accepted['ok'],accepted
        task=worker.node.tasks.snapshot('f'*32)
        assert task['goal']==request['goal']
        assert task['reference_path'][-1]==request['goal']
        assert task['reference_path'][0]=={'x':0.,'y':0.}
        assert task['map_version']==snapshot['version']
        canceled=client.post('/api/outdoor/cancel',json={'run_id':'f'*32}).json()
        assert canceled['task']['state']=='canceling'
        assert client.get('/api/outdoor/state').json()['navigation']['task']['state']=='canceling'
