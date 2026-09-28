#!/usr/bin/env python3
"""Observe WAM-V watchdog and upstream thruster force-mode feedback.

Run on a fresh simulator-only partition. Upstream angular speed encodes its
accepted force (gz-sim8 Thruster.cc ThrustToAngularVec). This tests the running
plugin's input saturation, not direct load-cell force or marine fidelity.
"""
import hashlib
import json
import os
from pathlib import Path
import time
import xml.etree.ElementTree as ET
from gz.transport13 import Node
from gz.msgs10.double_pb2 import Double
from gz.msgs10.float_v_pb2 import Float_V


def main():
    out = Path(os.environ.get('OUTPUT_DIR', '/outputs'))
    result = dict(passed=False, evidence_level='live_watchdog_and_upstream_force_mode_response', checks=[])
    node = Node()
    seen = {}
    subscriptions = []
    pub = node.advertise('/njord/actuator_forces', Float_V)
    def send(forces):
        m = Float_V(); m.data.extend(forces); pub.publish(m)
    try:
        manifest = out/'run_manifest.json'
        result['manifest_sha256'] = hashlib.sha256(manifest.read_bytes()).hexdigest()
        result['image_identity'] = json.loads(manifest.read_text())['image_identity']
        result['probe_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        model = ET.parse(out/'wamv.sdf').find('model')
        plugins = [p for p in model.findall('plugin') if p.get('name') == 'gz::sim::systems::Thruster']
        if len(plugins) != 2: raise ValueError('expected WAM-V two-thruster model')
        k = {}
        limit = float(model.find("plugin[@name='njord::ActuatorWatchdog']/max_force_n").text)
        for p in plugins:
            root = '/'+p.findtext('namespace')+'/'+p.findtext('topic')
            side = p.findtext('name')
            k[side] = float(p.findtext('fluid_density'))*float(p.findtext('thrust_coefficient'))*float(p.findtext('propeller_diameter'))**4
            for suffix, kind in (('', 'forwarded'), ('/ang_vel', 'upstream_angular_speed')):
                def receive(msg, key=(side,kind)):
                    seen[key] = (time.monotonic(), msg.data)
                if not node.subscribe(Double, root+suffix, receive): raise ValueError('subscribe failed '+root+suffix)
                subscriptions.append(root+suffix)
        deadline = time.monotonic()+30
        while not pub.has_connections():
            if time.monotonic()>deadline: raise TimeoutError('no watchdog subscriber')
            time.sleep(.02)
        for factor in (.5, 2., -.5, -2.):
            target = max(-limit, min(limit, factor*limit))
            begin = time.monotonic(); deadline = begin+15; samples=[]
            while time.monotonic()<deadline:
                send([factor*limit]*2)
                time.sleep(.04)
                if all(key in seen and seen[key][0]>begin for key in ((s,c) for s in k for c in ('forwarded','upstream_angular_speed'))):
                    row={}
                    for side, coefficient in k.items():
                        omega=seen[(side,'upstream_angular_speed')][1]
                        row[side]={'forwarded_n':seen[(side,'forwarded')][1], 'upstream_equivalent_n':coefficient*omega*abs(omega)}
                    if all(abs(v-target)<=max(.02,abs(target)*1e-5) for values in row.values() for v in values.values()):
                        samples.append(row)
                        if len(samples)>=10: break
                    else: samples=[]
            passed = len(samples)>=10
            result['checks'].append({'command_n':factor*limit, 'expected_n':target, 'passed':passed, 'samples':samples})
            if not passed: raise RuntimeError(f'force response did not match {target} N')
        result['passed'] = True
    except Exception as error:
        result['failure'] = f'{type(error).__name__}: {error}'
    finally:
        for _ in range(5): send([0.,0.]); time.sleep(.04)
        for topic in subscriptions:
            node.unsubscribe(topic)
        # Stop callback delivery before interpreter finalization destroys Python
        # functions retained by Gazebo's transport threads.
        time.sleep(.1)
        with (out/'wamv_limits.json').open('x') as stream: json.dump(result, stream,indent=2,allow_nan=False)
    print(json.dumps({k:v for k,v in result.items() if k != 'checks'}))
    raise SystemExit(0 if result['passed'] else 2)

if __name__=='__main__': main()
