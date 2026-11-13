"""Check generated URDF mass, drivetrain topology and finite actuator settings."""
import json
import subprocess
import xml.etree.ElementTree as ET
from ringquad_control import params as p

robot=ET.fromstring(subprocess.check_output(['xacro','src/ringquad_description/urdf/ringquad.urdf.xacro'],text=True))
mass=sum(float(m.attrib['value']) for m in robot.findall('link/inertial/mass'))
assert abs(mass-(p.MASS+.04))<1e-9
assert abs(mass-1.98)<1e-9
joints={j.attrib['name']:j for j in robot.findall('joint')}
for name in p.ROTOR_NAMES:
    for prefix in ['spin_','wheel_']:
        joint=joints[prefix+name];limit=joint.find('limit')
        assert float(limit.attrib['effort'])==.33
    assert joints['spin_'+name].find('child').attrib['link']=='prop_'+name
    assert joints['wheel_'+name].find('child').attrib['link']=='ring_'+name
    assert float(joints['wheel_'+name].find('limit').attrib['velocity'])==20
plugins=robot.findall('gazebo/plugin')
prop=[x for x in plugins if x.findtext('jointName','').startswith('spin_')]
wheel=[x for x in plugins if x.findtext('jointName','').startswith('wheel_')]
assert len(prop)==len(wheel)==4
assert all(abs(float(x.findtext('motorConstant'))-p.K_THRUST)<1e-12 for x in prop)
assert all(float(x.findtext('motorConstant'))==0 for x in wheel)
assert robot.findtext('gazebo/self_collide')=='true'
result={'mass_kg':mass,'independent_propellers':len(prop),'wheel_drives':len(wheel),'actuator_peak_torque_Nm':.33,
        'wheel_max_speed_rad_s':20,'flight_thrust_coefficient':p.K_THRUST,'self_collision':True,'passed':True}
open('hardware-results/independent-model-check.json','w').write(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
