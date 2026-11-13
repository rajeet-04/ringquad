"""Software fixtures only. They cannot qualify a physical vehicle."""
import csv, json
from pathlib import Path
import pytest
from calibration import evaluate
from power_thermal import current_limit, motor_temperature

@pytest.fixture
def measurements(tmp_path):
    path=tmp_path/'synthetic.csv'
    rows=[]
    for i,rotor in enumerate(('fl','fr','rl','rr')):
        for w in (400,500,600,700,800,900,1000,1040):
            rows.append(dict(rotor=rotor,omega_rad_s=w,thrust_N=(8.4e-6*(1-.05*i))*w*w,current_A=10,voltage_V=14.8,temperature_C=45))
    def write():
        with path.open('w') as out:
            writer=csv.DictWriter(out,fieldnames=rows[0].keys());writer.writeheader();writer.writerows(rows)
    write()
    return path,rows,write,dict(source='synthetic',vehicle_mass_kg=2.65,centre_of_mass_m=[0,0,0])

def test_synthetic_fit_cannot_release(measurements):
    path,rows,write,meta=measurements;r=evaluate(path,meta)
    assert r['controller_parameters']['thrust_coefficients']==pytest.approx([8.4e-6,7.98e-6,7.56e-6,7.14e-6])
    assert not r['bench_measurements_declared'] and not r['physical_release_ready']

@pytest.mark.parametrize('field,value',[('temperature_C',80),('current_A',33),('voltage_V',12),('omega_rad_s',float('nan'))])
def test_reject_unsafe_measurements(measurements,field,value):
    path,rows,write,meta=measurements;rows[0][field]=value;write()
    with pytest.raises(ValueError):evaluate(path,meta)

def test_missing_rotor_rejected(measurements):
    path,rows,write,meta=measurements;rows[:]=[r for r in rows if r['rotor']!='rr'];write()
    with pytest.raises(ValueError):evaluate(path,meta)

def test_power_derating():
    assert current_limit(14.8)==90
    assert current_limit(14.2)==pytest.approx(58.8235294)
    assert current_limit(13)==0
    assert motor_temperature(45,10,4,20,180)>75
