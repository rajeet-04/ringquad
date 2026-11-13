"""Fit measured thrust maps and enforce commissioning data provenance.
CSV: rotor,omega_rad_s,thrust_N,current_A,voltage_V,temperature_C.
Metadata: source,operator,instrument_ids,vehicle_mass_kg,centre_of_mass_m.
Synthetic fixtures can verify software but can never establish commissioning.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import numpy as np


def evaluate(path, metadata):
    rows=list(csv.DictReader(Path(path).open()))
    groups={n:[] for n in ['fl','fr','rl','rr']}
    for row in rows:
        if row['rotor'] not in groups:raise ValueError('unknown rotor')
        v={k:float(row[k]) for k in ['omega_rad_s','thrust_N','current_A','voltage_V','temperature_C']}
        if not all(np.isfinite(x) for x in v.values()):raise ValueError('non-finite measurement')
        if v['omega_rad_s']<=0 or v['thrust_N']<=0:raise ValueError('nonpositive thrust/speed')
        if v['current_A']<0 or v['current_A']>32 or v['temperature_C']>75 or v['voltage_V']<13.2:
            raise ValueError('electrical or temperature envelope violated')
        groups[row['rotor']].append(v)
    fits={}
    for n,values in groups.items():
        if len(values)<8:raise ValueError('at least eight steady points per rotor required')
        w=np.array([r['omega_rad_s'] for r in values]);f=np.array([r['thrust_N'] for r in values]);x=w*w
        if min(w)>500 or max(w)<1035:raise ValueError('insufficient hover-to-maximum speed coverage')
        k=float(x@f/(x@x));residual=float(np.sqrt(np.mean((f-k*x)**2))/max(f))
        if not 5e-6<k<1.5e-5 or residual>.08:raise ValueError('invalid or poorly fitted thrust map')
        fits[n]={'k_thrust':k,'relative_rms_residual':residual,'samples':len(values)}
    mass=float(metadata['vehicle_mass_kg']);com=np.asarray(metadata['centre_of_mass_m'],float)
    if not np.isfinite(mass) or not 1.<mass<3. or com.shape!=(3,) or not np.all(np.isfinite(com)):
        raise ValueError('invalid measured mass/CG')
    measured=(metadata.get('source')=='bench' and bool(metadata.get('operator')) and bool(metadata.get('instrument_ids')))
    return {'fit_passed':True,'bench_measurements_declared':measured,'physical_release_ready':False,
            'unclosed_gates':['structural proof','installed thermal run','floor/contact test','rotor containment test'],
            'data_sha256':hashlib.sha256(Path(path).read_bytes()).hexdigest(),'fits':fits,
            'controller_parameters':{'vehicle_mass':mass,'centre_of_mass':com.tolist(),'thrust_coefficients':[fits[n]['k_thrust'] for n in groups]}}


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('csv');parser.add_argument('metadata');parser.add_argument('--output',required=True);a=parser.parse_args()
    result=evaluate(a.csv,json.loads(Path(a.metadata).read_text()))
    Path(a.output).write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
