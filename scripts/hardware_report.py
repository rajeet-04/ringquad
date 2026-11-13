"""Summarize reproducible simulation evidence; never converts a pass to hardware qualification."""
import csv
import gzip
import hashlib
import json
import math
from pathlib import Path

ROOT=Path('hardware-results')


def loads(path):
    stats={}
    invalid=0
    with (gzip.open(path,'rt') if path.suffix=='.gz' else path.open()) as file:
        for row in csv.DictReader(file):
            if 'base_z' not in row:continue
            if any(row.get(key) in (None,'') for key in ('base_z','fold','steer','torque_x','torque_y','torque_z','force_x','force_y','force_z')):
                invalid+=1
                continue
            z=float(row['base_z']);fold=float(row['fold'])
            phase=('airborne_locked' if z>.5 and abs(fold)<math.radians(5) else
                   'folded_ground' if z<.4 and abs(fold-math.radians(80))<math.radians(.5) else
                   'folding' if math.radians(5)<fold<math.radians(75) else 'other')
            s=stats.setdefault(phase,{'samples':0,'axis_moment_peak_Nm':0.,'axis_moment_sum_sq':0.,'bending_peak_Nm':0.,'force_peak_N':0.,'joint_angle_peak_deg':0.})
            t=float(row['torque_y']);b=math.hypot(float(row['torque_x']),float(row['torque_z']))
            force=math.sqrt(sum(float(row['force_'+a])**2 for a in 'xyz'))
            s['samples']+=1;s['axis_moment_peak_Nm']=max(s['axis_moment_peak_Nm'],abs(t));s['axis_moment_sum_sq']+=t*t
            s['bending_peak_Nm']=max(s['bending_peak_Nm'],b);s['force_peak_N']=max(s['force_peak_N'],force)
            s['joint_angle_peak_deg']=max(s['joint_angle_peak_deg'],abs(math.degrees(float(row['steer']))))
    for s in stats.values():s['axis_moment_rms_Nm']=math.sqrt(s.pop('axis_moment_sum_sq')/s['samples'])
    stats['_invalid_rows_skipped']=invalid
    return stats


def main():
    flight=json.loads((ROOT/'flight-sweep.json').read_text())
    matrix=json.loads((ROOT/'gazebo/matrix.json').read_text())
    physical=ROOT/'physical-candidate-mission'
    if (physical/'run-1.log').exists():
        text=(physical/'run-1.log').read_text()
        matrix.append({'case':'physical_rim_candidate','settings':{'ring_offset':'.075','steering_limit_deg':'25','self_collision':'true','tread_collision':'cylinder'},
                       'passed':'mission complete in' in text,'returncode':0 if 'mission complete in' in text else 1,
                       'folder_override':str(physical),'load_file':str(ROOT/'physical-candidate-loads.csv')})
    cases={}
    for name in flight['cases']:
        rows=[r for r in flight['runs'] if r['case']==name]
        cases[name]={'passed':sum(r['passed'] for r in rows),'total':len(rows),'crashes':sum(r['crash'] for r in rows),
                     'worst_final_error_m':max(r['position_error_m'] for r in rows),'worst_tilt_deg':max(r['peak_tilt_deg'] for r in rows)}
    measurements={}
    for case in matrix:
        name=case['case'];folder=Path(case.get('folder_override',str(ROOT/'gazebo'/name)))
        text=(folder/'run-1.log').read_text()
        import re
        case['completion']=re.findall(r'mission complete in .*',text)
        case['abort']=re.findall(r'mission aborted: .*',text)
        case['drive']=re.findall(r'drive done, worst cross-track .*',text)
        case['status']='mission_complete' if case['passed'] else 'mission_abort' if case['abort'] else 'incomplete_or_infrastructure_failure'
        goal=re.findall(r'(\d+) cm from the goal pad centre',text)
        cross=re.findall(r'worst cross-track ([\d.]+) cm',text)
        case['goal_error_cm']=int(goal[-1]) if goal else None
        case['worst_cross_track_cm']=float(cross[-1]) if cross else None
        case['meets_hardware_accuracy_screen']=bool(case['passed'] and goal and int(goal[-1])<=10 and cross and float(cross[-1])<=5)
        raw=Path(case.get('load_file',str(folder/'joint-loads.csv')))
        if not raw.exists():raw=Path(str(raw)+'.gz')
        if raw.exists():measurements[name]=loads(raw)
    geometry=json.loads((ROOT/'geometry.json').read_text())
    dense=json.loads((ROOT/'candidate-dense-geometry.json').read_text())
    files=['src/ringquad_description/urdf/ringquad.urdf.xacro','src/ringquad_description/plugins/SteeringLock.cc','src/ringquad_control/ringquad_control/plant.py','src/ringquad_control/ringquad_control/mixer.py']
    output={'hardware_qualified':False,'reason':'default ring/chassis interference; full-width candidate fails ground mission; no material, thermal, finite-lock-strength or real-motor calibration qualification',
            'default_intersecting_samples':len(geometry['chassis_intersections']), 'candidate_geometry':dense,'physical_75mm_geometry':json.loads((ROOT/'physical-rim-75mm.json').read_text()),'physical_60mm_geometry':json.loads((ROOT/'physical-rim-60mm.json').read_text()),
            'flight_cases':cases,'gazebo_cases':matrix,'joint_transmitted_loads':measurements,
            'source_sha256':{f:hashlib.sha256(Path(f).read_bytes()).hexdigest() for f in files},
            'load_semantics':'joint-frame transmitted wrench, not motor current or a measured material strength; ideal rigid constraint peaks are stiffness/time-step dependent'}
    (ROOT/'summary.json').write_text(json.dumps(output,indent=2))
    print(json.dumps({'gazebo':{r['case']:r['status'] for r in matrix},'flight_passed':sum(r['passed'] for r in flight['runs']),'flight_runs':len(flight['runs'])},indent=2))
    # Preserve raw evidence compressed; excludes interrupted-attempt provenance from summaries.
    for path in ROOT.rglob('*.csv'):
        with path.open('rb') as source,gzip.open(str(path)+'.gz','wb',compresslevel=6) as target:
            while chunk:=source.read(1024*1024):target.write(chunk)
        path.unlink()


if __name__=='__main__':main()
