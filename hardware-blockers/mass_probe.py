import sys
sys.path.insert(0,'src/ringquad_control');sys.path.insert(0,'scripts')
from ringquad_control import params as p
p.MASS=2.428;p.INERTIA*=1.1
import hardware_flight_sweep as h
import separate_drive_flight_sweep as s
from concurrent.futures import ProcessPoolExecutor
import json
if __name__=='__main__':
 tasks=[(n,i) for n in ['nominal','thrust_minus20pct','one_motor_thrust_minus20pct','combined_adverse'] for i in range(3)]
 with ProcessPoolExecutor(max_workers=4) as pool:rows=list(pool.map(s.evaluate,tasks))
 open('hardware-blockers/results/mass-probe.json','w').write(json.dumps(rows,indent=2))
 for r in rows:print(r['case'],r['seed'],r['passed'],r['crash'],round(r['position_error_m'],3))
