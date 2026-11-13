"""Parametric provisional 7075-T651 carrier, standoff and double-shear latch.
Exports STEP/STL and exact CAD volumes. Dimensions in millimetres.
"""
import json
from pathlib import Path
import cadquery as cq

out=Path('hardware-blockers/cad');out.mkdir(parents=True,exist_ok=True)
# 17 OD / 10 ID standoff; 100 mm span, independent of motor shaft.
tube=cq.Workplane('XY').circle(8.5).circle(5).extrude(100)
# Three hollow 16x18x1 mm ribs behind the propeller plane; hub clears motor.
hub=cq.Workplane('XY').circle(25).circle(14).extrude(3.5).translate((0,0,-31.75))
carrier=hub
for angle in (0,120,240):
 rib=cq.Workplane('YZ').rect(16,18).rect(14,16).extrude(75).translate((20,0,-30))
 rib=rib.rotate((0,0,0),(0,0,1),angle)
 carrier=carrier.union(rib)
# Bolt pattern is provisional and must match selected motor plate.
for x,y in [(19,0),(-19,0),(0,19),(0,-19)]:
 carrier=carrier.cut(cq.Workplane('XY').center(x,y).circle(1.6).extrude(60).translate((0,0,-60)))
# Double-shear ears: 4 mm pin, two 3 mm ears, 12 mm lever arm.
ears=cq.Workplane('XY')
for z in (-6,3):
 lug=cq.Workplane('XY').box(18,14,3,centered=(False,True,False)).translate((0,0,z))
 ears=ears.union(lug)
ears=ears.union(cq.Workplane('XY').box(4,14,12,centered=(False,True,True)))
pin=cq.Workplane('XY').circle(2).extrude(14).translate((12,0,-7))
tongue=cq.Workplane('XY').box(19,8,5,centered=(False,True,True)).translate((5,0,0))
bore=cq.Workplane('XY').center(12,0).circle(2.01).extrude(20).translate((0,0,-10))
ears=ears.cut(bore);tongue=tongue.cut(bore)
assert ears.intersect(pin).val().Volume()<1e-6
assert tongue.intersect(pin).val().Volume()<1e-6
race=cq.Workplane('XZ').polyline([(101,-22.5),(103,-24.5),(105,-22.5),(103,-20.5)]).close().revolve(360,(0,0),(0,1))
roller=cq.Workplane('XZ').polyline([(8,-5),(13,-5),(13,-2),(11,0),(13,2),(13,5),(8,5)]).close().revolve(360,(0,0),(0,1))
parts={'latch_tongue':tongue,'rear_v_race':race,'v_roller':roller,'standoff':tube,'carrier':carrier,'latch_ears':ears,'latch_pin':pin}
rows={}
for name,part in parts.items():
 cq.exporters.export(part,str(out/f'{name}.step'))
 cq.exporters.export(part,str(out/f'{name}.stl'))
 shape=part.val();box=shape.BoundingBox()
 density=.00785 if name=='latch_pin' else .0016 if name=='rear_v_race' else .0014 if name=='v_roller' else .00281
 rows[name]={'volume_mm3':shape.Volume(),'mass_g':shape.Volume()*density,'density_g_cm3':density*1000,'bounds_mm':[box.xlen,box.ylen,box.zlen],'valid':shape.isValid()}
 assert shape.isValid()
(out/'volumes.json').write_text(json.dumps(rows,indent=2))
print(json.dumps(rows,indent=2))
