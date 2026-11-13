"""Provisional electrical envelope and lumped thermal screens; no measured calibration."""
import json, math
from pathlib import Path

def current_limit(open_voltage, resistance=.017):
    return max(0., min(90., (open_voltage-13.2)/resistance))

def motor_temperature(ambient, heat, resistance, capacity, duration):
    return ambient+heat*resistance*(1-math.exp(-duration/(resistance*capacity)))

def evaluate():
    rows={
      'measurement_source':'unmeasured design assumptions',
      'battery':'4S 3Ah; pack resistance <=15mOhm; mass <=330g; cell maker limits required',
      'current_limit_A':{str(v):current_limit(v) for v in [16.8,14.8,14.2,13.2]},
      'loaded_voltage_at_14_8V_90A':14.8-90*.017,
      'unlimited_128A_voltage':14.8-128*.017,
      'motor_180s_C_at_35C':motor_temperature(35,10,4,20,180),
      'motor_180s_C_at_45C':motor_temperature(45,10,4,20,180),
      'motor_180s_C_if_thermal_resistance_doubles':motor_temperature(35,10,8,20,180),
      'battery_180s_C_at_40A_35C_adiabatic':35+40**2*.015*180/330,
      'battery_capacity_used_Ah':40*180/3600,
      'rotor_limit_rad_s':1090.,'rotor_speed_rpm':1090*60/(2*math.pi),
      'APC_9in_MR_rpm_limit':105000/9,
      'thermal_parameters':{'motor_heat_W':10,'motor_R_K_W':4,'motor_C_J_K':20,'battery_C_J_K':330},
      'hardware_enforcement_required':['pack current and voltage sensing; ESC current, RPM and temperature telemetry',
        'limit aggregate current using measured pack resistance; land before voltage headroom is exhausted',
        '75C motor case trip and 55C battery trip subject to component maker limits',
        '6V servo BEC >=4A continuous/10A peak; separate 12V wheel/fold rail >=15A peak'],
      'coupled_electrical_flight_simulation_passed':False,
      'physical_thermal_test_passed':False}
    assert rows['loaded_voltage_at_14_8V_90A']>=13.2
    assert rows['unlimited_128A_voltage']<13.2
    assert rows['motor_180s_C_at_35C']<75<rows['motor_180s_C_at_45C']
    assert rows['motor_180s_C_if_thermal_resistance_doubles']>75
    assert rows['rotor_speed_rpm']<rows['APC_9in_MR_rpm_limit']
    return rows
if __name__=='__main__':
    result=evaluate();Path('hardware-blockers/results/power-thermal.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
