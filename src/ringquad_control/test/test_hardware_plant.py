import numpy as np
from ringquad_control import params as p, mixer
from ringquad_control.plant import Plant


def test_trimmed_hover_balances_shifted_com_in_dynamic_plant():
    com=np.array([.005,-.003,0.])
    plant=Plant(centre_of_mass=com)
    forces=mixer.wrench_to_thrusts(p.MASS*p.GRAVITY,np.zeros(3),centre_of_mass=com)
    commands=mixer.thrusts_to_commands(forces)
    plant.joint_vel=p.ROTOR_DIR*commands
    for _ in range(100):plant.step(commands)
    assert np.linalg.norm(plant.rate)<1e-9
    assert np.linalg.norm(plant.vel)<1e-9


def test_uniform_thrust_loss_produces_expected_vertical_acceleration():
    plant=Plant(thrust_scale=np.full(4,.8))
    commands=abs(plant.joint_vel)
    plant.step(commands)
    assert np.isclose(plant.vel[2],-.2*p.GRAVITY*plant.dt)


def test_single_motor_thrust_loss_produces_correct_roll_pitch_signs():
    plant=Plant(thrust_scale=np.array([.8,1.,1.,1.]))
    plant.step(abs(plant.joint_vel))
    assert plant.rate[0]<0 and plant.rate[1]>0
