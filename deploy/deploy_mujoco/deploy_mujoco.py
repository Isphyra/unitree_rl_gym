import time
import mujoco.viewer
import mujoco
import numpy as np
from legged_gym import LEGGED_GYM_ROOT_DIR
import torch
import yaml
import keyboard
import pygame
import argparse

def get_gravity_orientation(quaternion):
    qw = quaternion[0]
    qx = quaternion[1]
    qy = quaternion[2]
    qz = quaternion[3]

    gravity_orientation = np.zeros(3)
    gravity_orientation[0] = 2 * (-qz * qx + qw * qy)
    gravity_orientation[1] = -2 * (qz * qy + qw * qx)
    gravity_orientation[2] = 1 - 2 * (qw * qw + qz * qz)
    return gravity_orientation

def pd_control(target_q, q, kp, target_dq, dq, kd):
    """Calculates torques from position commands"""
    return (target_q - q) * kp + (target_dq - dq) * kd

if __name__ == "__main__":
    # --- XBOX CONTROLLER SETUP ---
    pygame.init()
    pygame.joystick.init()
    joystick = None
    if pygame.joystick.get_count() > 0:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print(f"Controller verbunden: {joystick.get_name()}")
        print("Steuerung: Sticks zum Laufen.")
        print("Extras: 'A' zum leichten Schubsen, 'Back' zum Resetten.")
    else:
        print("Kein Controller! Nutze WASD.")
        print("Extras: 'Leertaste' zum leichten Schubsen, 'R' zum Resetten.")
    # -----------------------------

    parser = argparse.ArgumentParser()
    parser.add_argument("config_file", type=str, help="config file name in the config folder")
    args = parser.parse_args()
    config_file = args.config_file
    
    with open(f"{LEGGED_GYM_ROOT_DIR}/deploy/deploy_mujoco/configs/{config_file}", "r") as f:
        config = yaml.load(f, Loader=yaml.FullLoader)
        policy_path = config["policy_path"].replace("{LEGGED_GYM_ROOT_DIR}", LEGGED_GYM_ROOT_DIR)
        xml_path = config["xml_path"].replace("{LEGGED_GYM_ROOT_DIR}", LEGGED_GYM_ROOT_DIR)

        simulation_duration = config["simulation_duration"]
        simulation_dt = config["simulation_dt"]
        control_decimation = config["control_decimation"]

        kps = np.array(config["kps"], dtype=np.float32)
        kds = np.array(config["kds"], dtype=np.float32)
        default_angles = np.array(config["default_angles"], dtype=np.float32)

        ang_vel_scale = config["ang_vel_scale"]
        dof_pos_scale = config["dof_pos_scale"]
        dof_vel_scale = config["dof_vel_scale"]
        action_scale = config["action_scale"]
        cmd_scale = np.array(config["cmd_scale"], dtype=np.float32)
        num_actions = config["num_actions"]
        num_obs = config["num_obs"]
        cmd = np.array(config["cmd_init"], dtype=np.float32)

    action = np.zeros(num_actions, dtype=np.float32)
    target_dof_pos = default_angles.copy()
    obs = np.zeros(num_obs, dtype=np.float32)
    counter = 0

    m = mujoco.MjModel.from_xml_path(xml_path)
    d = mujoco.MjData(m)
    m.opt.timestep = simulation_dt

    # Speichern der Startposition für den Reset
    mujoco.mj_forward(m, d)
    init_qpos = d.qpos.copy()
    init_qvel = d.qvel.copy()

    policy = torch.jit.load(policy_path)

    with mujoco.viewer.launch_passive(m, d) as viewer:
        start = time.time()
        
        while viewer.is_running(): 
            step_start = time.time()
            
            # --- FEATURE 1: RESET (Wiederaufstehen) ---
            should_reset = False
            if keyboard.is_pressed('r'): should_reset = True
            if joystick and joystick.get_button(6): should_reset = True # Back Button

            if should_reset:
                d.qpos[:] = init_qpos
                d.qvel[:] = init_qvel
                d.ctrl[:] = 0.0
                mujoco.mj_forward(m, d)
                print("Reset ausgeführt!")
                time.sleep(0.2) 
            # ------------------------------------------

            # --- FEATURE 2: SCHUBSEN (Balance testen) ---
            should_push = False
            if keyboard.is_pressed('space'): should_push = True
            if joystick and joystick.get_button(0): should_push = True # A Button

            if should_push:
                push_dir = np.random.uniform(-1, 1, 2) 
                
                # HIER GEÄNDERT: 0.5 ist deutlich sanfter (vorher 1.5)
                push_force = 0.5 
                
                d.qvel[0] += push_dir[0] * push_force 
                d.qvel[1] += push_dir[1] * push_force 
                print(f"Leicht geschubst! Richtung: {push_dir}")
                time.sleep(0.2) 
            # --------------------------------------------

            tau = pd_control(target_dof_pos, d.qpos[7:], kps, np.zeros_like(kds), d.qvel[6:], kds)
            d.ctrl[:] = tau
            mujoco.mj_step(m, d)

            counter += 1
            if counter % control_decimation == 0:
                vx = 0.0
                vy = 0.0
                dyaw = 0.0

                if joystick:
                    pygame.event.pump() 
                    raw_vx = -joystick.get_axis(1) 
                    raw_vy = -joystick.get_axis(0) 
                    raw_dyaw = -joystick.get_axis(3) 
                    def deadzone(val, threshold=0.1): return val if abs(val) > threshold else 0.0
                    vx = deadzone(raw_vx) * 0.8
                    vy = deadzone(raw_vy) * 0.6
                    dyaw = deadzone(raw_dyaw) * 1.0

                if keyboard.is_pressed('w'): vx = 0.6
                if keyboard.is_pressed('s'): vx = -0.4
                if keyboard.is_pressed('a'): vy = 0.4
                if keyboard.is_pressed('d'): vy = -0.4
                if keyboard.is_pressed('q'): dyaw = 0.5
                if keyboard.is_pressed('e'): dyaw = -0.5

                cmd[0] = vx
                cmd[1] = vy
                cmd[2] = dyaw

                qj = d.qpos[7:]
                dqj = d.qvel[6:]
                quat = d.qpos[3:7]
                omega = d.qvel[3:6]

                qj = (qj - default_angles) * dof_pos_scale
                dqj = dqj * dof_vel_scale
                gravity_orientation = get_gravity_orientation(quat)
                omega = omega * ang_vel_scale

                period = 0.8
                count = counter * simulation_dt
                phase = count % period / period
                sin_phase = np.sin(2 * np.pi * phase)
                cos_phase = np.cos(2 * np.pi * phase)

                obs[:3] = omega
                obs[3:6] = gravity_orientation
                obs[6:9] = cmd * cmd_scale
                obs[9 : 9 + num_actions] = qj
                obs[9 + num_actions : 9 + 2 * num_actions] = dqj
                obs[9 + 2 * num_actions : 9 + 3 * num_actions] = action
                obs[9 + 3 * num_actions : 9 + 3 * num_actions + 2] = np.array([sin_phase, cos_phase])
                
                obs_tensor = torch.from_numpy(obs).unsqueeze(0)
                action = policy(obs_tensor).detach().numpy().squeeze()
                target_dof_pos = action * action_scale + default_angles

            viewer.sync()

            time_until_next_step = m.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)