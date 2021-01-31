# ==============================================================================
# -- Find carla module ---------------------------------------------------------
# ==============================================================================
import glob
import os
import sys

try:
    sys.path.append(glob.glob('./CARLA/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

# ==============================================================================
# -- Impoer lib ----------------------------------------------------------------
# ==============================================================================
import argparse

import gym
from gym import spaces

from stable_baselines.common.vec_env import DummyVecEnv
from stable_baselines.common.policies import MlpPolicy
from stable_baselines import PPO2
from stable_baselines import A2C
from stable_baselines import DQN

import numpy as np
import carla
import random
import time
import pygame
import math


# ==============================================================================
# -- Define CARLA world --------------------------------------------------------
# ==============================================================================
class CarlaEnv(object):
    def __init__(self, args):
        client = carla.Client(args.host, args.port)
        client.set_timeout(2.0)
        self.world = client.get_world()
        self.map = self.world.get_map()
        self.tick = self.world.tick()
        self.agent = None
        self.lead = None

        settings = self.world.get_settings()
        #settings.no_rendering_mode = True
        #settings.synchronous_mode = True
        #settings.fixed_delta_seconds = (1.0/20.0) # 20 Hz
        #settings.fixed_delta_seconds = (1.0/40.0) # 40 Hz
        settings.fixed_delta_seconds = (1.0/60.0) # 60 Hz
        #settings.fixed_delta_seconds = None # Realtime
        self.world.apply_settings(settings)

    def reset(self):
        self.destroy()

        # Find vehicle blueprint
        blueprint = random.choice(self.world.get_blueprint_library().filter('vehicle.toyota.prius'))
        
        # Spwan agent vehicle
        if blueprint.has_attribute('color'):
            blueprint.set_attribute('color', '0, 255, 0') # Green
        spawn_point = carla.Transform()
        spawn_point.location.x = 15.0
        spawn_point.location.y = 47.1139772697
        spawn_point.location.z = 0.1
        spawn_point.rotation.yaw = 0.0
        self.agent = self.world.try_spawn_actor(blueprint, spawn_point)

        # Spwan lead vehicle
        if blueprint.has_attribute('color'):
            blueprint.set_attribute('color', '255, 255, 255') # White
        spawn_point = carla.Transform()
        spawn_point.location.x = 35.0
        spawn_point.location.y = 47.1139772697
        spawn_point.location.z = 0.1
        spawn_point.rotation.yaw = 0.0
        self.lead = self.world.try_spawn_actor(blueprint, spawn_point)
        if(self.lead is not None):
            self.lead.set_autopilot(False)
        time.sleep(0.05)
        self.lead.set_autopilot(True)

    def destroy(self):
        if(self.agent is not None):
            self.agent.destroy()
        if(self.lead is not None):
            self.lead.destroy()

    def get_distance(self):
        if((self.agent is not None) and (self.lead is not None)):
            return self.agent.get_location().distance(self.lead.get_location())
        else:
            return 0.0


# ==============================================================================
# -- Define Gym Environment ----------------------------------------------------
# ==============================================================================
class AccEnv(gym.Env):
    """
    Custom Environment that follows gym interface"
  
    Observation: Type: Box(3)
        No. Observation                 Min     Max
        0	Gap Distance                -Inf    Inf
        1	Ego Vehicle Velocity        -Inf    Inf
        2	Ego Vehicle Accelerations   -Inf    Inf
    
    Actions: Type: Box(1)
        No. Action
        0   Throttle Value (-1, 1)
        

    Reward:
        No. Reward
        1   +1 Every step that the error gap distance less than +-1 meter from target distance (20 meters)
        2   -0.5 Every step that Accleration is more than 1G ()Not implement yet)
    """

    metadata = {'render.modes': ['human']}

    # Initial environment
    def __init__(self, world):
        super(AccEnv, self).__init__()

        # Define observation space
        space_low = np.array([-10.0,            # Gap Distance -5.0 meter
                                0.0,            # Ego Vehicle Velocity 0.0 m/s
                                0.0])           # Ego Vehicle Accelerations 0.0
        space_high = np.array([10.0,            # Gap Distance 5.0 meter
                                (120.0/3.6),    # Ego Vehicle Velocity (120.0/3.6) m/s
                                8*9.8])          # Ego Vehicle Accelerations 8*9.8 m/s^2
        self.observation_space = spaces.Box(low=space_low, high=space_high, dtype=np.float32)

        action_low = np.array([-1.0])   # -1.0
        action_high = np.array([1.0])   # 1.0
        # Define action space
        self.action_space = spaces.Box(low=action_low, high=action_high, dtype=np.float32)

        self.state = np.zeros(3)

        #pygame.init()
        #self.clock = pygame.time.Clock()

        self.target_distance = 20.0

        self.world = world

        self.reward = 0.0
        self.reward_cumm = 0.0
        self.travelling_dist = 0.0
        self.acc_cumm = 0.0
        self.error_dist_cumm = 0.0
        self.reset()
        self.agent_current_location = self.world.agent.get_location()
        self.agent_previous_location = self.agent_current_location

    # Reset environment
    def reset(self):
        self.world.reset()

        self.agent_current_location = self.world.agent.get_location()
        self.agent_previous_location = self.agent_current_location

        # Compute observation state
        state = self.state
        distance_error = self.target_distance - self.world.get_distance()
        state[0] = distance_error
        v = self.world.agent.get_velocity()
        state[1] = math.sqrt(v.x**2 + v.y**2)
        a = self.world.agent.get_acceleration()
        state[2] = math.sqrt(a.x**2 + a.y**2)

        return state

    def step(self, action):
        #self.world.tick # tick the CARLA world
        #self.clock.tick_busy_loop(30)  # For limit cliant frame rate        

        # Auto steering control
        car_front = self.world.agent.get_transform().transform(carla.Location(x=1.42))
        x_0, y_0 = [car_front.x, car_front.y]

        waypoint = self.world.map.get_waypoint(carla.Location(car_front))
        x_1, y_1 = [waypoint.next(0.01)[0].transform.location.x, waypoint.next(0.01)[0].transform.location.y]
        x_2, y_2 = [waypoint.next(0.1)[0].transform.location.x, waypoint.next(0.1)[0].transform.location.y]

        cross_track_error = (((y_2-y_1)*x_0 - (x_2-x_1)*y_0 + x_2*y_1 - y_2*x_1)
                                / np.sqrt(np.power(y_2-y_1, 2) + np.power(x_2-x_1, 2)))

        k = 1.0
        k_s = 1e-100
        v = self.world.agent.get_velocity()
        vv = math.sqrt(v.x**2 + v.y**2 + v.z**2)
        cross_track_steering = np.arctan2((k*cross_track_error), (k_s + vv))

        # Calculate heading error
        heading_error = self._angle_diff(np.arctan2((y_2-y_1), (x_2-x_1)),
                                        np.radians(self.world.agent.get_transform().rotation.yaw))

        # Change the steer output with the lateral controller.
        steer_output_rad = heading_error + cross_track_steering

        steer_output_deg = np.degrees(steer_output_rad)
        steer_output = min(70.0, max(-70.0, steer_output_deg))
        steer_output = steer_output / 70.0

        # Take action
        assert self.action_space.contains(action), "%r (%s) invalid"%(action, type(action))

        control = carla.VehicleControl()
        if(action[0] >= 0.0):
            control.throttle = action[0].item()
            control.brake = 0.0
        else:
            control.throttle = 0.0
            control.brake = -action[0].item()
        control.steer = steer_output
        self.world.agent.apply_control(control)

        # Compute observation state
        state = self.state
        distance_error = self.target_distance - self.world.get_distance()
        state[0] = distance_error
        v = self.world.agent.get_velocity()
        state[1] = math.sqrt(v.x**2 + v.y**2)
        a = self.world.agent.get_acceleration()
        state[2] = math.sqrt(a.x**2 + a.y**2)

        speed_limit = (self.world.agent.get_speed_limit() + 10.0)/3.6  # m/s
        acc_limit = 4.0*9.8 # 4G

        # Comppute done
        if(np.abs(distance_error) >= 5.0):
            done = True
            print("Reward(%f = %f + %f + %f), Done by distance error(%f) >= (5.0)" % (self.reward_cumm, self.travelling_dist, self.acc_cumm, self.error_dist_cumm, np.abs(distance_error)))
        elif(state[1] >= speed_limit):
            done = True
            print("Reward(%f = %f + %f + %f), Done by speed(%f) >= speedlimit(%f)" % (self.reward_cumm, self.travelling_dist, self.acc_cumm, self.error_dist_cumm, state[1], speed_limit))
        elif(state[2] >= acc_limit):
            done = True
            print("Reward(%f = %f + %f + %f), Done by acc(%f) >= 4G(%f)" % (self.reward_cumm, self.travelling_dist, self.acc_cumm, self.error_dist_cumm, state[2], acc_limit))
        else:
            done = False

        # Comppute reward
        self.agent_current_location = self.world.agent.get_location()
        if not done:
            self.reward = self.agent_current_location.distance(self.agent_previous_location) - (0.0001*state[1]*state[2]) - (0.001*np.abs(distance_error))
            #self.reward = self.agent_current_location.distance(self.agent_previous_location) - (0.0001*state[1]*state[2])
            #self.reward = self.agent_current_location.distance(self.agent_previous_location)
            self.reward_cumm += self.reward
            self.travelling_dist += self.agent_current_location.distance(self.agent_previous_location)
            self.acc_cumm -= (0.0001*state[1]*state[2])
            self.error_dist_cumm -= (0.001*np.abs(distance_error))
        else:
            self.reward = 0.0
            self.reward_cumm = 0.0
            self.travelling_dist = 0.0
            self.acc_cumm = 0.0
            self.error_dist_cumm = 0.0
        self.agent_previous_location = self.agent_current_location
        return state, self.reward, done, {}

    def render(self, mode='human', close=False):
        print("TODO render")

    @staticmethod
    def _angle_diff(angle1, angle2):
        angle_diff = angle1 - angle2
        if(angle_diff > np.pi):
            angle_diff -= 2.0*np.pi
        elif(angle_diff < -np.pi):
            angle_diff += 2.0*np.pi
        return (angle_diff)


# ==============================================================================
# -- execute_loop() ------------------------------------------------------------
# ==============================================================================
def execute_loop(args):
    try:
        world = CarlaEnv(args)

        # Instantiate and wrap the env
        env = DummyVecEnv([lambda: AccEnv(world)])

        TOTAL_TIMESTEPS = 1000000
        LOOP_START = 73 # Select loop to continue train
        LOOP_END = 100
        
        for loop in range(LOOP_START, LOOP_END, 1):

            try:
                model = PPO2.load("./final_acc/model/PPO2_%dM" % (loop), env=env, learning_rate=0.00001, verbose=0, tensorboard_log="./final_acc/tensorboard")
            except:
                print("############################################## Cannot load model ##############################################")
                model = PPO2('MlpPolicy', env=env, learning_rate=0.00001, verbose=0, tensorboard_log="./final_acc/tensorboard")
                model.save("./final_acc/model/PPO2_%dM" % (loop))

            print("#################### Learn loop=%dM ####################" % (loop))
            model.learn(total_timesteps=TOTAL_TIMESTEPS, tb_log_name=("PPO2_%dM" % loop))

            # Save train result
            model.save("./final_acc/model/PPO2_%dM" % (loop + 1))

    finally:
        model.save("./final_acc/model/PPO2_%dM-unfinished" % (loop))
        world.destroy()


# ==============================================================================
# -- Main ----------------------------------------------------------------------
# ==============================================================================
def main():
    argparser = argparse.ArgumentParser(
        description='ACC RL')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    args = argparser.parse_args()
    
    try:
        execute_loop(args)
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')

if __name__ == '__main__':
    main()
