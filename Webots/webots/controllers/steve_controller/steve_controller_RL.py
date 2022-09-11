"""steve_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot,Motor,PositionSensor
from math import pi,sin


#RL imports
from deepbots.supervisor.controllers.robot_supervisor import RobotSupervisor
from utilities import normalizeToRange, plotData
from PPO_agent import PPOAgent, Transition

import gzip
from gym.spaces import Box, Discrete
import numpy as np

PI = 3.141592653589793

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

Hip_joint_inital = [0,0,0,0,0,0,0,0]
leg_pos_initial = [0,0]
initial_pos = []
for i in Hip_joint_inital:
    initial_pos.append(i)
    for y in leg_pos_initial:
        initial_pos.append(y)

goal_threshold = 3
side_threshold = 2

##===================STEVE motor control ==============================================
class Joint:
    def __init__(self,position,parent_name):
        switcher = {
            0: "base_link_fixed_hip2",
            1: "hip2_hipjoint",
            2: "uknee_lknee",
        }
        pos = switcher.get(position, "nothing")
        self.actuator =robot.getDevice(str(parent_name+"_"+pos))
        self.position_sensor = robot.getDevice(str(parent_name+"_"+pos+"_sensor"))
        self.position_sensor.enable(timestep)
        
class Leg:
    def __init__(self,name):
        self.name = name
        self.top  = Joint(0,self.name)
        self.mid  = Joint(1,self.name)
        self.bottom  = Joint(2,self.name)
        self.touch_sensor = robot.getDevice(str("touch_sensor_"+name))
        self.touch_sensor.enable(timestep)
    


class Leg_controller:
    def __init__(self):
        self.leg_1R = Leg("leg1R")
        self.leg_2R = Leg("leg2R")
        self.leg_3R = Leg("leg3R")
        self.leg_4R = Leg("leg4R")
        self.leg_1L = Leg("leg1L")
        self.leg_2L = Leg("leg2L")
        self.leg_3L = Leg("leg3L")
        self.leg_4L = Leg("leg4L")
        

##=====================================================================================
##===================RL setup==========================================================

robot = Leg_controller()

class OpenAIGymEnvironment(Supervisor, gym.Env):
    def __init__(self, max_episode_steps=1000):
        super().__init__()

        # Open AI Gym generic
        self.action_space = Box(low=np.array([-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,]), 
				     high=np.array([1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,]), dtype=np.float32)
        self.observation_space = Box(low=np.array([-PI,-PI,-PI,-PI,-PI,-PI,-PI,-PI,-PI,-PI,-PI,-PI,-PI,-PI,-PI,-PI,-PI,-PI,-PI,-PI,-PI,-PI,-PI,-PI,0,0,0,0,0,0,0,0]),
                             high=np.array([PI,PI,PI,PI,PI,PI,PI,PI,PI,PI,PI,PI,PI,PI,PI,PI,PI,PI,PI,PI,PI,PI,PI,PI,1,1,1,1,1,1,1,1]),
                             dtype=np.float32)
        self.state = None
        self.spec = gym.envs.registration.EnvSpec(id='SteveEnv-v0', max_episode_steps=2000)

        # Environment specific
        self.__timestep = int(self.getBasicTimeStep())
        self.__actuators = []
        self.__sensors = None

    def move(self, positions):
       
	  robot.leg1R.top.actuator.setPosition(positions[0])
	  
	  robot.leg1R.mid.actuator.setPosition(positions[1])
	  robot.leg1R.bottom.actuator.setPosition(positions[2])
	  robot.leg2R.top.actuator.setPosition(positions[3])
	  robot.leg2R.mid.actuator.setPosition(positions[4])
	  robot.leg2R.bottom.actuator.setPosition(positions[5])
	  robot.leg3R.top.actuator.setPosition(positions[6])
	  robot.leg3R.mid.actuator.setPosition(positions[7])
	  robot.leg3R.bottom.actuator.setPosition(positions[8])
	  robot.leg4R.top.actuator.setPosition(positions[9])
	  robot.leg4R.mid.actuator.setPosition(positions[10])
	  robot.leg4R.bottom.actuator.setPosition(positions[11])

	  robot.leg1L.top.actuator.setPosition(positions[12])
	  robot.leg1L.mid.actuator.setPosition(positions[13])
	  robot.leg1L.bottom.actuator.setPosition(positions[14])
	  robot.leg2L.top.actuator.setPosition(positions[15])
	  robot.leg2L.mid.actuator.setPosition(positions[16])
	  robot.leg2L.bottom.actuator.setPosition(positions[17])
	  robot.leg3L.top.actuator.setPosition(positions[18])
	  robot.leg3L.mid.actuator.setPosition(positions[19])
	  robot.leg3L.bottom.actuator.setPosition(positions[20])
	  robot.leg4L.top.actuator.setPosition(positions[21])
	  robot.leg4L.mid.actuator.setPosition(positions[22])
	  robot.leg4L.bottom.actuator.setPosition(positions[23])

    def get_states(self):
        state= [] 
	
	  states.append(robot.leg1R.top.position_sensor.getValue())
	  states.append(robot.leg1R.mid.position_sensor.getValue())
	  states.append(robot.leg1R.bottom.position_sensor.getValue())
	  states.append(robot.leg2R.top.position_sensor.getValue())
	  states.append(robot.leg2R.mid.position_sensor.getValue())
	  states.append(robot.leg2R.bottom.position_sensor.getValue())
	  states.append(robot.leg3R.top.position_sensor.getValue())
	  states.append(robot.leg3R.mid.position_sensor.getValue())
	  states.append(robot.leg3R.bottom.position_sensor.getValue())
	  states.append(robot.leg4R.top.position_sensor.getValue())
	  states.append(robot.leg4R.mid.position_sensor.getValue())
	  states.append(robot.leg4R.bottom.position_sensor.getValue())
	  states.append(robot.leg1L.top.position_sensor.getValue())
	  states.append(robot.leg1L.mid.position_sensor.getValue())
	  states.append(robot.leg1L.bottom.position_sensor.getValue())
	  states.append(robot.leg2L.top.position_sensor.getValue())
	  states.append(robot.leg2L.mid.position_sensor.getValue())
	  states.append(robot.leg2L.bottom.position_sensor.getValue())
	  states.append(robot.leg3L.top.position_sensor.getValue())
	  states.append(robot.leg3L.mid.position_sensor.getValue())
	  states.append(robot.leg3L.bottom.position_sensor.getValue())
	  states.append(robot.leg4L.top.position_sensor.getValue())
	  states.append(robot.leg4L.mid.position_sensor.getValue())
	  states.append(robot.leg4L.bottom.position_sensor.getValue())

	  states.append(robot.leg1R.touch_sensor.getValue())
	  states.append(robot.leg2R.touch_sensor.getValue())
	  states.append(robot.leg3R.touch_sensor.getValue())
	  states.append(robot.leg4R.touch_sensor.getValue())
	  states.append(robot.leg1L.touch_sensor.getValue())
	  states.append(robot.leg2L.touch_sensor.getValue())
	  states.append(robot.leg3L.touch_sensor.getValue())
	  states.append(robot.leg4L.touch_sensor.getValue())

	  return np.array(states)
	
    def reset(self):
        # Reset the simulation
        self.simulationResetPhysics()
        self.simulationReset()
        super().step(self.__timestep)

        # Motors
	  self.move(initial_pos)


        # Sensors
        #self.__pendulum_sensor = self.getDevice("polePosSensor")
        #self.__pendulum_sensor.enable(self.__timestep)

        # Internals
        super().step(self.__timestep)

        # Open AI Gym generic
        return np.array([0, 0, 0, 0]).astype(np.float32)

    def step(self, action):
        # Execute the action
        for wheel in self.__wheels:
            #wheel.setVelocity(1.3 if action == 1 else -1.3)
            wheel.setVelocity(float(action[0])*5)
        super().step(self.__timestep)

        # Observation
        robot = self.getSelf()
	  self.state = get_states()


        # Done
	  done = 0
	  pos = robot.getPosition()
	  if pos[2]> goal_threshold or pos[1] > side_threshold or pos[2]< -goal_threshold or pos[1]< -side_threshold:
	  	done = 1


        # Reward
        reward = 0 if done else abs(normalizeToRange(pos[2], -5, 5, -3, 3, clip=True))
        #print(reward)
        return self.state.astype(np.float32), reward, done, {}
##=====================================================================================
env =  OpenAIGymEnvironment
agent = PPOAgent(numberOfInputs=env.observation_space.shape[0], numberOfActorOutputs=env.action_space.n)            
solved = False 
episodeCount = 0
episodeLimit = 2000



# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# Main loop:
# Run outer loop until the episodes limit is reached or the task is solved
while not solved and episodeCount < episodeLimit:
    observation = env.reset()  # Reset robot and get starting observation
    env.episodeScore = 0

    for step in range(env.stepsPerEpisode):
        # In training mode the agent samples from the probability distribution, naturally implementing exploration
        selectedAction, actionProb = agent.work(observation, type_="selectAction")
        
        # Step the supervisor to get the current selectedAction's reward, the new observation and whether we reached 
        # the done condition
        newObservation, reward, done, info = env.step([selectedAction])

        # Save the current state transition in agent's memory
        trans = Transition(observation, selectedAction, actionProb, reward, newObservation)
        agent.storeTransition(trans)  
              
        if done:
            # Save the episode's score
            env.episodeScoreList.append(env.episodeScore)
            agent.trainStep(batchSize=step)
            solved = env.solved()  # Check whether the task is solved
            break

        env.episodeScore += reward  # Accumulate episode reward
        observation = newObservation  # observation for next step is current step's newObservation    
   
    print("Episode #", episodeCount, "score:", env.episodeScore)
    episodeCount += 1  # Increment episode counter  
    
if not solved:
    print("Task is not solved, deploying agent for testing...")
elif solved:
    print("Task is solved, deploying agent for testing...")

observation = env.reset()
while True:
    selectedAction, actionProb = agent.work(observation, type_="selectActionMax")
    observation, _, _, _ = env.step([selectedAction])    


# Enter here exit cleanup code.
