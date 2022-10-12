"""/***************************************************************************
 *  ClipsWorld.py -
 *
 *  Created:
 *  Copyright
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
  *  it under the terms of the GNU General Public License as published by
  *  the Free Software Foundation; either version 2 of the License, or
  *  (at your option) any later version.
  *
  *  This program is distributed in the hope that it will be useful,
  *  but WITHOUT ANY WARRANTY; without even the implied warranty of
  *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  *  GNU Library General Public License for more details.
  *
  *  Read the full text in the LICENSE.GPL file in the doc directory.
  */"""

import sys
from tokenize import String
sys.path.append("~/fawkes/plugins")

import clips_gym 

import numpy as np
import gym
from gym import spaces
import inspect
import ast

class ClipsWorld(gym.Env):
  """
  Custom Environment that follows gym interface.
  This is a simple env where the agent must learn to go always left. 
  """
  # Because of google colab, we cannot implement the GUI ('human' render mode)
  metadata = {'render.modes': ['console']}
  # Define constants for clearer code
  #LEFT = 0
  #RIGHT = 1

  def __init__(self, action_space_as_string_array=['action1'], obs_space_as_string_array=['obs1']):
    super(ClipsWorld, self).__init__()
    sorted_actions = sorted(set(action_space_as_string_array))
    sorted_obs = sorted(set(obs_space_as_string_array))

    #action dict
    set_keys = range(0,len(sorted_actions))
    self.action_dict = dict(zip(set_keys, sorted_actions))
    self.inv_action_dict = (dict (zip (sorted_actions, set_keys)))
    print(self.action_dict)

    #obs dict
    set_keys_obs = range(0,len(sorted_obs))
    self.obs_dict = dict(zip(set_keys_obs, sorted_obs))
    self.inv_obs_dict = (dict (zip (sorted_obs, set_keys_obs)))
    print(self.obs_dict)

    # Define action and observation space
    # They must be gym.spaces objects
    # Example when using discrete actions, we have two: left and right
    self.n_actions = len(sorted_actions)
    self.action_space = spaces.Discrete(self.n_actions)
    # The observation will be the coordinate of the agent
    # this can be described both by Discrete and Box space
    self.n_obs = len(sorted_obs)
    self.observation_space = gym.spaces.Box(0, 1, (self.n_obs,))
    print(self.observation_space)

    try:
      print(self.state)
      self.state = np.zeros(self.n_obs)
      print(self.state)
    except:
      print("State problem")

    #TODO add gametime, reward
    
    #self.observation_space = spaces.Box(low=0, high=self.grid_size,
    #                                    shape=(1,), dtype=np.float32)

  def reset(self):
    """
    Important: the observation must be a numpy array
    :return: (np.array) 
    """
    print("ClipsWorld: start reset function")

    #print("NOT IMPLEMENTED ",inspect.currentframe().f_code.co_name)
    print("ClipsWorld: reset: before get ClipsGymThread instance")
    p = clips_gym.ClipsGymThread.getInstance()
    #result = p.resetCX()
    print("ClipsWorld: reset: Finished resetCX ")

    print("ClipsWorld: reset: call create_rl_env_state_from_facts")
    fact_string = p.create_rl_env_state_from_facts()
    print("ClipsWorld reset: reseived facts: ", fact_string)
    raw_facts = ast.literal_eval(fact_string)
    print("\nfacts: ", raw_facts)
    state = self.get_state_from_facts(raw_facts)
    print("ClipsWorld: New env state from facts: ",state)
    print("ClipsWorld: end reset function")
    

    # Initialize the agent at the right of the grid
    #self.agent_pos = self.grid_size - 1
    # here we convert to float32 to make it more general (in case we want to use continuous actions)
    return state #np.array([self.n_obs]).astype(np.int_)

  def step(self, action):
    print("ClipsWorld: step ", action)
    print("Clips_gym add: ", clips_gym.add(1, 2))  # use the default parameter value
    p = clips_gym.ClipsGymThread.getInstance()
    result = p.step("Test")
    print(result)

    #TODO check action valid (if not done - reward -1) (da durch action masking nur valide actions ausgesucht werden sollten, außer es gibt keine validen mehr)

    # Create observation from clips
    fact_string = p.create_rl_env_state_from_facts()
    print("ClipsWorld reseived facts: ", fact_string)
    raw_facts = ast.literal_eval(fact_string)
    print("\nfacts: ", raw_facts)
    state = self.get_state_from_facts(raw_facts)
    print("New env state from facts: ",state)

    #  Flag that marks the termination of an episode
    # TODO if we use action masking check if there are valid action and switch to done if not
    done = False #bool(self.agent_pos == 0)

    # Null reward everywhere except when reaching the goal (left of the grid)
    reward = 1 #1 if self.agent_pos == 0 else 0

    # Optionally we can pass additional info, we are not using that for now
    info = {}

    return state, reward, done, info
    #return np.array([self.n_obs]).astype(np.int_), reward, done, info

  #Exposes a method called action_masks(), which returns masks for the wrapped env.
  # action_mask_fn: A function that takes a Gym environment and returns an action mask,
  #      or the name of such a method provided by the environment.
  def action_masks(self) -> np.ndarray:
    print("ClipsWorld: action_masks")
  # Returns the action mask for the current env. 
  #def mask_fn(env: gym.Env) -> np.ndarray:
    p = clips_gym.ClipsGymThread.getInstance()
    formulated_goals = p.getAllFormulatedGoals()
    print(formulated_goals)
    valid_actions = np.zeros((self.n_actions), dtype=int)
    """     for i in range(0, self.n_actions):
        #check if action is valid
        a = env.env.actions[i]        
        s = env.env.unwrapped._state       
        v = env.env.unwrapped._action_valid_test(s,a)
       
        if v:
            valid_actions[i]=1 """
    #print("Valid Actions: ", valid_actions)

    #TODO clips_gym call get executable goals
    print("NOT IMPLEMENTED ",inspect.currentframe().f_code.co_name)
    return valid_actions

  def render(self, mode='console'):
    if mode != 'console':
      raise NotImplementedError()
    # agent is represented as a cross, rest as a dot
    #print("." * self.agent_pos, end="")
    #print("x", end="")
    #print("." * (self.grid_size - self.agent_pos))

  def close(self):
    pass

  def get_state_from_facts(self,obs_f):
    print("ClipsWorld: in get_state_from_facts function")
    new_state = np.zeros(self.n_obs)
    print("new state np array")
    print("Obs space: ", self.obs_dict)
    for f in obs_f:
      if self.inv_obs_dict.get(f) is not None:
        pos = self.inv_obs_dict[f]
        #print(pos)
        new_state[pos]=1
      #else:
        #print(f"No key: '{f}' in dict")
    return new_state

  def set_state_from_facts(self,obs_f):
    print("ClipsWorld: set_state_from facts")
    new_state = np.zeros(self.n_obs)
    for f in obs_f:
      if self.inv_obs_dict.get(f) is not None:
        pos = self.inv_obs_dict[f]
        new_state[pos]=1
    print("New env state from facts: ",new_state)
    self.state = new_state
  
  def get_state(self):
    return self.state
    #
        #literals =[]
        #for f in obs_f:
            #x = re.split("\(|,|\)",f)
            #print(x)
            #for key, value in self.observations.items():
                #x = key.__str__().replace(":block", '').replace(":robot",'')
                #is_literal = True
                #print (f,x)        
                #if f == x:
                    #literals.append(self._inverted_obs[value])
        #print(literals)
        #self.env.env.set_state(self.get_state().with_literals(literals))
