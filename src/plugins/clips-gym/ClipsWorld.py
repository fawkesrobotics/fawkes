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

import clips_gym 

import numpy as np
import gym
from gym import spaces
import inspect
import ast
import pandas as pd
from itertools import product

def expand_grid(dictionary):
   return pd.DataFrame([row for row in product(*dictionary.values())], 
                       columns=dictionary.keys())


#def generateActionSpace(clips_gym):
		


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

  def __init__(self):
    super(ClipsWorld, self).__init__()

    #generate action space
    print("ClipsWorld init: before generateActionSpace")
    p = clips_gym.ClipsGymThread.getInstance()
    action_space = p.generateActionSpace()
    print("ClipsWorld init: after generateActionSpace\n action_space = ", action_space)
    """ 
    #-----------
    #Get Goal Class list
    goalClasses = p.getGoalClassList()
    print("ClipsWorld: goalClasses: ", goalClasses)
    #for(std::string goalClass: goalClasses)
    partial = {}

    action_space_2=[];
    
    #goalClass = "TOWER-C1"
    for goalClass in goalClasses:
      #get key value map of param-name and param-type (buttom - block)
      mapParamNameType =  p.getParamsNameTypeMapOfGoal(goalClass)
      print("clipsWorld: mapParamNameType ", mapParamNameType)

      if not mapParamNameType:
        c = (goalClass + "#").replace(" ","")
        p.log('GoalClass without params: ' + c)
        action_space_2 += c
        continue;


      for pair in sorted(mapParamNameType.items()):
        #print(x)
      #For each param of the goal do:
      #for key, value in mapParamNameType:
        #print("Key: {} Value:{}", key, value)
        #{buttom#a, buttom#b,...}	
        
        #paramNameDOComb = p.getParamNameDomainObjectsComb(key, value);
        paramNameDOComb = p.getParamNameDomainObjectsComb(pair[0], pair[1]) #x,mapParamNameType[x])
        #print(paramNameDOComb)
        #partial[x]=	paramNameDOComb
        partial[pair[0]] = paramNameDOComb
      
      df = expand_grid(partial)
      df = df.reindex(sorted(df.columns), axis=1)
      #print(df)
      df.insert(0,'Class', goalClass)
      #print("DataFrame: ", df)
      x = df.to_string(header=False,index=False,index_names=False).split('\n')
      vals = ['#'.join(ele.split()) for ele in x]
      #print(vals)
      action_space_2 += vals
      
    #-----------------------------------------
    p.log('Actionspace: '+' '.join(action_space_2[:10])+ ' '.join(action_space_2[-10:]))
    p.log('Actionspace size: '+str(len(action_space_2)))
    action_space = action_space_2
    """
    #print("ClipsWorld init: after generateActionSpace\n action_space = ", action_space)

    #generate observation space
    print("ClipsWorld init: before generateObservationSpace")
    obs_space = p.generateObservationSpace()

    #-----------------------------------------
    

    obs_space_2=[];
    
    # Key: predicateName Value:ParamName-Type-Dict
    domainPredicatesDict = p.getDomainPredicates()
    # predicate = "ontable"
    for predicate in domainPredicatesDict:
      predicateParamsObjectComb = {}
      #print("ClipsWorld: predicate ", predicate)
      #get key value map of param-name and param-type (buttom - block)
      mapParamNameType =  domainPredicatesDict[predicate]
      #print("clipsWorld: mapParamNameType ", mapParamNameType)
      if not mapParamNameType:
        continue

      for x in mapParamNameType:
        #For each param of the predicate do:
        #{buttom#a, buttom#b,...}	
        #paramNameDOComb = p.getParamNameDomainObjectsComb(x,domainPredicatesDict[predicate][x]) #mapParamNameType[x]);
        paramNameDOComb = p.getDomainObjects(mapParamNameType[x])
        if not paramNameDOComb:
          continue
        #print(paramNameDOComb)
        predicateParamsObjectComb[x]=	paramNameDOComb
        break
      
      #print(predicateParamsObjectComb)
      obs_df = expand_grid(predicateParamsObjectComb)
      #print(obs_df)
      #obs_df = obs_df.reindex(sorted(obs_df.columns), axis=1)
      #print(df)
      obs_df.insert(0,'(','(')
      obs_df.insert(len(obs_df.columns),')',')')
      obs_df.insert(0,'Predicate', predicate)
      #print(obs_df)
      obs_str = obs_df.to_string(header=False,index=False,index_names=False).split('\n')
      vals = ['#'.join(ele.split()) for ele in obs_str]
      vals = [w.replace('#(#','(') for w in vals]
      vals = [w.replace('#)',')') for w in vals]
      obs_space_2 += vals

      obs_space = obs_space_2
    #-----------------------------------------
    p.log('Observationspace: '+' '.join(obs_space_2[:10]))
    p.log('Observationspace size: '+str(len(obs_space_2)))

    #print("ClipsWorld init: after generateObservationSpace\n obs_space = ", obs_space_2)

    sorted_actions = sorted(set(action_space)) #action_space_as_string_array))
    sorted_obs = sorted(set(obs_space))#obs_space_as_string_array))

    #action dict: key: number; value: goal
    set_keys = range(0,len(sorted_actions))
    self.action_dict = dict(zip(set_keys, sorted_actions))
    #inv_action_dict: key: goal; value number
    self.inv_action_dict = (dict (zip (sorted_actions, set_keys)))

    #obs dict: key: number; value: facts
    set_keys_obs = range(0,len(sorted_obs))
    self.obs_dict = dict(zip(set_keys_obs, sorted_obs))
    #inv_obs_dict: key: facts; value: number
    self.inv_obs_dict = (dict (zip (sorted_obs, set_keys_obs)))
    
    # Define action and observation space
    # They must be gym.spaces objects
    # Example when using discrete actions, we have two: left and right
    self.n_actions = len(sorted_actions)
    self.action_space = spaces.Discrete(self.n_actions)
    
    # The observation will be the coordinate of the agent
    # this can be described both by Discrete and Box space
    self.n_obs = len(sorted_obs)
    self.observation_space = gym.spaces.Box(0, 1, (self.n_obs,))

    try:
      self.state = np.zeros(self.n_obs)
      print("self.state: ", self.state)
    except:
      print("State problem")

    #TODO add gametime, reward
    

  def reset(self):
    """
    Important: the observation must be a numpy array
    :return: (np.array) 
    """
    print("ClipsWorld: start reset function")

    #print("NOT IMPLEMENTED ",inspect.currentframe().f_code.co_name)
    print("ClipsWorld: reset: before get ClipsGymThread instance")
    p = clips_gym.ClipsGymThread.getInstance()
    result = p.resetCX()
    print("ClipsWorld: reset: Finished resetCX ")

    print("ClipsWorld: reset: call create_rl_env_state_from_facts")
    fact_string = p.create_rl_env_state_from_facts()
    #print("ClipsWorld reset: reseived facts: ", fact_string)
    raw_facts = ast.literal_eval(fact_string)
    #print("\nfacts: ", raw_facts)
    state = self.get_state_from_facts(raw_facts)
    #print("ClipsWorld: New env state from facts: ",state)
    p.log("ClipsWorld: end reset function")
    

    # Initialize the agent at the right of the grid
    #self.agent_pos = self.grid_size - 1
    # here we convert to float32 to make it more general (in case we want to use continuous actions)
    return np.array(state).astype(np.int_)  #np.array(state).astype(np.int_)
    #state #

  def step(self, action):
    goal = self.action_dict[action]
    p = clips_gym.ClipsGymThread.getInstance()
    p.log(f"ClipsWorld: step '{action}': '{goal}'")
    result = p.step(goal) #+"#")
    print("ClipsWorld: p.step result: ", result)
    #print("ClipsWorld: observation ", result.observation)
    p.log(f"ClipsWorld: info  '{result.info}'")
    p.log(f"ClipsWorld: reward '{result.reward}'")

    #TODO check action valid (if not done - reward -1) (da durch action masking nur valide actions ausgesucht werden sollten, außer es gibt keine validen mehr)

    # Create observation from clips
    #print("ClipsWorld reseived facts: ", fact_string)
    raw_facts = ast.literal_eval(result.observation)#fact_string)
    #print("\nfacts: ", raw_facts)
    state = self.get_state_from_facts(raw_facts)
    print("New env state from facts: ",state)

    #  Flag that marks the termination of an episode
    # TODO if we use action masking check if there are valid action and switch to done if not
    executableGoals = p.getAllFormulatedExecutableGoals()
    print ("ClipsWorld: getAllFormulatedExecutableGoals {} length: ", executableGoals, len(executableGoals))
    
    time_sec =  p.getRefboxGameTime()
    p.log(f"ClipsWorld: Time: '{time_sec}'")
    game_time = 1200 #180 #in sec = normally 1200
    reward = 1 
    if time_sec >= game_time:
      # game over (e.g. if over 300 points you might won the game - extra check with refbox necessary / no logic for game extension!)
      done = True
      reward = result.reward + 10
    elif not executableGoals and time_sec < game_time:
      # there are no executable goals, but game didn't finished
      p.clipsGymSleep(500) #time in milliseconds
      done = False
      reward = 0
    else:
      #there are still executable goals (middle of the game)
      done = False
      reward = result.reward
    #done = False if len(executableGoals) else True #bool(self.agent_pos == 0)
    p.log(f"\n\nClipsWorld: done '{done}' reward {reward}\n")
    

    # Optionally we can pass additional info, we are not using that for now
    info = {}

    return state, reward, done, info
    #return np.array([self.n_obs]).astype(np.int_), reward, done, info

  #Exposes a method called action_masks(), which returns masks for the wrapped env.
  # action_mask_fn: A function that takes a Gym environment and returns an action mask,
  #      or the name of such a method provided by the environment.
  def action_masks(self) -> np.ndarray:
    print("ClipsWorld: in action_masks")
  # Returns the action mask for the current env. 
  #def mask_fn(env: gym.Env) -> np.ndarray:
    p = clips_gym.ClipsGymThread.getInstance()
    executable_goals = p.getAllFormulatedExecutableGoals()
    print("ClipsWorld: action_masks executable goals: ")
    import json
    p.log(json.dumps(self.inv_action_dict))

    valid_actions = np.zeros((self.n_actions), dtype=int)
    print("ClipsWorld: action_masks size: {0} {1}".format(len(valid_actions), valid_actions[:5]))
    for g in executable_goals:
      goal = g.getGoalString()
      p.log("Executable goal: "+goal)
      pos = self.inv_action_dict.get(goal)
      if (pos is not None):
        print(pos)
        valid_actions[pos]=1
        """     for i in range(0, self.n_actions):
        #check if action is valid
        a = env.env.actions[i]        
        s = env.env.unwrapped._state       
        v = env.env.unwrapped._action_valid_test(s,a)
       
        if v:
            valid_actions[i]=1 """
    #print("Valid Actions: ", valid_actions)

    #TODO clips_gym call get executable goals
    #print("NOT IMPLEMENTED ",inspect.currentframe().f_code.co_name)
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
    #print("new state np array")
    #print("Obs space: ", self.obs_dict)
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

  def getCurrentObs(self):
    print(f"ClipsWorld: getCurrentObs ")
    p = clips_gym.ClipsGymThread.getInstance()
    fact_string = p.create_rl_env_state_from_facts()
    print("ClipsWorld: fact_string: ", fact_string)
    raw_facts = ast.literal_eval(fact_string)
    print("ClipsWorld: getCurrentObs facts: ", raw_facts)
    state = self.get_state_from_facts(raw_facts)
    print("ClipsWorld: getCurrentObs new env state: ",state)
    return state

    
