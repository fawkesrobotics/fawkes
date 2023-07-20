"""/***************************************************************************
 *  ClipsWorldEnv.py -
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

# #!usr/bin/env python
import sys
import os

sys.path.append("~/fawkes/plugins")
print("In ClipsWorldEnv - sys path: ", sys.path)

# import clips_gym


# if __name__ == '__main__':
#     result = clips_gym.add(1,2)
#     print(result)

import clips_gym 

#print("Clips_gym add: ", clips_gym.add(1, 2))  # use the default parameter value
#p = clips_gym.ClipsGymThread.getInstance()
#result = p.step("Test")
#print(result)

#p= clips_gym.ClipsGymThread()
#print(p)
#print(p.getClipsEnv())


import gym
from gym.spaces.discrete import Discrete
import pddlgym
from pddlgym import structs
from pddlgym.core import PDDLEnv
from pddlgym.spaces import LiteralSpace, LiteralSetSpace
import numpy as np
import re

from pddlgym.rendering.blocks import render as blocks_render
#from .blocks import render as blocks_render

class ClipsWorldEnv(PDDLEnv):

    
    def __init__(self, domain_file, problem_dir, render):
        print("Creating ClipsWorld")
        super().__init__(domain_file, problem_dir, render=render, seed = 0)#, operators_as_actions=True)#blocks_render, seed=0)
        print("Render: ", render)
        state, info = self.reset()
        
        #problem = self.problems[0]
        #self._observation_space = LiteralSpace2(
         #   set(self.domain.predicates.values()) - set(self.action_predicates),
          #  state,
           # problem,
            #type_hierarchy=self.domain.type_hierarchy,
            #ype_to_parent_types=self.domain.type_to_parent_types)
        
    def step(self, action):
        obs, reward, done, info = super().step(action)
        
        print("Clips_gym add: ", clips_gym.add(1, 2))  # use the default parameter value
        p = clips_gym.ClipsGymThread.getInstance()
        result = p.step("Test")
        print(result)

        if done:
            reward = 10
        return obs, reward, done, info

    #def sample:

     #   initial_state = State(frozenset(self._problem.initial_state),
      #                        frozenset(self._problem.objects),
      #                        self._problem.goal)
      #  initial_state = self._handle_derived_literals(initial_state)
    


class LiteralSpace2(LiteralSetSpace):

    def __init__(self, predicates, state, problem,
                 lit_valid_test=lambda state,lit: True,
                 type_hierarchy=None,
                 type_to_parent_types=None):
        
        super().__init__(predicates,
                 lit_valid_test,
                 type_hierarchy,
                 type_to_parent_types)

        self._sample_state = state
        self._sample_problem = problem
        self.shape = tuple((len(predicates),len(self._sample_problem.objects)))
       
    def contains(self, x) -> bool:
        #print (x)
        
        return True

    """ def sample(self, state):
        print(super())
        self._update_objects_from_state(state)
        return self.sample_literal(state)
    #    raise NotImplementedError()

    def sample(self, problem):
        return frozenset(problem.initial_state) """

    def sample(self):
        return frozenset(self._sample_problem.initial_state)


    
class LiteralActionWrapper(gym.ActionWrapper):
    def __init__(self, env):
        super().__init__(env)
        self._literal_action_space = env.action_space
        env_state = env.reset()[0]
        env.action_space._update_objects_from_state(env_state)
        #print("Env state action wrapper: ", env_state)
        set_values = env.action_space._all_ground_literals
        set_keys = range(0,len(set_values))
        self.actions = dict(zip(set_keys, set_values))
        #print("Action mapping: \n", self.actions)
        self.n_actions = len (self.actions)
        self.action_space = Discrete(self.n_actions)
    
    def action(self, act: int):
        # modify act
        action = self.actions[act]
        #print("Agents acts: {} - {}".format(act,action))
        return action 


class LiteralObsWrapper(gym.ObservationWrapper):
    def __init__(self, env):
        super().__init__(env)
        env.reset()
        self._literal_observation_space = env.observation_space
        env_state = env.get_state()
        env.observation_space._update_objects_from_state(env_state)
        #print("Env state obs wrapper: ", env_state)
        set_keys = env.observation_space._all_ground_literals
        set_values = range(0,len(set_keys))
        self.observations = dict(zip(set_keys, set_values))
        self._inverted_obs = dict (zip(set_values, set_keys))


        #print("Observation mapping: ",self.observations)

        self.n_obs = len (self.observations)
        self.observation_space = gym.spaces.Box(0, 1, (self.n_obs,))
    
    def observation(self, obs):
        # modify obs
        literals = {}
        
        if isinstance(obs, pddlgym.structs.State):
            literals = obs.literals
        elif isinstance(obs, tuple):            
            literals=obs[0].literals
        else:
            literasl = obs
             
        #print("Observation literal: {}".format(literals))
        new_obs = np.zeros(self.n_obs)
        for l in literals:
            num = self.observations[l]
            new_obs[num]=1
        
        return new_obs

    
    def set_state_np(self, obs_np):
        #translates the np array observation state to a literal set for the pddlgym state
        literals = []
        for index,value in enumerate(obs_np):
            if value == 1:
                literals.append(self._inverted_obs[index])

        self.env.env.set_state(self.get_state().with_literals(literals))

    def set_state_lit(self, literals):
        #given a literal set as state for the pddlgym state
        self.env.env.set_state(self.get_state().with_literals(literals))
    
    def set_state_from_facts(self,obs_f):
        literals =[]
        for f in obs_f:
            #x = re.split("\(|,|\)",f)
            #print(x)
            for key, value in self.observations.items():
                x = key.__str__().replace(":block", '').replace(":robot",'')
                #is_literal = True
                #print (f,x)        
                if f == x:
                    literals.append(self._inverted_obs[value])
        print(literals)
        self.env.env.set_state(self.get_state().with_literals(literals))
         #for index, value in enumerate(obs_f):
            