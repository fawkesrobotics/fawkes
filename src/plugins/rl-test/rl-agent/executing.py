"""/***************************************************************************
 *  executing.py -
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
import numpy as np
import os
import imageio
import gym
import pddlgym

# Wrapper for PDDLGym for discrete action and observation space
#from PDDLExtension import BlocksWorld, LiteralSpace2, LiteralActionWrapper, LiteralObsWrapper
from ClipsWorldEnv import ClipsWorld, LiteralSpace2, LiteralActionWrapper, LiteralObsWrapper


#Action Masking
from sb3_contrib.common.maskable.policies import MaskableActorCriticPolicy
from sb3_contrib.common.wrappers import ActionMasker
from sb3_contrib.ppo_mask import MaskablePPO

def env_creator(env_name, dir_path, render):
    domain_file = os.path.join(dir_path, "{}.pddl".format(env_name))

    #gym_name = env_name.capitalize()
    problem_dirname = env_name
    problem_dir = os.path.join(dir_path, problem_dirname)

    print("Load domain file from: {}\nand problem file: {}".format(domain_file, problem_dir))

    #env = BlocksWorld(domain_file, problem_dir, render='blocks_render')#,render) #, seed=0)
    env = ClipsWorld(domain_file, problem_dir, render='blocks_render')#,render) #, seed=0)
    
    env.fix_problem_index(0)
    env = LiteralObsWrapper(LiteralActionWrapper(env)) 
    print(env)
    return env

# Returns the action mask for the current env. 
def mask_fn(env: gym.Env) -> np.ndarray:
    valid_actions = np.zeros((env.action_space.n), dtype=int)
    for i in range(0, env.action_space.n):
        #check if action is valid
        a = env.env.actions[i]        
        s = env.env.unwrapped._state       
        v = env.env.unwrapped._action_valid_test(s,a)
       
        if v:
            valid_actions[i]=1
        else:
            print("Action: ", a)
            print("Unwrapped state: ", s)
            print("Action valid test: ", v)
    #print("Valid Actions: ", valid_actions)
    return valid_actions

if __name__ == '__main__':
    render = None
    env_name = "blocks" #"blockstower"
    dir_path = "~/fawkes/src/plugins/rl-test/rl-agent"

    print("Config values for env_name and path: " + env_name + " " + dir_path)
    env = env_creator(env_name, dir_path, render)
    env.reset()
    print("Created Env")
    print(env.observations)
    #set to current env state given by c++
    print("Obs: ", obs)
    env.set_state_from_facts(obs)
    #obs = [0., 1., 1., 1., 0., 1., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 1., 1.,]
    #print("Obs: ", obs)
    #env.set_state_np(obs) #set_state_lit(obs) - as soon as we get the state from facts
    print("Env state 1: ", env.get_state())
    
    #Mask environment
    env = ActionMasker(env, mask_fn)  # Wrap to enable masking
    print("Env state 2: ", env.get_state())
    
    new_obs = env.observation(env.get_state())
    print("Env obs: ", new_obs)
    #file_name = "~/fawkes/src/plugins/rl-test/rl-agent/MaskablePPOAgent"
    print("Config Value agent file name: ", file_name)
    model = MaskablePPO.load(file_name, env=env)

    valid_actions = mask_fn(env)#TODO set current env state
    action, _ = model.predict(new_obs, deterministic=True, action_masks=valid_actions)
    
    result = str(env.env.actions[action])
    print("Action {}: {}".format(action, env.env.actions[action]))
    print("Valid actions: ", valid_actions)
    print("Valid actions: ", env.env.actions)