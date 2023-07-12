"""/***************************************************************************
 *  Training.py -
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

sys.path.append("~/fawkes/src/plugins/clips_gym/")
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
    #print("Valid Actions: ", valid_actions)
    return valid_actions

if __name__ == '__main__':
    render = None
 #   env_name = "blockstower"
#    dir_path = "~/fawkes/src/plugins/rl-test/rl-agent"
    print("In Training script")
    print("Config values for env_name and path: " + env_name + " " + dir_path)
    print("Creating env")
    env = env_creator(env_name, dir_path, render)
    print("Env {} created".format(env_name))
    obs = env.reset()
    
    print("Observations: ", env.observations)
    print("Observation space: ",env.observation_space)
    print("Action space: ",env.action_space)
    print("Actions: ", env.actions)

    #Mask environment
    env = ActionMasker(env, mask_fn)  # Wrap to enable masking

    # MaskablePPO behaves the same as SB3's PPO unless the env is wrapped
    # with ActionMasker. If the wrapper is detected, the masks are automatically
    # retrieved and used when learning. Note that MaskablePPO does not accept
    # a new action_mask_fn kwarg, as it did in an earlier draft.
    model = MaskablePPO(MaskableActorCriticPolicy, env, verbose=1)

    print("Start trainig the agent - for {} timesteps".format(timesteps))
    model.learn(total_timesteps=timesteps)
    print ("Saving the agent at: ", file_name)
    model.save(file_name)
