"""/***************************************************************************
 *  InitClipsWorld.py -
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
print("Hello from initClipsWorld")
#import sys
#from tabnanny import verbose
#import numpy as np
import os
import imageio
import gym
print(sys.path)

#sys.path.append("~/fawkes/src/plugins/clips-gym/")
# Wrapper for PDDLGym for discrete action and observation space
#from PDDLExtension import BlocksWorld, LiteralSpace2, LiteralActionWrapper, LiteralObsWrapper
from ClipsWorld import ClipsWorld   #, LiteralSpace2, LiteralActionWrapper, LiteralObsWrapper

print("Hello after import ClipsWorld in initClipsWorld")


#Action Masking
from sb3_contrib.common.maskable.policies import MaskableActorCriticPolicy
from sb3_contrib.common.wrappers import ActionMasker
from sb3_contrib.ppo_mask import MaskablePPO
from stable_baselines3 import PPO
from stable_baselines3.ppo.policies import MlpPolicy


print("Hello after SB3 import in initClipsWorld")

if __name__ == '__main__':
    
    print("In initClipsWorld script")
    print("Script: Creating env")
    env = ClipsWorld()
    print("Observations: ", env.obs_dict)
    print("Action space: ",env.action_space)
    

    print("ExecutingClipsWorld: Config Value agent file name: ", file_name)
    model = MaskablePPO.load(file_name, env=env)
    
    print ("Loaded the agent from file")
    

