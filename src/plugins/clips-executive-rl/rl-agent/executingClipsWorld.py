"""/***************************************************************************
 *  executingClipsWorld.py -
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

sys.path.append("~/fawkes/src/plugins/clips_gym/")
sys.path.append("~/fawkes/plugins")

# Wrapper for PDDLGym for discrete action and observation space
#from PDDLExtension import BlocksWorld, LiteralSpace2, LiteralActionWrapper, LiteralObsWrapper
from ClipsWorld import ClipsWorld#, LiteralSpace2, LiteralActionWrapper, LiteralObsWrapper
import clips_gym

#Action Masking
from sb3_contrib.common.maskable.policies import MaskableActorCriticPolicy
from sb3_contrib.common.wrappers import ActionMasker
from sb3_contrib.ppo_mask import MaskablePPO


if __name__ == '__main__':
    render = None
    #env_name = "blocks" #"blockstower"
    #dir_path = "~/fawkes/src/plugins/rl-test/rl-agent"

    print("ExecutingClipsWorld: start") #Config values for env_name and path: " + env_name + " " + dir_path)
    
    env = ClipsWorld()
    
    print("ExecutingClipsWorld: Created Env")

    print("Observations: ", env.obs_dict)
    print("Observation space: ",env.observation_space)
    print("Action space: ",env.action_space)
    #set to current env state given by c++
    
    #file_name = "~/fawkes/src/plugins/rl-test/rl-agent/MaskablePPOAgent_d"
    print("ExecutingClipsWorld: Config Value agent file name: ", file_name)
    model = MaskablePPO.load(file_name, env=env)

    print("ExecutingClipsWorld: Model loaded - before predicting")
    #valid_actions = mask_fn(env)#TODO set current env state
    #action, _ = model.predict(new_obs, deterministic=True)#, action_masks=valid_actions)
    
    print("Obs: ", obs)
    env.set_state_from_facts(obs)

    # action is the position in the discrete action space
    action, _ = model.predict(env.get_state(), action_masks = env.action_masks(), deterministic=True)#, action_masks=valid_actions)
    print("ExecutingClipsWorld: predicted action {0}".format(action))
    
    goal = env.action_dict[action]
    print(f"executingClipsWorld: next goal '{goal}'")
    
    #p = clips_gym.ClipsGymThread.getInstance()
    #goalID = p.getGoalId(goal)
    #print(f"executingClipsWorld: goal ID '{goalID}'")
    #p.assertRlGoalSelectionFact(goalID)
    #print (f"executingClipsWorld asserted RL goal selection fact")
    result = goal
    #result = str(env.action_dict[action])
    #print("Action {}: {}".format(action, env.action_dict[action]))
    #print("Valid actions: ", valid_actions)
    #print("Valid actions: ", env.env.actions)