import sys
import numpy as np
import os
import imageio
import gym
import pddlgym

sys.path.append("/home/sginter/Documents/fawkes/src/plugins/clips-gym/")

# Wrapper for PDDLGym for discrete action and observation space
#from PDDLExtension import BlocksWorld, LiteralSpace2, LiteralActionWrapper, LiteralObsWrapper
from ClipsWorld import ClipsWorld#, LiteralSpace2, LiteralActionWrapper, LiteralObsWrapper


#Action Masking
from sb3_contrib.common.maskable.policies import MaskableActorCriticPolicy
from sb3_contrib.common.wrappers import ActionMasker
from sb3_contrib.ppo_mask import MaskablePPO
"""
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
    """

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
    dir_path = "/home/sginter/Documents/fawkes/src/plugins/rl-test/rl-agent"

    print("Config values for env_name and path: " + env_name + " " + dir_path)
    #env = env_creator(env_name, dir_path, render)
    #env = ClipsWorld(action_space, obs_space, render='blocks_render')#,render) #, seed=0)
    
    action_space = ['TOWER-C1#a#b', 'TOWER-C1#a#c', 'TOWER-C1#a#d' , 'TOWER-C1#a#e']
    print(action_space)

    obs_space = ['clear(a)','clear(b)','clear(c)','clear(d)','clear(e)'\
        ,'handempty(robo1)','handfull(robo1)','holding(a)','holding(b)','holding(c)','holding(d)','holding(e)'\
        ,'on(a,b)','on(a,c)','on(a,d)','on(a,e)','on(b,a)','on(b,c)'\
        ,'on(b,d)','on(b,e)','on(c,a)','on(c,b)','on(c,d)','on(c,e)'\
        ,'on(d,a)','on(d,b)','on(d,c)','on(d,e)','on(e,a)','on(e,b)'\
        ,'on(e,c)','on(e,d)','ontable(a)', 'ontable(b)', 'ontable(c)', 'ontable(d)', 'ontable(e)']
    #print(obs_space)
    
    #env = env_creator(env_name, dir_path, render)
    env = ClipsWorld(action_space, obs_space)# render='blocks_render')#,render) #, seed=0)
    

    #env.reset()
    print("Created Env")
    print(env.obs_dict)
    #set to current env state given by c++
    print("Obs: ", obs)
    env.set_state_from_facts(obs)
    #obs = [0., 1., 1., 1., 0., 1., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 1., 1.,]
    #print("Obs: ", obs)
    #env.set_state_np(obs) #set_state_lit(obs) - as soon as we get the state from facts
    #print("Env state 1: ", env.get_state())
    
    #Mask environment
    #env = ActionMasker(env, mask_fn)  # Wrap to enable masking
    #print("Env state 2: ", env.get_state())
    
    new_obs = env.get_state()
    print("Env obs: ", new_obs)
    file_name = "/home/sginter/Documents/fawkes/src/plugins/rl-test/rl-agent/MaskablePPOAgent_d"
    print("Config Value agent file name: ", file_name)
    model = MaskablePPO.load(file_name, env=env)

    #valid_actions = mask_fn(env)#TODO set current env state
    action, _ = model.predict(new_obs, deterministic=True)#, action_masks=valid_actions)
    
    result = str(env.action_dict[action])
    print("Action {}: {}".format(action, env.action_dict[action]))
    #print("Valid actions: ", valid_actions)
    #print("Valid actions: ", env.env.actions)