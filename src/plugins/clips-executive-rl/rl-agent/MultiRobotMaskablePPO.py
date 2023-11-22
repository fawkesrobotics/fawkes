"""/***************************************************************************
 *  MultiRobotMaskablePPO.py -
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
import time
from collections import deque
from typing import Any, ClassVar, Dict, Optional, Tuple, Type, TypeVar, Union
import numpy as np
import torch as th
from gymnasium import spaces
from stable_baselines3.common import utils
from stable_baselines3.common.buffers import RolloutBuffer
from stable_baselines3.common.callbacks import BaseCallback, CallbackList, ConvertCallback, ProgressBarCallback
from stable_baselines3.common.on_policy_algorithm import OnPolicyAlgorithm
from stable_baselines3.common.policies import BasePolicy
from stable_baselines3.common.type_aliases import GymEnv, MaybeCallback, Schedule
from stable_baselines3.common.utils import explained_variance, get_schedule_fn, obs_as_tensor, safe_mean
from stable_baselines3.common.vec_env import VecEnv
from torch.nn import functional as F
from sb3_contrib.common.maskable.buffers import MaskableDictRolloutBuffer, MaskableRolloutBuffer
from sb3_contrib.common.maskable.policies import MaskableActorCriticPolicy
from sb3_contrib.common.maskable.utils import get_action_masks, is_masking_supported
from sb3_contrib.ppo_mask.policies import CnnPolicy, MlpPolicy, MultiInputPolicy
from MultiRobotMaskableRolloutBuffer import MultiRobotMaskableRolloutBuffer, MultiRobotMaskableDictRolloutBuffer
from threading import Thread, Lock
import threading
from sb3_contrib.ppo_mask import MaskablePPO

class MultiRobotMaskablePPO(MaskablePPO):
    """
    Proximal Policy Optimization algorithm (PPO) (clip version) with Invalid Action Masking for Muli-Robot Scenarios in the CLIPS Executive.

    Based on the original Stable Baselines 3 implementation and the maskable PPO implementation in Stable Baselines3 contrib.

    Introduction to PPO: https://spinningup.openai.com/en/latest/algorithms/ppo.html
    Background on Invalid Action Masking: https://arxiv.org/abs/2006.14171

    :param policy: The policy model to use (MlpPolicy, CnnPolicy, ...)
    :param env: The environment to learn from (if registered in Gym, can be str)
    :param learning_rate: The learning rate, it can be a function
        of the current progress remaining (from 1 to 0)
    :param n_steps: The number of steps to run for each environment per update
        (i.e. batch size is n_steps * n_env where n_env is number of environment copies running in parallel)
    :param batch_size: Minibatch size
    :param n_epochs: Number of epoch when optimizing the surrogate loss
    :param gamma: Discount factor
    :param gae_lambda: Factor for trade-off of bias vs variance for Generalized Advantage Estimator
    :param clip_range: Clipping parameter, it can be a function of the current progress
        remaining (from 1 to 0).
    :param clip_range_vf: Clipping parameter for the value function,
        it can be a function of the current progress remaining (from 1 to 0).
        This is a parameter specific to the OpenAI implementation. If None is passed (default),
        no clipping will be done on the value function.
        IMPORTANT: this clipping depends on the reward scaling.
    :param normalize_advantage: Whether to normalize or not the advantage
    :param ent_coef: Entropy coefficient for the loss calculation
    :param vf_coef: Value function coefficient for the loss calculation
    :param max_grad_norm: The maximum value for the gradient clipping
    :param target_kl: Limit the KL divergence between updates,
        because the clipping is not enough to prevent large update
        see issue #213 (cf https://github.com/hill-a/stable-baselines/issues/213)
        By default, there is no limit on the kl div.
    :param stats_window_size: Window size for the rollout logging, specifying the number of episodes to average
        the reported success rate, mean episode length, and mean reward over
    :param tensorboard_log: the log location for tensorboard (if None, no logging)
    :param policy_kwargs: additional arguments to be passed to the policy on creation
    :param verbose: the verbosity level: 0 no output, 1 info, 2 debug
    :param seed: Seed for the pseudo random generators
    :param device: Device (cpu, cuda, ...) on which the code should be run.
        Setting it to auto, the code will be run on the GPU if possible.
    :param _init_setup_model: Whether or not to build the network at the creation of the instance
    """

    policy_aliases: ClassVar[Dict[str, Type[BasePolicy]]] = {
        "MlpPolicy": MlpPolicy,
        "CnnPolicy": CnnPolicy,
        "MultiInputPolicy": MultiInputPolicy,
    }

    def __init__(
        self,
        policy: Union[str, Type[MaskableActorCriticPolicy]],
        env: Union[GymEnv, str],
        learning_rate: Union[float, Schedule] = 3e-4,
        n_steps: int = 2048,
        batch_size: Optional[int] = 64,
        n_epochs: int = 10,
        gamma: float = 0.99,
        gae_lambda: float = 0.95,
        clip_range: Union[float, Schedule] = 0.2,
        clip_range_vf: Union[None, float, Schedule] = None,
        normalize_advantage: bool = True,
        ent_coef: float = 0.0,
        vf_coef: float = 0.5,
        max_grad_norm: float = 0.5,
        target_kl: Optional[float] = None,
        stats_window_size: int = 100,
        tensorboard_log: Optional[str] = None,
        policy_kwargs: Optional[Dict[str, Any]] = None,
        verbose: int = 0,
        seed: Optional[int] = None,
        device: Union[th.device, str] = "auto",
        _init_setup_model: bool = True,
        n_robots: int = 3,
        time_based: bool = False,
        n_time: int = 450,
        deadzone: int = 10,
        wait_for_all_robots: bool = True
    ):
        self.time_based = time_based
        super().__init__(
            policy,
            env,
            learning_rate=learning_rate,
            n_steps=n_steps,
            batch_size=batch_size,
            n_epochs=n_epochs,
            gamma=gamma,
            gae_lambda=gae_lambda,
            clip_range = clip_range,
            clip_range_vf=clip_range_vf,
            normalize_advantage=normalize_advantage,
            ent_coef=ent_coef,
            vf_coef=vf_coef,
            max_grad_norm=max_grad_norm,
            target_kl=target_kl,
            stats_window_size=stats_window_size,
            tensorboard_log=tensorboard_log,
            policy_kwargs=policy_kwargs,
            verbose=verbose,
            seed=seed,
            device=device,
            _init_setup_model=_init_setup_model,
        )
        self.n_robots = n_robots
        
        self.n_time = n_time
        self.deadzone = deadzone
        self.wait_for_all_robots = wait_for_all_robots
        self.n_current_steps = 0
        self.no_callback = False
        self.rollouts_gathered = False
        print("init MAMPPO")

    def _setup_model(self) -> None:
        self._setup_lr_schedule()
        self.set_random_seed(self.seed)

        buffer_cls = MultiRobotMaskableDictRolloutBuffer if isinstance(self.observation_space, spaces.Dict) else MultiRobotMaskableRolloutBuffer

        self.policy = self.policy_class(
            self.observation_space,
            self.action_space,
            self.lr_schedule,
            **self.policy_kwargs,  # pytype:disable=not-instantiable
        )
        self.policy = self.policy.to(self.device)

        if not isinstance(self.policy, MaskableActorCriticPolicy):
            raise ValueError("Policy must subclass MaskableActorCriticPolicy")

        if self.time_based:
            buffer_size = 1000
        else:
            buffer_size = self.n_steps
        
        self.rollout_buffer = buffer_cls(
            buffer_size,
            self.observation_space,
            self.action_space,
            self.device,
            gamma=self.gamma,
            gae_lambda=self.gae_lambda,
            n_envs=self.n_envs,
        )

        # Initialize schedules for policy/value clipping
        self.clip_range = get_schedule_fn(self.clip_range)
        if self.clip_range_vf is not None:
            if isinstance(self.clip_range_vf, (float, int)):
                assert self.clip_range_vf > 0, "`clip_range_vf` must be positive, " "pass `None` to deactivate vf clipping"

            self.clip_range_vf = get_schedule_fn(self.clip_range_vf)
    
    def collect_rollouts(
        self,
        env: VecEnv,
        callback: BaseCallback,
        rollout_buffer: RolloutBuffer,
        n_rollout_steps: int,
        use_masking: bool = True,
    ) -> bool:
        """
        Collect experiences using the current policy and fill a ``RolloutBuffer``.
        The term rollout here refers to the model-free notion and should not
        be used with the concept of rollout used in model-based RL or planning.

        This method is largely identical to the implementation found in the parent class.

        :param env: The training environment
        :param callback: Callback that will be called at each step
            (and at the beginning and end of the rollout)
        :param rollout_buffer: Buffer to fill with rollouts
        :param n_steps: Number of experiences to collect per environment
        :param use_masking: Whether or not to use invalid action masks during training
        :return: True if function returned with at least `n_rollout_steps`
            collected, False if callback terminated rollout prematurely.
        """

        assert isinstance(
            rollout_buffer, (MultiRobotMaskableRolloutBuffer, MultiRobotMaskableDictRolloutBuffer)
        ), "RolloutBuffer doesn't support action masking"
        assert self._last_obs is not None, "No previous observation was provided"
        # Switch to eval mode (this affects batch norm / dropout)
        self.policy.set_training_mode(False)
        self.n_current_steps = 0
        threads = []
        self.rollouts_gathered = False
        self.no_callback = False
        action_masks = None
        rollout_buffer.reset()

        if use_masking and not is_masking_supported(env):
            raise ValueError("Environment does not support action masking. Consider using ActionMasker wrapper")

        callback.on_rollout_start()

        if self.time_based:
            start_time = time.time()
            while time.time() - start_time < self.n_time:
                if len(threads) < self.n_robots and time.time() - start_time < self.n_time - self.deadzone:
                    t = Thread(target= self._do_step_time_based, args = (env, callback, rollout_buffer, use_masking))
                    threads.append(t)
                    #print(f"Thread appended, new length: {len(threads)}")
                    t.start()
                threads_alive =[]
                for thread in threads:
                    if not thread.is_alive():
                        #print("Thread not alive")
                        thread.join()
                        if self.no_callback:
                            return False
                    else:
                        #print("Thread alive, appending")
                        threads_alive.append(thread)
                threads = threads_alive
                #print(f"Number of Threads after cleanup {len(threads)}")
        else:
            while self.n_current_steps < n_rollout_steps:
                if len(threads) < self.n_robots:
                    t = Thread(target= self._do_step, args = (env, callback, rollout_buffer, use_masking))
                    threads.append(t)
                    #print(f"Thread appended, new length: {len(threads)}")
                    t.start()
                threads_alive =[]
                for thread in threads:
                    if not thread.is_alive():
                        #print("Thread not alive")
                        thread.join()
                        if self.no_callback:
                            return False
                    else:
                        #print("Thread alive, appending")
                        threads_alive.append(thread)
                threads = threads_alive
                #print(f"Number of Threads after cleanup {len(threads)}")
        
        print("Rollouts gathered")
        self.rollouts_gathered = True
        if self.wait_for_all_robots:
            print("Waiting for steps to conclude")
            while len(threads) > 0:
                for thread in threads:
                    threads_alive =[]
                    if not thread.is_alive():
                        thread.join()
                        if self.no_callback:
                            return False
                    else:
                        threads_alive.append(thread)
                    threads = threads_alive
            print("Steps concluded")
        print("Updating policy")
        with th.no_grad():
            # Compute value for the last timestep
            # Masking is not needed here, the choice of action doesn't matter.
            # We only want the value of the current observation.
            values = self.policy.predict_values(obs_as_tensor(self._last_obs, self.device))

        rollout_buffer.compute_returns_and_advantage(last_values=values, dones=self._last_episode_starts)

        callback.on_rollout_end()

        return True

    def _do_step(
        self,
        env: VecEnv,
        callback: BaseCallback,
        rollout_buffer: RolloutBuffer,
        use_masking: bool = True,
    ):
        current_thread = threading.get_ident()
        print(f"In new Thread {current_thread}")
        with th.no_grad():
            # Convert to pytorch tensor or to TensorDict
            obs_tensor = obs_as_tensor(self._last_obs, self.device)
            print(f"RL: before get_action masks, Thread {current_thread}")
            # This is the only change related to invalid action masking
                
            if use_masking:
                action_masks = get_action_masks(env)
            print(f"RL: after get_action masks, Thread {current_thread}")
            actions, values, log_probs = self.policy(obs_tensor, action_masks=action_masks)
            actions = actions.cpu().numpy()
        print(f"Calling step in thread {current_thread}")
        new_obs, rewards, dones, infos = env.step(actions)
        if infos[0].get("outcome")=="FAILED":
            print(f"RL: step failed, returning... (Thread {current_thread})")
            return
        self.num_timesteps += env.num_envs
        print(f"Vecenv: {env.reset_infos}")
        if self.no_callback:
            print(f"RL: training has finished in another step function, returning... (Thread {current_thread})")
            return
        # Give access to local variables
        callback.update_locals(locals())
        
        if not callback.on_step():
            self.no_callback = True
            print(f"RL: no callback, returning... (Thread {current_thread})")
            return

        self._update_info_buffer(infos)
        self.n_current_steps += 1

        if isinstance(self.action_space, spaces.Discrete):
            # Reshape in case of discrete action
            actions = actions.reshape(-1, 1)

        # Handle timeout by bootstraping with value function
        # see GitHub issue #633
        for idx, done in enumerate(dones):
            if (
                done
                and infos[idx].get("terminal_observation") is not None
                and infos[idx].get("TimeLimit.truncated", False)
            ):
                terminal_obs = self.policy.obs_to_tensor(infos[idx]["terminal_observation"])[0]
                with th.no_grad():
                    terminal_value = self.policy.predict_values(terminal_obs)[0]
                rewards[idx] += self.gamma * terminal_value
            
        if not self.rollouts_gathered:
            rollout_buffer.add(
                self._last_obs,
                actions,
                rewards,
                self._last_episode_starts,
                values,
                log_probs,
                action_masks=action_masks,
            )
        self._last_obs = new_obs
        self._last_episode_starts = dones
        print(f"RL: finished step in thread {current_thread}")


    def _do_step_time_based(
        self,
        env: VecEnv,
        callback: BaseCallback,
        rollout_buffer: RolloutBuffer,
        use_masking: bool = True,
    ):
        current_thread = threading.get_ident()
        print(f"In new Thread {current_thread}")
            #self._last_obs = env.getCurrentObs()
        with th.no_grad():
            # Convert to pytorch tensor or to TensorDict
            obs_tensor = obs_as_tensor(self._last_obs, self.device)
            print(f"RL: before get_action masks, Thread {current_thread}")
            # This is the only change related to invalid action masking
                
            if use_masking:
                action_masks = get_action_masks(env)
            print(f"RL: after get_action masks, Thread {current_thread}")
            actions, values, log_probs = self.policy(obs_tensor, action_masks=action_masks)
            actions = actions.cpu().numpy()
        print(f"Calling step in thread {current_thread}")
        new_obs, rewards, dones, infos = env.step(actions)
        if infos[0].get("outcome")=="FAILED":
            print(f"RL: step failed, returning... (Thread {current_thread})")
            return
        self.num_timesteps += env.num_envs

        # Give access to local variables
 
        callback.update_locals(locals())
        if not callback.on_step():
            self.no_callback = True
            print(f"RL: no callback, returning... (Thread {current_thread})")
            return

        self._update_info_buffer(infos)

        if isinstance(self.action_space, spaces.Discrete):
            # Reshape in case of discrete action
            actions = actions.reshape(-1, 1)

        # Handle timeout by bootstraping with value function
        # see GitHub issue #633
        for idx, done in enumerate(dones):
            if (
                done
                and infos[idx].get("terminal_observation") is not None
                and infos[idx].get("TimeLimit.truncated", False)
            ):
                terminal_obs = self.policy.obs_to_tensor(infos[idx]["terminal_observation"])[0]
                with th.no_grad():
                    terminal_value = self.policy.predict_values(terminal_obs)[0]
                rewards[idx] += self.gamma * terminal_value
            
        if not self.rollouts_gathered:
            rollout_buffer.add(
                self._last_obs,
                actions,
                rewards,
                self._last_episode_starts,
                values,
                log_probs,
                action_masks=action_masks,
            )
        self._last_obs = new_obs
        self._last_episode_starts = dones
        print(f"RL: finished step in thread {current_thread}")

        

