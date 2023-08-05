#! /usr/bin/env python
# -*- Mode: python; py-indent-offset: 4; indent-tabs-mode: nil; coding: utf-8; -*-
# Copyright (c) 2021 Huazhong University of Science and Technology, Dian Group
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation;
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
#
# Author: Xun Deng <dorence@hust.edu.cn>
#         Hao Yin <haoyin@uw.edu>

from ctypes import *
from py_interface import *
from DQN import DQNAgent, ReinforceAgent
from A2C import A2CAgent
import torch
import numpy as np
import time
import os
from tqdm import tqdm

class AiConstantRateEnv(Structure):
    _pack_ = 1
    _fields_ = [
        ('mcs', c_ubyte),
        ('max_mcs', c_ubyte),
        ('cw', c_ushort),
        ('throughput', c_double),
        ('snr', c_double)
    ]


class AiConstantRateAct(Structure):
    _pack_ = 1
    _fields_ = [
        ('nss', c_ubyte),
        ('next_mcs', c_ubyte)
    ]


class AiConstantRateContainer:
    use_ns3ai = True

    def __init__(self, uid: int = 2335,model_name='',retrain=False) -> None:
        self.rl = Ns3AIRL(uid, AiConstantRateEnv, AiConstantRateAct)
        self.agent = ReinforceAgent(1,8)
        self.isFinished = False
        self.startTime = time.time()
        self.packetTotal = 0
        self.packetLastSecond = 0
        self.last_tpt = 0
        self.last_state = {'throughput': 0, 'cw': 0, 'mcs': 0}
        self.cur_state = {'throughput': 0, 'cw': 0, 'mcs': 0}
        self.last_action = {'nss': 0, 'next_mcs': 0}
        self.log_snr = 0
        self.isInference = False
        self.isFirstEpisode = True
        # check if model file exists then load the model
        if os.path.exists(model_name):
            self.load_model(model_name, retrain)
            self.isInference = True if not retrain else False


    def train_dqn(self, env: AiConstantRateEnv, act: AiConstantRateAct) -> AiConstantRateAct:
        if self.isInference:
            return self.inference_dqn(env, act)
        self.cur_state['throughput'] = env.throughput
        self.cur_state['cw'] = 0
        self.cur_state['mcs'] = env.mcs
        self.cur_state['snr'] = np.log(env.snr+1)
        # use min-max normailize the input
        # _max = max([self.cur_state['throughput'], self.cur_state['cw'], self.cur_state['mcs'], self.cur_state['snr']])
        # _min = min([self.cur_state['throughput'], self.cur_state['cw'], self.cur_state['mcs'], self.cur_state['snr']])
        # self.cur_state['throughput'] = (self.cur_state['throughput'] - _min)/(_max - _min)
        # self.cur_state['snr'] = (self.cur_state['snr'] - _min)/(_max - _min) 
        # todo: inaccurate one_hot
        one_hot = [0]*8  # Initialize the array with zeros
        # one_hot[self.cur_state['mcs']] = 1  # Set the index of the input value to 1

        if self.last_state['mcs'] == 0 and self.last_state['cw'] == 0 and self.last_state['throughput'] == 0:
            self.last_state['mcs'] = env.mcs
            self.last_state['cw'] = 0
            self.last_state['throughput'] = env.throughput
            self.last_state['snr'] = np.log(env.snr+1)
            self.last_action['nss'] = act.nss
            self.last_action['next_mcs'] = act.next_mcs
            return act
        rl_input = [self.last_state['snr']]
        # DARA Reward Function
        # reward = self.cur_state['mcs']/7.0 * self.cur_state['cw']
        reward = self.reward_function(self.cur_state)
        # reward = self.reward_tpt_gain(self.cur_state)
        next_mcs = self.agent.choose_action(rl_input)
        act.next_mcs = next_mcs
        print(f'[Training] mcs: {self.last_state["mcs"]} -> {self.last_action["next_mcs"]}\t, cur_tpt: {self.cur_state["throughput"]:.2f} Mbps, FER: {self.cur_state["fer"]:.2f}, reward: {reward:.2f}, Snr: {self.cur_state["snr"]:.2f}')

        # last_one_hot[self.last_state['mcs']] = 1  # Set the index of the input value to 1
        last_rl_input = [self.last_state['snr']]
        # self.agent.remember(last_rl_input,
        #                     self.last_action['next_mcs'], 
        #                     reward, 
        #                     rl_input,
        #                     self.isFinished)
        self.agent.rewards.append(reward)



        for key in self.cur_state:
            self.last_state[key] = self.cur_state[key]
        self.last_action['next_mcs'] = next_mcs

        return act
    def train_reinforce(self, env: AiConstantRateEnv, act: AiConstantRateAct) -> AiConstantRateAct:
        if self.isInference:
            return self.inference_reinforce(env, act)

        scaled_snr = np.log(env.snr+1).astype(np.float32)
        if self.isFirstEpisode == True :
            self.last_state['mcs'] = env.mcs
            rl_input = [scaled_snr]
            next_mcs = self.agent.choose_action(rl_input)
            self.last_action['next_mcs'] = next_mcs
            act.next_mcs = next_mcs
            self.isFirstEpisode = False
            return act

        reward = self.reward_mcs(self.last_action['next_mcs'], env.max_mcs)
        self.agent.rewards.append(reward)
        self.agent.update()
        print(f'[Training] mcs: {self.last_state["mcs"]} -> {self.last_action["next_mcs"]}\t, cur_tpt: {env.throughput:.2f} Mbps, reward: {reward:.2f}, Snr: {scaled_snr:.2f}')
        rl_input = [scaled_snr]
        next_mcs = self.agent.choose_action(rl_input)
        
        act.next_mcs = env.max_mcs
        # act.next_mcs = next_mcs

        self.last_state['mcs'] = env.mcs
        self.last_state['throughput'] = env.throughput
        self.last_action['next_mcs'] = next_mcs
        return act

    def inference_reinforce(self, env: AiConstantRateEnv, act: AiConstantRateAct) -> AiConstantRateAct:
            scaled_snr = np.log(env.snr+1).astype(np.float32)
            rl_input = [scaled_snr]
            next_mcs = self.agent.choose_action(rl_input)
            
            act.next_mcs = next_mcs
            # if env.max_mcs == 0:
                # print(f'snr(0) ={env.snr}')

            # print(f'[Inference] mcs: {env.mcs}\t, cur_tpt: {env.throughput:.2f} Mbps, Snr: {scaled_snr:.2f}')
            return act        
    def inference_dqn(self, env: AiConstantRateEnv, act: AiConstantRateAct) -> AiConstantRateAct:
            # scale snr by 100 
            self.cur_state['throughput'] = env.throughput
            self.cur_state['cw'] = 0
            self.cur_state['mcs'] = env.mcs
            self.cur_state['snr'] = np.log(env.snr+1)


            # todo: inaccurate one_hot
            one_hot = [0]*8  # Initialize the array with zeros
            # one_hot[self.cur_state['mcs']] = 1  # Set the index of the input value to 1
            rl_input = [self.cur_state['snr']]
            next_mcs = self.agent.choose_action(rl_input)
            
            # next_mcs = 1
            act.next_mcs = next_mcs

            # print(f'[Inference] mcs: {self.cur_state["mcs"]} -> {next_mcs}\t, cur_tpt: {self.cur_state["throughput"]:.2f} Mbps, FER: {self.cur_state["fer"]:.2f}, Snr: {self.cur_state["snr"]:.2f}')
            return act

    def calculate_max_throughput(self, mcs_index, channel_bandwidth):
        """
        Calculate the theoretical max throughput for 802.11n with a single spatial stream and a long guard interval.
        
        Args:
        mcs_index: The MCS index (0-7 for 802.11n with a single spatial stream)
        channel_bandwidth: The channel bandwidth in MHz (20 or 40 for 802.11n)
        
        Returns:
        The theoretical max throughput in Mbps.
        """
        
        # Data rates for 802.11n with a single spatial stream and a long guard interval
        data_rates_20MHz = [5.4, 9.8, 13.6, 16.9, 22.2, 26.1, 28.1, 29.6]  # in Mbps
        data_rates_40MHz = [13.5, 27, 40.5, 54, 81, 108, 121.5, 135]  # in Mbps

        if channel_bandwidth == 20:
            max_data_rate = data_rates_20MHz[mcs_index]
        elif channel_bandwidth == 40:
            max_data_rate = data_rates_40MHz[mcs_index]
        else:
            raise ValueError("Invalid channel bandwidth. It should be either 20 or 40 MHz for 802.11n.")

        return max_data_rate


    def calculate_reward(self, achieved_throughput, mcs_index, channel_bandwidth):
        """
        Calculate the reward based on the difference between the achieved and theoretical max throughput.
        
        Args:
        achieved_throughput: The achieved throughput in Mbps
        mcs_index: The MCS index (0-7 for 802.11n with a single spatial stream)
        channel_bandwidth: The channel bandwidth in MHz (20 or 40 for 802.11n)
        
        Returns:
        The reward, a number between 0 and 1.
        """
        # Calculate the theoretical max throughput
        max_throughput = self.calculate_max_throughput(mcs_index, channel_bandwidth)

        # Calculate the reward as 1 divided by (1 + difference)
        reward = achieved_throughput / max_throughput 
        reward = mcs_index/7.0 * reward**3
        return reward

    def reward_mcs(self, mcs, max_mcs):
        return 0.1 if mcs == max_mcs else 0
        
    def reward_tpt_gain(self, tpt):
        if tpt - self.last_state['throughput']>0:
            reward = 1
        else:
            reward = -1

        reward = self.cur_state['mcs']/7.0 * reward**3
        return reward

    def save_model(self, filename):
        self.agent.save_model(filename)
    def load_model(self, filename, retrain=False):
        self.agent.load_model(filename)
        if retrain:
            self.agent.train()
        else:
            self.agent.eval()


if __name__ == '__main__':
    # ns3Settings = {"apManager":"ns3::rl-rateWifiManager","staManager":"ns3::rl-rateWifiManager"}
    # ns3Settings = {"apManager":"ns3::MinstrelHtWifiManager","staManager":"ns3::MinstrelHtWifiManager"}
    ns3Settings = {"apManager":"ns3::IdealWifiManager","staManager":"ns3::IdealWifiManager"}
    mempool_key = 1346 # memory pool key, arbitrary integer large than 1000
    mem_size = 4096 # memory pool size in bytes
    exp = Experiment(mempool_key, mem_size, 'rate-control', '../../', using_waf=False)
    exp.reset()

    memblock_key = 2335 # memory block key in the memory pool, arbitrary integer, and need to keep the same in the ns-3 script
    c = AiConstantRateContainer(memblock_key,model_name='model_18_fine_50k_02.pt', retrain=False)
    # c = AiConstantRateContainer(memblock_key)
    iters_count = 0

    pro = exp.run(setting=ns3Settings, show_output=True)
    print("run rate-control", ns3Settings)
    epochs = 50000
    isTqdm = False 
    if isTqdm:
        with tqdm(total=epochs) as pbar:
            while not c.rl.isFinish():
                with c.rl as data:
                    if data == None:
                        break
                    data.act = c.train_reinforce(data.env, data.act)
                    pbar.update(1)
                    if iters_count > epochs:
                        c.save_model('model_18_fine_50k_03.pt')
                        print('model saved')
                        break
                    iters_count += 1
    else:
        while not c.rl.isFinish():
            with c.rl as data:
                if data == None:
                    break
                data.act = c.train_reinforce(data.env, data.act)
                if iters_count > epochs:
                    c.save_model('model_18_fine_50k_03.pt')
                    print('model saved')
                    break
                iters_count += 1
    pro.wait()
    del exp
