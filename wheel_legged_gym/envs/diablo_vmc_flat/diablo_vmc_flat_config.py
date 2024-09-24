# SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2021 ETH Zurich, Nikita Rudin

from wheel_legged_gym.envs.diablo_vmc.diablo_vmc_config import (
    DiabloVMCCfg,
    DiabloVMCCfgPPO,
)


class DiabloVMCFlatCfg(DiabloVMCCfg):

    class terrain(DiabloVMCCfg.terrain):
        mesh_type = "plane"
    class rewards(DiabloVMCCfg.rewards):
        class scales:
            tracking_lin_vel = 1.0
            tracking_lin_vel_enhance = 1
            tracking_ang_vel = 1.0

            base_height = 5.0
            nominal_state = -0.2
            lin_vel_z = -1
            ang_vel_xy = -0.05
            orientation = -300.0

            dof_vel = -5e-5
            dof_acc = -2.5e-7
            torques = -0.1e-5
            action_rate = -0.01
            action_smooth = -0.01

            collision = -1000.0
            dof_pos_limits = -3

            theta_limit = -0.01
            same_l = -0.01

        base_height_target = 0.30

class DiabloVMCFlatCfgPPO(DiabloVMCCfgPPO):

    class runner(DiabloVMCCfgPPO.runner):
        # logging
        # policy_class_name = (
        #     "ActorCriticSequence"  # could be ActorCritic, ActorCriticSequence
        # )
        experiment_name = "diablo_vmc_flat"
        max_iterations = 30000
