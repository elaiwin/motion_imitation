# coding=utf-8
# Copyright 2020 The Google Research Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Simple openloop trajectory generators."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import attr
from gym import spaces
import numpy as np


class SimpleTG(object):
    """A trajectory generator that return constant motor angles."""

    def __init__(
            self , phi_leg , delta_phi , alpha_tg , Ae , Cs , t_p, theta, z , h_tg, k_sle , phi_t , f_tg , beta
    ):

        # We will need this I think
        # self.action_space = spaces.Box(-action_high, action_high, dtype=np.float32)

        ##################################3
        # tunable parameters

        self._alpha_tg = alpha_tg
        self._Ae = Ae
        self._Cs = Cs
        self._theta = theta
        self._z = z
        self._h_tg = h_tg
        self._k_sle = k_sle
        self._delta_phi = delta_phi
        self._beta = beta

        # from upstream untunable
        self._phi_t = phi_t
        self._phi_leg = phi_leg
        self._t_p = t_p


        # upstream tunable
        self._f_tg = f_tg


    def reset(self):
        pass


    def _update_phi_leg(self, phi_t, phi_diff):
        phi_leg = np.mod(phi_t + phi_diff, 2.0 * np.pi)
        return phi_leg


    def _sync_phi_t(self, phi_t):
        self._phi_t = phi_t
        return


    def _compute_t_prime(self, phi_leg, beta):
        if 2 * np.pi * beta > phi_leg >= 0:
            t_p = phi_leg / (2.0 * beta)
        else:
            t_p = 2 * np.pi - (2 * np.pi - phi_leg) / (2 * (1 - beta))
        return t_p

    def _assemble_leg_height(self , phi_leg , beta , k_sle , t_p , Ae):
        if 2 * np.pi * beta > phi_leg >= 0:
            h_tg = self._h_tg
        else:
            h_tg = self._h_tg + (-k_sle * Ae * np.sin(t_p) )
        return h_tg



    def _genertate_trajectory(self, alpha_tg, Ae, Cs, t_p, h_leg, theta, z):
        x = Cs + alpha_tg * np.cos(t_p)
        y = h_tg + Ae * np.sin(t_p) + theta * np.cos(t_p)
        return np.array([x, y, z])



    def get_trajectory(self , phi_t):
        self._sync_phi_t(phi_t = phi_t)

        self._phi_leg = self._update_phi_leg(phi_t = self._phi_t , phi_diff = self._delta_phi)

        self._t_p = self._compute_t_prime(phi_leg=self._phi_leg,beta=self._beta)
        h_leg = self._assemble_leg_height(phi_leg=self._phi_leg,beta=self._beta,k_sle=self._k_sle,t_p=self._t_p,Ae=self._Ae)

        self._genertate_trajectory(alpha_tg = self._alpha_tg, Ae = self._Ae, Cs = self._Cs, t_p = self._t_p,
                                    h_leg = h_leg, theta = self._theta, z = self._z)


        #### TODO
        # get that Ik in here

        res = np.array([3])
        #### TODO

        return res

