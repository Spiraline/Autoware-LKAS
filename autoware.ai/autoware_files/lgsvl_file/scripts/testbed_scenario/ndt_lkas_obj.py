#!/usr/bin/env python3
#
# Copyright (c) 2019 LG Electronics, Inc.
#
# This software contains code licensed as described in LICENSE.
#

import os
import lgsvl
import random
import time
from pathlib import Path
import json

sim = lgsvl.Simulator(os.environ.get("SIMULATOR_HOST", "127.0.0.1"), 8181)

layer_mask = 0
layer_mask |= 1 << 0  # 0 is the layer for the road (default)

if sim.current_scene == "Testbed":
  sim.reset()
else:
  sim.load("Testbed")

spawns = sim.get_spawn()

forward = lgsvl.utils.transform_to_forward(spawns[0])
right = lgsvl.utils.transform_to_right(spawns[0])

state = lgsvl.AgentState()
state.transform.position = spawns[0].position + 150 * forward
state.transform.rotation = spawns[0].rotation

ego = sim.add_agent("SingleLiDAR (Autoware)", lgsvl.AgentType.EGO, state)
ego.connect_bridge(os.environ.get("BRIDGE_HOST", "127.0.0.1"), 9090)


#------- Stand vehicle -------#
#set stand vehicle's initial position


sv_state_arr = []

# for (x, y) in pose_arr:
#   sv_state_arr.append(lgsvl.AgentState())
#   sv_state_arr[-1].transform.position = spawns[0].position + y * forward + x * right
#   sv_state_arr[-1].transform.rotation = spawns[0].rotation

  # _ = sim.add_agent("Sedan", lgsvl.AgentType.NPC, sv_state_arr[-1])

for i in range(30):
  sv_state_arr.append(lgsvl.AgentState())
  sv_state_arr[-1].transform.position = spawns[0].position + (150 + i * 7) * forward - 3.5 * right
  sv_state_arr[-1].transform.rotation = spawns[0].rotation

  _ = sim.add_agent("Sedan", lgsvl.AgentType.NPC, sv_state_arr[-1])

for i in range(30):
  sv_state_arr.append(lgsvl.AgentState())
  sv_state_arr[-1].transform.position = spawns[0].position + (150 + i * 7) * forward + 3.5 * right
  sv_state_arr[-1].transform.rotation = spawns[0].rotation

  _ = sim.add_agent("Sedan", lgsvl.AgentType.NPC, sv_state_arr[-1])

for i in range(30):
  sv_state_arr.append(lgsvl.AgentState())
  sv_state_arr[-1].transform.position = spawns[0].position + (150 + i * 7) * forward - 6 * right
  sv_state_arr[-1].transform.rotation = spawns[0].rotation

  _ = sim.add_agent("Sedan", lgsvl.AgentType.NPC, sv_state_arr[-1])


sim.run()
