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
sx = spawns[0].position.x
sz = spawns[0].position.z

spawns = sim.get_spawn()

state = lgsvl.AgentState()
# state.transform.position = spawns[0].position
state.transform.position = spawns[0].position + 120 * forward
state.transform.rotation = spawns[0].rotation

ego = sim.add_agent("SingleLiDAR (Autoware)", lgsvl.AgentType.EGO, state)
ego.connect_bridge(os.environ.get("BRIDGE_HOST", "127.0.0.1"), 9090)


#------- Stand vehicle -------#
#set stand vehicle's initial position
sv_state = lgsvl.AgentState()
sv_state.transform.position = spawns[0].position + 200 * forward - 2.5 * right
sv_state.transform.rotation = spawns[0].rotation

stand_vehicle = sim.add_agent("Sedan", lgsvl.AgentType.NPC, sv_state)

sv_state_2 = lgsvl.AgentState()
sv_state_2.transform.position = spawns[0].position + 200 * forward + 2.5 * right
sv_state_2.transform.rotation = spawns[0].rotation

stand_vehicle_2 = sim.add_agent("Sedan", lgsvl.AgentType.NPC, sv_state_2)

sv_state_3 = lgsvl.AgentState()
sv_state_3.transform.position = spawns[0].position + 250 * forward + 2.5 * right
sv_state_3.transform.rotation = spawns[0].rotation

stand_vehicle_3 = sim.add_agent("Sedan", lgsvl.AgentType.NPC, sv_state_3)

sv_state_4 = lgsvl.AgentState()
sv_state_4.transform.position = spawns[0].position + 250 * forward - 2.5 * right
sv_state_4.transform.rotation = spawns[0].rotation

stand_vehicle_4 = sim.add_agent("Sedan", lgsvl.AgentType.NPC, sv_state_4)

sv_state_5 = lgsvl.AgentState()
sv_state_5.transform.position = spawns[0].position + 150 * forward + 2.5 * right
sv_state_5.transform.rotation = spawns[0].rotation

stand_vehicle_5 = sim.add_agent("Sedan", lgsvl.AgentType.NPC, sv_state_5)

sv_state_6 = lgsvl.AgentState()
sv_state_6.transform.position = spawns[0].position + 150 * forward - 2.5 * right
sv_state_6.transform.rotation = spawns[0].rotation

stand_vehicle_6 = sim.add_agent("Sedan", lgsvl.AgentType.NPC, sv_state_6)

sim.run()
