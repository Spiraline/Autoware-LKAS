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

sim = lgsvl.Simulator(os.environ.get("SIMULATOR_HOST", "127.0.0.1"), 8181)
if sim.current_scene == "BorregasAve":
  sim.reset()
else:
  sim.load("BorregasAve")

spawns = sim.get_spawn()
forward = lgsvl.utils.transform_to_forward(spawns[0])
right = lgsvl.utils.transform_to_right(spawns[0])


sx = spawns[0].position.x
sz = spawns[0].position.z

spawns = sim.get_spawn()

state = lgsvl.AgentState()
state.transform = spawns[0]
# state = lgsvl.AgentState()
# state.transform.position = spawns[0].position + 40 * forward
# state.transform.rotation = spawns[0].rotation

ego = sim.add_agent("Lexus2016RXHybrid (Autoware)", lgsvl.AgentType.EGO,state)
ego.connect_bridge(os.environ.get("BRIDGE_HOST", "127.0.0.1"), 9090)


state = lgsvl.AgentState()
# Spawn the pedestrian in front of car
state.transform.position = spawns[0].position + 50 * forward
state.transform.rotation = spawns[0].rotation

p = sim.add_agent("Bob", lgsvl.AgentType.PEDESTRIAN, state)
for sensor in ego.get_sensors():
  if sensor.name == "Lidar":
    sensor.save("lidar.pcd")

sim.run()