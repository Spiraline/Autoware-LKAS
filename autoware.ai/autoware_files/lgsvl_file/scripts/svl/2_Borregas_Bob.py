#!/usr/bin/python3
import lgsvl
from lgsvl.geometry import Transform
import tqdm
import os
import random
import yaml


class Exp(object):
	def __init__(self):
		# load cfg file
		cfg_file = __file__.replace('.py', '.yaml')
		with open(cfg_file, 'r') as f:
			self.cfg = yaml.load(f, Loader=yaml.FullLoader)
		
		self.sim = lgsvl.Simulator(
		    address=self.cfg['simulator']['address'],
		    port=self.cfg['simulator']['port'])
		# self.sim = lgsvl.Simulator(os.environ.get("SIMULATOR_HOST","127.0.0.1"), 8181)


		# reset scene
		target_scene = self.cfg['simulator']['scene']
		if self.sim.current_scene == target_scene:
			self.sim.reset()
		else:
			self.sim.load(target_scene)
		# calc position
		spawns = self.sim.get_spawn()
		# calc offset
		self.origin = Transform(
			spawns[0].position,
			spawns[0].rotation)
		self.origin.position.x += self.cfg['origin']['offset']['x']
		self.origin.position.y += self.cfg['origin']['offset']['y']
		self.origin.position.z += self.cfg['origin']['offset']['z']
		self.origin.rotation.y += self.cfg['origin']['offset']['r']

		self.u_forward = lgsvl.utils.transform_to_forward(self.origin)
		self.u_right = lgsvl.utils.transform_to_right(self.origin)

		# tracking info
		self.collisions = []

	def create_ego(self, sim):
		# ego (main car)
		ego_state = lgsvl.AgentState()
		ego_state.transform = \
			Transform(self.origin.position, self.origin.rotation)
		ego = sim.add_agent(self.cfg['ego']['asset-id'],
			lgsvl.AgentType.EGO, ego_state)
		
		ego.connect_bridge(
		    self.cfg['lgsvl_bridge']['address'],
		    self.cfg['lgsvl_bridge']['port'])		

		def ego_collision(agent1, agent2, contact):
			self.collisions.append([agent1, agent2, contact])
			return

		ego.on_collision(ego_collision)
		return

	def create_npc_waypoint(self, npc_state):
		# create waypoint (npc0)
		w = []  # waypoint
		w_length = self.cfg['waypoint']['length']
		w_interval = self.cfg['waypoint']['interval']
		idle_time = self.cfg['waypoint']['idle_time']

		for i in range(w_length):
			offset = (i + 1) * w_interval * self.u_forward
			stop_every = self.cfg['waypoint']['stop_every']
			stop_for = self.cfg['waypoint']['stop_for']
			if i % stop_every >= (stop_every - stop_for):
				speed = self.cfg['waypoint']['stop_speed']
			else:
				speed = self.cfg['waypoint']['cruise_speed']

			wp = lgsvl.DriveWaypoint(
				position=npc_state.transform.position + offset,
				speed=speed,
				angle=npc_state.transform.rotation,
				idle=idle_time)
			w.append(wp)
		return w

	def create_npc(self, sim):
		print(self.cfg['npc'])

		# npc
		npc_state = lgsvl.AgentState()
		npc_state.transform = \
			Transform(self.origin.position, self.origin.rotation)
		npc_state.transform.position += \
			self.cfg['npc'][0]['offset']['forward'] * self.u_forward
		
		npc = sim.add_agent(
			self.cfg['npc'][0]['type'],
			lgsvl.AgentType.NPC, npc_state)

		w = self.create_npc_waypoint(npc_state)
		npc.follow(waypoints=w, loop=False)
		return

	def create_pedestrian(self, sim):
		# pedestrian
		pedestrian_state = lgsvl.AgentState()
		pedestrian_state.transform = \
			Transform(self.origin.position, self.origin.rotation)
		pedestrian_state.transform.position += \
			self.cfg['pedestrian'][0]['offset']['forward'] * self.u_forward
		
		pedestrian = sim.add_agent(
			self.cfg['pedestrian'][0]['type'],
			lgsvl.AgentType.PEDESTRIAN, pedestrian_state)
		
		return

	def setup_sim(self):
		self.create_ego(self.sim)
		# self.create_npc(self.sim)
		self.create_pedestrian(self.sim)
		return

	def run(self):
		for exp_iter in range(self.cfg['exp']['iteration']):
			self.sim.reset()			
			self.setup_sim()			
			# collisions = []
			print('starting exp #{}'.format(exp_iter))

			if(self.cfg['simulator']['timeout'] == 0):
				self.sim.run()
			else:
				for _ in tqdm.tqdm(range(self.cfg['simulator']['timeout'])):
					self.sim.run(1)

		print('self.collisions: {}'.format(len(self.collisions)))
		print('success')


if __name__ == '__main__':
	e = Exp()
	e.run()
	exit()
