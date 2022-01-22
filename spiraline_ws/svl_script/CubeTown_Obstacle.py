#!/usr/bin/python3
import lgsvl
from lgsvl.geometry import Transform, Vector
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
		self.u_up = lgsvl.utils.transform_to_up(self.origin)

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

	def create_npc(self, sim):
		for car in self.cfg['npc']:
			# npc
			npc_state = lgsvl.AgentState()
			npc_state.transform = \
				Transform(self.origin.position, self.origin.rotation)
			npc_state.transform.position += \
				car['offset']['forward'] * self.u_forward

			npc_state.transform.position += \
				car['offset']['right'] * self.u_right

			if 'up' in car['offset']:
				npc_state.transform.position += \
				car['offset']['up'] * self.u_up

			npc_state.transform.rotation.y = car['offset']['rotation']
			
			sim.add_agent(
				car['type'],
				# lgsvl.AgentType.NPC, npc_state, color=Vector(200, 0, 255))
				lgsvl.AgentType.NPC, npc_state, color=Vector(0, 0, 0))

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
		self.create_npc(self.sim)
		# self.create_pedestrian(self.sim)
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
