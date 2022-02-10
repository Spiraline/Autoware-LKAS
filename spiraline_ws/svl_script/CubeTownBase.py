#!/usr/bin/python3
import socket
import yaml
import lgsvl
from lgsvl.geometry import Transform, Vector
from time import sleep


class Exp(object):
	def __init__(self):
		# load cfg file
		cfg_file = __file__.replace('.py', '.yaml')
		with open(cfg_file, 'r') as f:
			self.cfg = yaml.load(f, Loader=yaml.FullLoader)
		
		self.sim = lgsvl.Simulator(
		    address=self.cfg['simulator']['address'],
		    port=self.cfg['simulator']['port'])

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

	def wait_for_bridge(self):
		bridge_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		location = (self.cfg['lgsvl_bridge']['address'], self.cfg['lgsvl_bridge']['port'])
		
		socket_open = bridge_socket.connect_ex(location)

		while socket_open:
			print('[System] Wait for rosbridge')
			sleep(2)
			socket_open = bridge_socket.connect_ex(location)

		print('[System] Bridge connected')

		bridge_socket.close()

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

	def setup_sim(self):
		self.wait_for_bridge()
		self.create_ego(self.sim)
		# self.create_npc(self.sim)
		# self.create_pedestrian(self.sim)

	def run(self):
		self.sim.reset()
		self.setup_sim()
		print('[System] Simulation Starts')
		self.sim.run()

if __name__ == '__main__':
	e = Exp()
	e.run()
	exit()
