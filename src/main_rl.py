import gym
from gym import spaces
import os
from tkinter import *
from queue import PriorityQueue

# Constants
FILE_PATH = "../benchmarks/oswald.infile"  # Path to the file with info about the circuit to route
NET_COLOURS = ["red", "yellow", "grey", "orange", "purple", "pink", "green", "medium purple", "white"]

# Global variables
net_dict = {}  # Dictionary of nets, keys are the net numbers
net_pq = PriorityQueue()  # PriorityQueue for nets to route
active_net = None  # Reference to Net that is currently being routed
target_sink = None  # Reference to a Cell (sink) that is currently the target for routing
wavefront = None  # List of cells composing of the propagating wavefront
routing_array = []  # 2D list of cells
text_id_list = []  # A list of Tkinter IDs to on-screen text
net_order = []  # Holds to order for the current routing attempt
failed_nets = []  # List of nets that failed to fully route in the current attempt
best_priority_set = []  # Tracks the net priorities from the net ordering that yields the best (failed) circuit outcome
starting_priority_set = []  # The net priorities at the start of the current routing attempt
array_width = 0  # Width of routing array
array_height = 0  # Height of routing array
current_net_order_idx = 0  # Index for the current net in net_order to route
best_num_segments_routed = 0  # Tracks the net ordering that yields the best (failed) circuit outcome
num_segments_routed = 0  # Number of segments routed in the current routing attempt
all_nets_routed = True  # Have all nets been routed? Assume true, prove false if a net fails to route
done_routing_attempt = False  # Has the current attempt at routing the circuit completed?
done_circuit = False  # Has the circuit been routed? (Or determined unroutable?)
final_route_initiated = False  # Is the current routing attempt definitely the last one?
circuit_is_hard = False  # Did the circuit fail to route on the first attempt?


class RouterEnv(gym.Env):
    """
    Maze Routing Environment
    """
    def __init__(self):
        super(RouterEnv, self).__init__()
        self.action_space = None
        self.observation_space = None

    def step(self, action):
        observation = None
        reward = None
        done = None
        info = None
        return observation, reward, done, info

    def reset(self):
        observation = None
        return observation

    def render(self, mode='human'):
        pass

    def close(self):
        pass