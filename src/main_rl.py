"""
UBC CPEN 513 Project
Implements A* for multi-pin nets with Reinforcement Learning rip-up and reroute
Uses Tkinter for GUI.
"""

import gym
from gym import spaces
from stable_baselines3 import DQN
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.evaluation import evaluate_policy
import os
from tkinter import *
from queue import SimpleQueue
from queue import PriorityQueue
import numpy as np
import random
from enum import Enum
import time
import datetime

# Constants
VERBOSE_ENV = False
FILE_PATH = "../benchmarks/stdcell.infile"  # Path to the file with info about the circuit to route
NET_COLOURS = ["red", "grey", "orange", "purple", "pink", "green", "medium purple", "yellow", "white"]
CONG_FRAC_IDX_A = 0
CONG_FRAC_IDX_B = 1
FREE_ADJ_IDX_A = 2
FREE_ADJ_IDX_B = 3
HPWL_IDX_A = 4
HPWL_IDX_B = 5
MAX_CONG_IDX_A = 6
MAX_CONG_IDX_B = 7
Y_FREE_IDX_A = 8
Y_FREE_IDX_B = 9
X_FREE_IDX_A = 10
X_FREE_IDX_B = 11
LEARN_RATE = 0.2
EXPLORE_INIT = 1.0
EXPLORE_FINAL = 0.1
GAMMA = 0.9
TIME_STEPS = 100


# General variables
_root = None  # Tkinter root
_routing_canvas = None  # Tkinter canvas GUI
_debug_counter = 0

# Maze Routing variables
_net_dict = {}  # Dictionary of nets, keys are the net numbers
_net_queue = SimpleQueue()  # FIFO for nets to route
_active_net = None  # Reference to Net that is currently being routed
_target_sink = None  # Reference to a Cell (sink) that is currently the target for routing
_wavefront = None  # List of cells composing of the propagating wavefront
_routing_array = []  # 2D list of cells
_text_id_list = []  # A list of Tkinter IDs to on-screen text
_failed_nets = []  # List of nets that failed to fully route in the current attempt
_array_width = 0  # Width of routing array
_array_height = 0  # Height of routing array
_best_num_segments_routed = 0  # Tracks the net ordering that yields the best (failed) circuit outcome
_num_segments_routed = 0  # Number of segments routed in the current routing attempt
_all_nets_routed = True  # Have all nets been routed? Assume true, prove false if a net fails to route
_done_routing_attempt = False  # Has the current attempt at routing the circuit completed?
_done_circuit = False  # Has the circuit been routed? (Or determined unroutable?)
_final_route_initiated = False  # Is the current routing attempt definitely the last one?
_circuit_is_hard = False  # Did the circuit fail to route on the first attempt?
_init_uncongested_nets = 0

# Reinforcement Learning variables
_rl_model = None  # The RL Agent
_is_first_step = True  # Is this the first step of the RL agent since the last environment reset?
_step_count = 0  # Number of steps taken by the RL agent during program run
_ripup_candidate_a = None
_ripup_candidate_b = None
_rl_target_cell = None
_rl_env = None

# Observations: fraction of cells congested, number of available surrounding cells, bounding box of net, max congestion
_obs_hi = np.array([1.0, 1.0, 8, 8, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf])
_obs_low = np.zeros(_obs_hi.shape[0])


class Actions(Enum):
    RIP_A = 0
    RIP_B = 1


class RouterEnv(gym.Env):
    """
    Maze Routing Environment
    """
    def __init__(self):
        global _net_dict
        global _obs_low
        global _obs_hi

        super(RouterEnv, self).__init__()
        num_nets = len(_net_dict)
        if num_nets == 0:
            exit("ERROR: invalid number of nets in gym environment")
        self.action_space = spaces.Discrete(2)

        self.observation_space = spaces.Box(low=_obs_low, high=_obs_hi, dtype=float)

        print(self.observation_space)

    def step(self, action):
        global _step_count
        global _done_circuit
        global _root

        print("Step " + str(_step_count))
        _step_count += 1
        reward, observation = rl_action_step(action)
        done = _done_circuit
        info = {}
        return observation, reward, done, info

    def reset(self):
        global _net_dict
        global _net_queue
        global _active_net
        global _target_sink
        global _wavefront
        global _failed_nets
        global _best_num_segments_routed
        global _num_segments_routed
        global _all_nets_routed
        global _done_routing_attempt
        global _done_circuit
        global _final_route_initiated
        global _circuit_is_hard
        global _is_first_step
        global _ripup_candidate_a
        global _ripup_candidate_b
        global _rl_target_cell
        global _init_uncongested_nets

        print("RESET**********************************************************************************************")

        # Rip-up any and all nets in current environment
        _rl_target_cell = None
        for rip_up_net in _net_dict.values():
            if len(rip_up_net.wireCells) > 0:
                rip_up_id = rip_up_net.num
                rip_up_one(rip_up_id)

        # Reset all cells
        for column in _routing_array:
            for reset_cell in column:
                reset_cell.isRouted = False
                reset_cell.isWire = False
                reset_cell.isCandidate = False
                reset_cell.isCongested = False
                reset_cell.netCount = 0
                reset_cell.hasPropagated = False
                reset_cell.distFromSource = 0
                reset_cell.routingValue = 0
                reset_cell.nextCell = []
                reset_cell.prevCell = None
                reset_cell.blacklist = []

        # Reset necessary global variables
        while not _net_queue.empty():
            _net_queue.get()
        for net in _net_dict.values():
            # Refill net queue with all nets in circuit
            _net_queue.put(net)
        _active_net = None
        _target_sink = None
        _wavefront = None
        _failed_nets = []
        _best_num_segments_routed = 0
        _num_segments_routed = 0
        _all_nets_routed = True
        _done_routing_attempt = False
        _done_circuit = False
        _final_route_initiated = False
        _circuit_is_hard = False
        _init_uncongested_nets = 0

        # Get environment to the point where the RL agent needs to make a decision
        while not _done_routing_attempt:
            rl_routing_step()

        _init_uncongested_nets = count_uncongested_nets()

        _rl_target_cell = get_least_congested_cell()

        if _rl_target_cell:
            if VERBOSE_ENV:
                print("RL target cell is " + str(_rl_target_cell.x) + ", " + str(_rl_target_cell.y))
            # Pick two arbitrary nets for the next rip-up comparison
            _ripup_candidate_a = _rl_target_cell.netGroups[1]
            _ripup_candidate_b = _rl_target_cell.netGroups[2]
        else:
            _ripup_candidate_a = None
            _ripup_candidate_b = None
            if VERBOSE_ENV:
                print("No congested cells!")

        observation = get_rl_observation()

        return observation

    def render(self, mode='human'):
        global _root
        print("render")
        if _root is not None:
            _root.update_idletasks()  # Update the Tkinter GUI

    def close(self):
        print("close")
        pass


class Cell:
    """
    A single cell in the routing grid/array
    """

    def __init__(self, x=-1, y=-1, obstruction=False, source=False, sink=False, net_group=-1):
        if obstruction and (source or sink):
            print("Error: Bad cell created!")
        self.x = x
        self.y = y
        self.isObstruction = obstruction
        self.isSource = source
        self.isSink = sink
        self.netGroups = [net_group]  # Int indicating which nets the Cell belongs to
        self.id = -1  # Tkinter ID for a corresponding rectangle
        self.isRouted = False  # Has this cell been routed in the current attempt?
        self.isWire = False  # Is this cell part of a trace?
        self.isCandidate = False  # Is the cell a candidate for the current route?
        self.isCongested = False  # Is the cell congested?
        self.netCount = 0  # the number of nets routed through this cell
        self.hasPropagated = False  # Has this cell already been used in a wavefront propagation?
        self.distFromSource = 0  # Routing distance from corresponding source Cell
        self.routingValue = 0  # Value used in wavefront for A*
        self.nextCell = []  # Can have multiple "next" cells, because wavefront propagates in 4 directions
        self.prevCell = None  # Reference to previous cell in route, for backtrace
        # self.congest_hist = 0  # number of times cell has been congested
        # self.isOwned = False
        self.blacklist = []  # List of nets (by ID) that are not allowed to use this cell

    def get_coords(self): return self.x, self.y


class Net:
    """
    A collection of cells
    """

    def __init__(self, source: Cell = None, sinks: Cell = None, num=-1):
        if sinks is None:
            sinks = []
        self.source = source  # Source Cell
        self.sinks = sinks  # List of sink Cells
        self.wireCells = []  # List of wire Cells
        self.congestedCells = []  # number of congested cells in trace
        self.num = num  # The net number for this net (effectively the net's identifier)
        self.sinksRemaining = len(self.sinks)  # Number of sinks left to be routed in this net
        self.initRouteComplete = False  # Has the net's source been routed to at least one sink?

        if self.num == -1:
            print("ERROR: assign a net number to the newly-created net!")


def main():
    """
    Top-level main function
    :return: void
    """
    global _routing_canvas
    global _routing_array
    global _array_width
    global _array_height
    global FILE_PATH
    global _rl_model
    global _root
    global _rl_env
    global CONG_FRAC_IDX_A
    global CONG_FRAC_IDX_B
    global FREE_ADJ_IDX_A
    global FREE_ADJ_IDX_B
    global HPWL_IDX_A
    global HPWL_IDX_B
    global MAX_CONG_IDX_A
    global MAX_CONG_IDX_B
    global Y_FREE_IDX_A
    global Y_FREE_IDX_B
    global X_FREE_IDX_A
    global X_FREE_IDX_B

    # Set RNG seed
    random.seed(0)

    # Read input file
    script_path = os.path.dirname(__file__)
    true_path = os.path.join(script_path, FILE_PATH)
    routing_file = open(true_path, "r")

    # Setup the routing grid/array
    _routing_array = create_routing_array(routing_file)
    _array_width = len(_routing_array)
    _array_height = len(_routing_array[0])

    # set max bounding box size
    _obs_hi[HPWL_IDX_A] = _array_width + _array_height
    _obs_hi[HPWL_IDX_B] = _array_width + _array_height

    _obs_hi[MAX_CONG_IDX_A] = len(_net_dict)
    _obs_hi[MAX_CONG_IDX_B] = len(_net_dict)

    _obs_hi[Y_FREE_IDX_A] = _array_height
    _obs_hi[Y_FREE_IDX_B] = _array_height

    _obs_hi[X_FREE_IDX_A] = _array_width
    _obs_hi[X_FREE_IDX_B] = _array_width

    # Create routing canvas in Tkinter
    _root = Tk()
    _root.columnconfigure(0, weight=1)
    _root.rowconfigure(0, weight=1)
    cell_length = 15
    _routing_canvas = Canvas(_root, bg='white', width=_array_width * cell_length, height=_array_height * cell_length)
    _routing_canvas.grid(column=0, row=0, sticky=(N, W, E, S))
    for x in range(_array_width):
        for y in range(_array_height):
            # Add a rectangle to the canvas
            top_left_x = cell_length * x
            top_left_y = cell_length * y
            bottom_right_x = top_left_x + cell_length
            bottom_right_y = top_left_y + cell_length
            route_cell = _routing_array[x][y]
            net_idx = route_cell.netGroups[-1]
            if route_cell.isObstruction:
                rectangle_colour = "blue"
            elif route_cell.isCongested:
                rectangle_colour = "light blue"
            else:
                rectangle_colour = NET_COLOURS[net_idx]
            rectangle_coords = (top_left_x, top_left_y, bottom_right_x, bottom_right_y)
            route_cell.id = _routing_canvas.create_rectangle(rectangle_coords, fill=rectangle_colour)
            if route_cell.isSource or route_cell.isSink:
                # Place text inside the rect to show its routing value
                text_x = (top_left_x + bottom_right_x) / 2
                text_y = (top_left_y + bottom_right_y) / 2
                if route_cell.routingValue > 99:
                    text_size = 7
                else:
                    text_size = 10
                _routing_canvas.create_text(text_x, text_y, font=("arial", text_size),
                                            text=str(route_cell.netGroups[-1]), fill='white')


    _rl_env = RouterEnv()

    # Event bindings and Tkinter start
    _routing_canvas.focus_set()
    _routing_canvas.bind("<Key>", lambda event: key_handler(event))
    _root.mainloop()


def key_handler(event):
    """
    Accepts a key event and makes an appropriate decision.
    :param event: Key event
    :return: void
    """
    global _routing_canvas
    global _rl_model
    global _is_first_step
    global _rl_env
    global _rl_target_cell
    global _step_count
    global LEARN_RATE
    global EXPLORE_INIT
    global EXPLORE_FINAL
    global GAMMA
    global TIME_STEPS

    e_char = event.char

    if e_char == '0':
        # Plain A* to completion
        algorithm_to_completion()
    elif str.isdigit(e_char):
        # Plain A*
        a_star_multistep(int(e_char))
    elif e_char == 'l':
        # RL Agent Learning pass

        # AI Gym Environment check
        #check_env(_rl_env)
        _step_count = 0  # Reset because check_env increments via step()

        # RL Agent
        _rl_model = DQN('MlpPolicy', _rl_env, verbose=1, learning_rate=LEARN_RATE, exploration_initial_eps=EXPLORE_INIT,
                        exploration_final_eps=EXPLORE_FINAL, gamma=GAMMA)
        print("Beginning RL training")
        _rl_model.learn(total_timesteps=TIME_STEPS)
        print("Finished RL training")
        print("Saving trained model")
        _rl_model.save("agent_" + time.strftime("%d-%m-%YT%H-%M-%S"))
    elif e_char == 't':
        # RL Agent Testing pass

        # AI Gym Environment check
        #check_env(_rl_env)
        _step_count = 0  # Reset because check_env increments via step()

        print("Loading trained model")
        _rl_model = DQN.load("target_model")

        obs = _rl_env.reset()
        done = False
        while not done:
            rl_action, states = _rl_model.predict(obs, deterministic=True)
            print("Action ", rl_action)
            obs, rewards, done, info = _rl_env.step(rl_action)
    elif e_char == 'r':
        # RL flow debugging (no agent involved, emulate actions randomly)
        if _is_first_step:
            _rl_env.reset()
            _is_first_step = False
        else:
            rand_action = random.randrange(1)
            rl_action_step(rand_action)
    else:
        pass


def count_uncongested_nets() -> int:
    """
    Get the number of nets in the circuit that have zero congested cells
    :return: net count
    """
    global _net_dict
    global _root

    uncongested = 0
    for net in _net_dict.values():
        if net.num not in _failed_nets:  # Don't consider an unroutable net as "uncongested"
            if not net.congestedCells:
                uncongested += 1
            if net.sinksRemaining != 0:
                print("Error: Bad call to count_uncongested_nets()")

    return uncongested


def rl_action_step(action):
    """
    Execute the program up to the point of the next Reinforcement Learning agent action.
    """
    global _root
    global _done_routing_attempt
    global _all_nets_routed
    global _all_nets_routed
    global _done_circuit
    global _active_net
    global _is_first_step
    global _debug_counter
    global _ripup_candidate_a
    global _ripup_candidate_b
    global _rl_target_cell
    global _init_uncongested_nets

    reward = 0

    if not _rl_target_cell:
        print("No RL cell to act on!")
        return reward, get_rl_observation()

    if _rl_target_cell.netCount > 1:
        # Rip-up the net decided by the RL action
        if action == Actions.RIP_A.value:
            if VERBOSE_ENV:
                print("Ripping up A")
            rip_up_one(_ripup_candidate_a)
        elif action == Actions.RIP_B.value:
            if VERBOSE_ENV:
                print("Ripping up B")
            rip_up_one(_ripup_candidate_b)
        else:
            print("Invalid action!")
    else:
        print("ERROR: Useless RL action input!")

    if _rl_target_cell.netCount == 1:
        # Previous rip-up resolved congestion for the target cell

        c_cell = get_least_congested_cell()
        if c_cell is None:
            _done_routing_attempt = False
            # No congested cells remain
            # Perform another round of routing
            if VERBOSE_ENV:
                print("Performing a routing attempt.")
            while not _done_routing_attempt:
                rl_routing_step()

            uncongested_new = count_uncongested_nets()

            reward += (uncongested_new - _init_uncongested_nets)
            # TODO: Consider fraction of uncongested nets as reward, may better account for failed/unroutable nets

            c_cell = get_least_congested_cell()
            _all_nets_routed = c_cell is None
            if _all_nets_routed:
                if len(_failed_nets) > 0:
                    # At least one route was blocked
                    print("Circuit could not be fully routed. Routed " + str(_num_segments_routed) + " segments.")
                    _done_circuit = True
                    reward += -10
                else:
                    # Successful route
                    print("Circuit routed successfully.")
                    _done_circuit = True
                    reward += 10
            else:
                # Attempt route again with new congestion (blacklist) settings
                _rl_target_cell = c_cell
                if VERBOSE_ENV:
                    print("RL target cell is " + str(_rl_target_cell.x) + ", " + str(_rl_target_cell.y))
                _done_routing_attempt = False
                _active_net = None
        else:
            # Move to next congested cell
            _rl_target_cell = c_cell
            if VERBOSE_ENV:
                print("RL target cell is " + str(_rl_target_cell.x) + ", " + str(_rl_target_cell.y))
            # Pick two arbitrary nets for the next rip-up comparison
            _ripup_candidate_a = c_cell.netGroups[1]
            _ripup_candidate_b = c_cell.netGroups[2]

    else:
        # RL agent needs to perform more rip-ups
        # Form new pair for next rip-up comparison
        # Only need to replace the net that was ripped up by the agent's most recent decision
        if action == Actions.RIP_A:
            for net_group in _rl_target_cell.netGroups:
                if net_group is not _ripup_candidate_b:
                    _ripup_candidate_a = net_group
        else:
            for net_group in _rl_target_cell.netGroups:
                if net_group is not _ripup_candidate_a:
                    _ripup_candidate_b = net_group

    observation = get_rl_observation()

    return reward, observation


def rl_routing_step():
    """
    Perform one iteration of A*, including algorithm overheads without doing any rip-ups
    :return: void
    """
    global _root
    global _routing_canvas
    global _active_net
    global _done_routing_attempt
    global _net_dict
    global _wavefront
    global _target_sink
    global _all_nets_routed
    global _failed_nets
    global _done_circuit
    global _final_route_initiated
    global _debug_counter

    if _done_circuit:
        return

    # Check if the current routing attempt is complete
    if _done_routing_attempt:
        return

    # Set a net to route if none already set
    if _active_net is None:
        _active_net = _net_queue.get()
        # Check that the retrieved net is not routed
        # This should be true by construction, as only unrouted nets are to be placed in the queue
        if _active_net.sinksRemaining == 0:
            print("ERROR: Attempted to route a completed net")

    # Set wavefront if none is set
    if _wavefront is None:
        _target_sink, best_start_cell = find_best_routing_pair()
        # Start from source cell by default
        _wavefront = [_active_net.source.get_coords()]

        # Add routed cells for this net
        for cell in _active_net.wireCells:
            _wavefront.append((cell.x, cell.y))

    # Check if the wavefront still contains cells
    if len(_wavefront) == 0:
        # No more available cells for wavefront propagation in this net
        # This net cannot be routed
        # Move on to next net

        if _active_net.num not in _failed_nets:
            _failed_nets.append(_active_net.num)  # Add this net to the list of failed nets

        print("Failed to route net " + str(_active_net.num) + " with colour " + NET_COLOURS[_active_net.num])
        _all_nets_routed = False
        _wavefront = None  # New wavefront will be created for next net

        cleanup_candidates()

        if _net_queue.empty():
            # All nets are routed
            print("Routing attempt complete")
            print("Failed nets: " + str(_failed_nets))
            _done_routing_attempt = True
            _active_net = None
        else:
            # Move on to next net
            _active_net = _net_queue.get()

        return

    # Run A*
    a_star_step()


def calc_bb_hpwl(net_id: int) -> int:
    """
    Calculate the Half-Perimeter WireLength of the bounding box of a net
    :param net_id: The int ID of the target net
    :return: The HPWL
    """
    net = _net_dict[net_id]

    max_x = net.source.x
    min_x = net.source.x
    max_y = net.source.y
    min_y = net.source.y

    for cell in net.sinks:
        if cell.x > max_x:
            max_x = cell.x
        elif cell.x < min_x:
            min_x = cell.x

        if cell.x > max_x:
            max_x = cell.x
        elif cell.x < min_x:
            min_x = cell.x

    return (max_x - min_x) + (max_y - min_y)


def free_adj(net_id: int) -> int:
    """
    Checks number of cells adjacent (including diagonal) to the target cell that a given net can route through.
    :param net_id: The int ID of the net to check for
    :return: int - number of free (routable) adjacent cells
    """
    global _rl_target_cell
    global _routing_array

    num_free = 0

    for off_x in [-1, 0, 1]:
        for off_y in [-1, 0, 1]:
            if off_x == 0 and off_y == 0:
                continue

            check_cell = _routing_array[_rl_target_cell.x + off_x][_rl_target_cell.y + off_y]

            skip = net_id in check_cell.blacklist or check_cell.isSink or check_cell.isSource or \
                check_cell.isObstruction

            if not skip:
                num_free += 1

    return num_free


def max_congest(net_id: int) -> int:
    """
    Find max degree of congestion in the net (i.e. the MOST congested cell involved that isn't this one)
    :param net_id: The int ID of the target net
    """
    global _net_dict

    max_c = 0

    # We use -1 here
    for cell in _net_dict[net_id].congestedCells:
        if cell != _rl_target_cell:
            if (len(cell.netGroups) - 1) > max_c:
                max_c = (len(cell.netGroups) - 1)

    return max_c


def get_freedom(net_id: int, x_dir: bool) -> int:
    global _rl_target_cell
    global _routing_array

    free = 0

    # Determine search bounds
    if x_dir:
        centre = _rl_target_cell.x
        high = _array_width
    else:
        # y_dir
        centre = _rl_target_cell.y
        high = _array_height
    low = -1

    # check positive direction (down or right) then negative direction (up or left)
    for start, bound, change in [(centre+1, high, 1), (centre-1, low, -1)]:
        for coord in range(start, bound, change):
            if x_dir:
                x_coord = coord
                y_coord = _rl_target_cell.y
            else:
                # y_dir
                x_coord = _rl_target_cell.x
                y_coord = coord

            check_cell = _routing_array[x_coord][y_coord]

            skip = net_id in check_cell.blacklist or check_cell.isSink or check_cell.isSource or \
                check_cell.isObstruction
            if skip:
                break

            free += 1

    return free


def get_rl_observation() -> np.ndarray:
    """
    Get an observation from the environment for the RL agent
    :return: observation
    """
    global _root
    global _routing_array
    global _obs_low
    global _net_dict
    global _ripup_candidate_a
    global _ripup_candidate_b
    global CONG_FRAC_IDX_A
    global CONG_FRAC_IDX_B
    global FREE_ADJ_IDX_A
    global FREE_ADJ_IDX_B
    global HPWL_IDX_A
    global HPWL_IDX_B
    global MAX_CONG_IDX_A
    global MAX_CONG_IDX_B
    global Y_FREE_IDX_A
    global Y_FREE_IDX_B
    global X_FREE_IDX_A
    global X_FREE_IDX_B

    observation_array = np.zeros(shape=(_obs_hi.shape[0],), dtype=float)

    if _ripup_candidate_a is None or _ripup_candidate_b is None:
        if VERBOSE_ENV:
            if _ripup_candidate_a is None:
                print("No ripup candidate A")
            else:
                print("Ripup candidate A is: ", _ripup_candidate_a)
            if _ripup_candidate_b is None:
                print("No ripup candidate B")
            else:
                print("Ripup candidate B is: ", _ripup_candidate_b)

        return observation_array
    if VERBOSE_ENV:
        print("----------------------------------------------------------------------")
        print('net a: ' + str(_ripup_candidate_a) + ' net b: ' + str(_ripup_candidate_b))

    observation_array[CONG_FRAC_IDX_A], observation_array[CONG_FRAC_IDX_B] = 0, 0

    # fraction of congested cells
    if len(_net_dict[_ripup_candidate_a].wireCells):
        observation_array[CONG_FRAC_IDX_A] = \
            len(_net_dict[_ripup_candidate_a].congestedCells) / len(_net_dict[_ripup_candidate_a].wireCells)
    if len(_net_dict[_ripup_candidate_b].wireCells):
        observation_array[CONG_FRAC_IDX_B] = \
            len(_net_dict[_ripup_candidate_b].congestedCells) / len(_net_dict[_ripup_candidate_b].wireCells)
    # number of free immediately adjacent cells
    observation_array[FREE_ADJ_IDX_A] = free_adj(_ripup_candidate_a)
    observation_array[FREE_ADJ_IDX_B] = free_adj(_ripup_candidate_b)
    # bounding box
    observation_array[HPWL_IDX_A] = calc_bb_hpwl(_ripup_candidate_a)
    observation_array[HPWL_IDX_B] = calc_bb_hpwl(_ripup_candidate_b)
    # max degree of congestion
    observation_array[MAX_CONG_IDX_A] = max_congest(_ripup_candidate_a)
    observation_array[MAX_CONG_IDX_B] = max_congest(_ripup_candidate_b)
    # y freedom (contiguous cells up/down)
    observation_array[Y_FREE_IDX_A] = get_freedom(_ripup_candidate_a, False)
    observation_array[Y_FREE_IDX_B] = get_freedom(_ripup_candidate_b, False)
    # x freedom (contiguous cells l/r)
    observation_array[X_FREE_IDX_A] = get_freedom(_ripup_candidate_a, True)
    observation_array[X_FREE_IDX_B] = get_freedom(_ripup_candidate_b, True)

    return observation_array


def algorithm_to_completion():
    """
    Execute the currently selected algorithm until completion.
    :return: void
    """
    global _routing_canvas
    global _done_circuit

    while not _done_circuit:
        a_star_multistep(1)


def can_blacklist(cell):
    global _routing_array

    for off_x in [-1, 0, 1]:
        for off_y in [-1, 0, 1]:
            in_bounds = cell.x + off_x >= 0 and cell.x + off_x < _array_width and cell.y + off_y >= 0 and cell.y + off_y < _array_height

            if (off_x == 0 and off_y == 0) or not in_bounds:
                continue

            neighbour_cell = _routing_array[cell.x + off_x][cell.y + off_y]

            skip = neighbour_cell.isSink or neighbour_cell.isSource

            if skip:
                return False

    return True


def get_least_congested_cell():
    """
    Get the cell with the least number of nets routed through it. If there is a tie, pick randomly.
    :return: Cell
    """
    global _routing_array

    least_congested_cells = [None]
    lowest_congestion = np.inf

    for column in _routing_array:
        for cell in column:
            if len(cell.netGroups) != cell.netCount+1:
                print("CONGESTION ERROR " + str(len(cell.netGroups)) + " " + str(cell.netCount))

            if 1 < cell.netCount < lowest_congestion:  # >1 otherwise uncongested/empty cells would be chosen
                least_congested_cells = [cell]
                lowest_congestion = cell.netCount
            elif cell.netCount == lowest_congestion:
                least_congested_cells.append(cell)

    least_congested_cell = random.choice(least_congested_cells)

    return least_congested_cell


def get_most_congested_cell():
    """
    Get the cell with the greatest number of nets routed through it
    :return: Cell
    """
    global _routing_array

    most_congested_cell = None
    greatest_congestion = 0

    for column in _routing_array:
        for cell in column:
            if cell.netCount > greatest_congestion and can_blacklist(cell):
                most_congested_cell = cell
                greatest_congestion = cell.netCount

    return most_congested_cell


def get_congested_cells():
    """
    Get the currently congested cells
    :return: list
    """
    global _routing_array

    congested_cells = []

    for column in _routing_array:
        for cell in column:
            if cell.isCongested:
                congested_cells.append(cell)

    return congested_cells


def get_congested_nets():
    """
    get the currently congested nets and the number of cells in each
    :return: dict
    """
    congestion = {}
    for net_id, net in _net_dict.items():
        if len(net.congestedCells) > 0:
            congestion[net_id] = len(net.congestedCells)

    return congestion


def rip_up_congested():
    """
    rip up nets until we reach 0 congestion
    :return: void
    """
    global _all_nets_routed

    c_nets = get_congested_nets()

    # if no congested nets, we are done
    _all_nets_routed = (len(c_nets) == 0)

    while len(c_nets) > 0:
        # get the most congested net and rip it up
        rip_net_id = max(c_nets, key=c_nets.get)
        rip_up_one(rip_net_id)

        # get new congestion levels
        c_nets = get_congested_nets()

    return


def a_star_multistep(n):
    """
    Perform multiple iterations of A*
    :param n: Number of iterations
    :return: void
    """
    global _routing_canvas
    global _active_net
    global _done_routing_attempt
    global _net_dict
    global _wavefront
    global _target_sink
    global _all_nets_routed
    global _failed_nets
    global _done_circuit
    global _final_route_initiated

    if _done_circuit:
        return

    # Set a net to route if none already set
    if _active_net is None:
        _active_net = _net_queue.get()

    # Check if the current routing attempt is complete
    if _done_routing_attempt:

        # rip up routes until we have 0 congestion
        rip_up_congested()

        # if no congestion, we can be done
        if _all_nets_routed:
            if len(_failed_nets) > 0:
                # At least one route was blocked
                print("Circuit could not be fully routed. Routed " + str(_num_segments_routed) + " segments.")
                _done_circuit = True
                return
            else:
                # Successful route
                print("Circuit routed successfully.")
                _done_circuit = True
                return
        else:
            # Try again with new congestion settings!
            _done_routing_attempt = False

            # Pick new net!
            _active_net = _net_queue.get()

    # Set wavefront if none is set
    if _wavefront is None:
        _target_sink, best_start_cell = find_best_routing_pair()
        # Start from source cell by default
        _wavefront = [_active_net.source.get_coords()]

        # Add routed cells for this net
        for cell in _active_net.wireCells:
            _wavefront.append((cell.x, cell.y))

    # Check if the wavefront still contains cells
    if len(_wavefront) == 0:
        # No more available cells for wavefront propagation in this net
        # This net cannot be routed
        # Move on to next net

        if _active_net.num not in _failed_nets:
            _failed_nets.append(_active_net.num)  # Add this net to the list of failed nets

        print("Failed to route net " + str(_active_net.num) + " with colour " + NET_COLOURS[_active_net.num])
        _all_nets_routed = False
        _wavefront = None  # New wavefront will be created for next net

        cleanup_candidates()

        if _net_queue.empty():
            # All nets are routed
            print("Routing attempt complete")
            print("Failed nets: " + str(_failed_nets))
            _done_routing_attempt = True
        else:
            # Move on to new net
            _active_net = _net_queue.get()
        return

    # Run A*
    for _ in range(n):
        a_star_step()


def adjust_congestion():
    """
    Correct congestion lists in each net.
    :return: void
    """
    for net_id, net in _net_dict.items():
        remove_cells = []

        # figure out which cells are no longer congested
        for c in net.congestedCells:
            if not c.isCongested:
                remove_cells.append(c)

        # remove decongested cells
        for r in remove_cells:
            net.congestedCells.remove(r)


def rip_up_one(net_id):
    """
    Rip-up a specific net.
    :param net_id: The net to be ripped up
    :return: void
    """
    global _routing_canvas
    global _num_segments_routed
    global _rl_target_cell
    global _active_net

    net = _net_dict[net_id]

    if net_id in _failed_nets:
        _failed_nets.remove(net_id)

    for cell in net.wireCells:
        cell.netGroups.remove(net_id)
        cell.isRouted = False
        cell.netCount -= 1
        if cell.netCount == 0:
            # if netCount is 0, cell is now free
            #if cell.isOwned:
            #    cell.isOwned = False
            cell.isWire = False
            fill = 'white'
        elif cell.netCount == 1:
            # if netCount is 1, cell is now decongested and owned by remaining net
            cell.isCongested = False
            #cell.isOwned = True
            fill = NET_COLOURS[cell.netGroups[-1]]
        else:
            # else, cell is still congested
            fill = 'light blue'
        cell.isCandidate = False
        cell.hasPropagated = False
        cell.distFromSource = 0
        cell.routingValue = 0
        cell.nextCell = []
        cell.prevCell = None

        _routing_canvas.itemconfigure(cell.id, fill=fill)

    adjust_congestion()

    # Reset source and sink cells
    source = net.source
    source.isRouted = False
    source.nextCell = []
    source.prevCell = None
    for sink in net.sinks:
        if sink.isRouted:
            _num_segments_routed += -1
            sink.isRouted = False
        sink.hasPropagated = False
        sink.distFromSource = 0
        sink.routingValue = 0
        sink.nextCell = []
        sink.prevCell = None

    # Reset net
    net.wireCells = []
    net.sinksRemaining = len(net.sinks)
    net.initRouteComplete = False
    net.congestedCells = []

    # Add this net to the target cell's blacklist
    if _rl_target_cell:
        _rl_target_cell.blacklist.append(net_id)
        if VERBOSE_ENV:
            print("Blacklisted " + str(net_id) + " from " + str(_rl_target_cell.x) + "," + str(_rl_target_cell.y))

    # If this rip-up resolved all congestion, then re-queue unrouted nets for future routing
    if len(get_congested_nets()) == 0:
        for net in _net_dict.values():
            if net.sinksRemaining > 0:
                _net_queue.put(net)

    # It's not a failed net anymore if we rip it up, eh?
    if net_id in _failed_nets:
        _failed_nets.remove(net_id)


def find_best_routing_pair():
    """
    Returns a sink to be routed and the best cell to start propagating from for that sink.
    "Best" starting cell is determined by the manhattan distance to the sink and the cell's "Freedom".
    Freedom refers to the number of adjacent cells that are unoccupied.
    :return: tuple of (target_sink, best_start_cell)
    """
    global _active_net
    global _circuit_is_hard

    if not isinstance(_active_net, Net):
        return

    # Start wavefront from the routed point that is closest to the next net
    shortest_dist = float("inf")
    best_start_cell = None
    best_sink = None
    # Separate sinks into routed and unrouted
    unrouted_sinks = [unrouted_sink for unrouted_sink in _active_net.sinks if not unrouted_sink.isRouted]
    routed_sinks = [routed_sink for routed_sink in _active_net.sinks if routed_sink.isRouted]
    if _circuit_is_hard:
        # Consider manhattan distance and cell freedom
        for unrouted_sink in unrouted_sinks:
            if not unrouted_sink.isRouted:
                # Check source cell and routed sinks first
                greatest_freedom = get_cell_freedom(_active_net.source)  # This is the greatest freedom by default
                dist = manhattan(unrouted_sink, _active_net.source)
                if dist < shortest_dist:
                    shortest_dist = dist
                    best_start_cell = _active_net.source
                    best_sink = unrouted_sink
                for routed_sink in routed_sinks:
                    routed_sink_freedom = get_cell_freedom(routed_sink)
                    if routed_sink_freedom >= greatest_freedom:
                        greatest_freedom = routed_sink_freedom
                        dist = manhattan(unrouted_sink, routed_sink)
                        if dist < shortest_dist:
                            shortest_dist = dist
                            best_start_cell = routed_sink
                            best_sink = unrouted_sink
                # Check wire cells
                for wire_cell in _active_net.wireCells:
                    wire_cell_freedom = get_cell_freedom(wire_cell)
                    if wire_cell_freedom >= greatest_freedom:
                        greatest_freedom = wire_cell_freedom
                        dist = manhattan(unrouted_sink, wire_cell)  # + wire_cell.congest_hist
                        if dist < shortest_dist:
                            shortest_dist = dist
                            best_start_cell = wire_cell
                            best_sink = unrouted_sink
    else:
        # Consider only manhattan distance
        for unrouted_sink in unrouted_sinks:
            if not unrouted_sink.isRouted:
                # Check source cell and routed sinks first
                dist = manhattan(unrouted_sink, _active_net.source)
                if dist < shortest_dist:
                    shortest_dist = dist
                    best_start_cell = _active_net.source
                    best_sink = unrouted_sink
                for routed_sink in routed_sinks:
                    dist = manhattan(unrouted_sink, routed_sink)
                    if dist < shortest_dist:
                        shortest_dist = dist
                        best_start_cell = routed_sink
                        best_sink = unrouted_sink
                # Check wire cells
                for wire_cell in _active_net.wireCells:
                    dist = manhattan(unrouted_sink, wire_cell)  # + wire_cell.congest_hist
                    if dist < shortest_dist:
                        shortest_dist = dist
                        best_start_cell = wire_cell
                        best_sink = unrouted_sink

    return best_sink, best_start_cell


def get_cell_freedom(cell: Cell) -> int:
    """
    Determine the freedom of a cell.
    Freedom is equal to the number of adjacent unoccupied cells (exluding diagonal adjacency)
    :param cell: Cell
    :return: int - freedom of the input cell
    """
    global _array_width
    global _array_height
    cell_x = cell.x
    cell_y = cell.y
    search_coords = [(cell_x, cell_y + 1), (cell_x, cell_y - 1),
                     (cell_x + 1, cell_y), (cell_x - 1, cell_y)]

    freedom = 0
    for (x, y) in search_coords:
        if 0 <= x < _array_width and 0 <= y < _array_height:
            neighbour = _routing_array[x][y]
            if not (neighbour.isObstruction or neighbour.isSource or neighbour.isSink or neighbour.isCandidate):
                freedom += 1
    return freedom


def a_star_step():
    """
    Perform a single iteration of A*
    :return: void
    """
    global _routing_canvas
    global _active_net
    global _wavefront
    global _target_sink
    global _done_routing_attempt
    global _num_segments_routed

    if not isinstance(_target_sink, Cell) or not isinstance(_active_net, Net) or not isinstance(_wavefront, list):
        return

    active_wavefront = _wavefront.copy()  # Avoid overwrite and loss of data
    _wavefront.clear()  # Will have a new wavefront after A* step

    # Perform a wavefront propagation
    sink_is_found = False
    sink_cell = None

    for cell_coords in active_wavefront:
        if not sink_is_found:
            # Get an active cell from the active wavefront
            cell_x = cell_coords[0]
            cell_y = cell_coords[1]
            active_cell = _routing_array[cell_x][cell_y]
            # Try to propagate one unit in each cardinal direction from the active cell
            search_coords = [(cell_x, cell_y + 1), (cell_x, cell_y - 1),
                             (cell_x + 1, cell_y), (cell_x - 1, cell_y)]
            for (cand_x, cand_y) in search_coords:
                if 0 <= cand_x < _array_width and 0 <= cand_y < _array_height:
                    cand_cell = _routing_array[cand_x][cand_y]  # Candidate cell for routing
                    # Check if a sink has been found
                    if cand_cell.isSink and _active_net.source.netGroups[0] in cand_cell.netGroups and \
                            cand_cell.isRouted is False:
                        # This is a sink for the source cell
                        sink_is_found = True
                        sink_cell = cand_cell
                        _active_net.sinksRemaining -= 1
                        sink_cell.routingValue = active_cell.distFromSource + 1
                        sink_cell.prevCell = active_cell
                        active_cell.nextCell.append(sink_cell)
                        break

                    # cell_is_viable = not cand_cell.isObstruction and not cand_cell.isCandidate and \
                    #                 not cand_cell.isSource and not cand_cell.isSink and not cand_cell.isOwned and \
                    #                 active_net.num not in cand_cell.blacklist

                    cell_is_viable = not cand_cell.isObstruction and not cand_cell.isCandidate and \
                                     not cand_cell.isSource and not cand_cell.isSink and \
                                     _active_net.num not in cand_cell.blacklist

                    # if cand_x == 6 and cand_y == 5:
                    #     print("blacklist:")
                    #     print(cand_cell.blacklist)
                    if _active_net.num in cand_cell.blacklist:
                        pass  # print("Net " + str(active_net.num) + " blocked from " + str(cand_x) + "," + str(cand_y))

                    if cell_is_viable:
                        # Note cell as a candidate for the routing path and add it to the wavefront
                        cand_cell.isCandidate = True
                        cand_cell.distFromSource = active_cell.distFromSource + 1
                        cand_cell.routingValue = cand_cell.distFromSource + manhattan(cand_cell, _target_sink)  # + \
                        cand_cell.prevCell = active_cell
                        active_cell.nextCell.append(cand_cell)
                        # Add routing value to rectangle
                        text_id = add_text(cand_cell)
                        _text_id_list.append(text_id)  # For later text deletion

    # Build wavefront for next step
    min_route_value = float("inf")
    for column in _routing_array:
        for cell in column:
            if cell.isCandidate:
                if not cell.hasPropagated:
                    _wavefront.append((cell.x, cell.y))
                    cell.hasPropagated = True
                    if cell.routingValue < min_route_value:
                        min_route_value = cell.routingValue

    # Remove cells from wavefront with large routing values
    wavefront_deletion_indices = []
    for idx, cell_coords in enumerate(_wavefront):
        cell_x = cell_coords[0]
        cell_y = cell_coords[1]
        cell = _routing_array[cell_x][cell_y]
        if cell.routingValue > min_route_value:
            wavefront_deletion_indices.append(idx)
            cell.hasPropagated = False
    wavefront_deletion_indices.reverse()  # Need to delete higher indices first to avoid index shifting between deletes
    for index in wavefront_deletion_indices:
        del _wavefront[index]

    if sink_is_found:
        # Connect sink to source (or other cell in net)
        net_is_routed = False
        net_colour = NET_COLOURS[sink_cell.netGroups[-1]]  # Needed to colour wires
        backtrace_cell = sink_cell
        while not net_is_routed:
            # Backtrace
            if ((_active_net.initRouteComplete and backtrace_cell in _active_net.wireCells) or backtrace_cell.isSource) \
                    and _active_net.num in backtrace_cell.netGroups:
                # Done
                net_is_routed = True
            elif backtrace_cell.isCandidate:
                backtrace_cell.isCandidate = False
                if backtrace_cell.isWire and _active_net.num not in backtrace_cell.netGroups:
                    backtrace_cell.isCongested = True
                    _active_net.congestedCells.append(backtrace_cell)
                    for net_id in backtrace_cell.netGroups:
                        if net_id != -1 and net_id != _active_net.num and \
                                backtrace_cell not in _net_dict[net_id].congestedCells:
                            _net_dict[net_id].congestedCells.append(backtrace_cell)
                    _routing_canvas.itemconfigure(backtrace_cell.id, fill="light blue")
                else:
                    backtrace_cell.isWire = True
                    _routing_canvas.itemconfigure(backtrace_cell.id, fill=net_colour)

                backtrace_cell.netCount += 1  # increase net count through this cell
                backtrace_cell.netGroups.append(_active_net.num)
                _active_net.wireCells.append(backtrace_cell)
            elif backtrace_cell.isSink:
                backtrace_cell.isRouted = True  # we only care if sinks are routed, they can't be used by other routes
                pass
            else:
                print("ERROR: Bad backtrace occurred!")
            backtrace_cell = backtrace_cell.prevCell

        # Increment counter for number of routed segments in current circuit routing attempt
        _num_segments_routed += 1

        # Clear non-wire cells
        cleanup_candidates()

        # Clear/increment active variables
        _wavefront = None
        if _active_net.sinksRemaining == 0:
            if _active_net.num in _failed_nets:
                _failed_nets.remove(_active_net.num)

            if _net_queue.empty():
                # All nets are routed
                _done_routing_attempt = True
                _active_net = None
            else:
                # Move on to next net
                _active_net = _net_queue.get()
        else:
            # Route the next sink
            _active_net.initRouteComplete = True


def add_text(cell: Cell) -> int:
    """
    Add text for the routing value of a cell to the canvas.
    To be used when adding a cell to the wavefront.
    :param cell: Cell
    :return: int - Tkinter ID for added text
    """
    global _routing_canvas

    # Edit rectangle in GUI to show it is in wavefront
    _routing_canvas.itemconfigure(cell.id, fill='black')
    # Place text inside the rect to show its routing value
    cell_rect_coords = _routing_canvas.coords(cell.id)
    text_x = (cell_rect_coords[0] + cell_rect_coords[2]) / 2
    text_y = (cell_rect_coords[1] + cell_rect_coords[3]) / 2
    if cell.routingValue > 99:
        text_size = 7
    else:
        text_size = 10
    return _routing_canvas.create_text(text_x, text_y, font=("arial", text_size),
                                       text=str(cell.routingValue), fill='white')


def cleanup_candidates():
    """
    Cleanup the canvas after wavefront propagation is complete.
    :return: void
    """
    global _routing_canvas
    global _routing_array
    global _text_id_list

    # Change routing candidate cells back to default colour
    for column in _routing_array:
        for cell in column:
            if cell.hasPropagated:
                cell.hasPropagated = False
            if cell.isCandidate:
                cell.isCandidate = False
                cell.routingValue = 0
                cell.hasPropagated = False
                if not cell.isWire:
                    _routing_canvas.itemconfigure(cell.id, fill='white')
                elif cell.isCongested:
                    _routing_canvas.itemconfigure(cell.id, fill='light blue')
                else:
                    net_idx = cell.netGroups[-1]
                    rectangle_colour = NET_COLOURS[net_idx]
                    _routing_canvas.itemconfigure(cell.id, fill=rectangle_colour)

    # Remove text from all cells (including cells that formed a route)
    for text_id in _text_id_list:
        _routing_canvas.delete(text_id)


def create_routing_array(routing_file):
    """
    Create the 2D routing grid/array
    :param routing_file: Path to the file with circuit info
    :return: list[list[Cell]] - Routing grid
    """
    global _net_queue

    grid_line = routing_file.readline()
    # Create the routing grid
    grid_width = int(grid_line.split(' ')[0])
    grid_height = int(grid_line.split(' ')[1])
    routing_grid = []
    # Create grid in column-major order
    for _ in range(grid_width):
        routing_grid.append([])
    # Populate grid with cells
    for cell_x, column in enumerate(routing_grid):
        for cell_y in range(grid_height):
            column.append(Cell(x=cell_x, y=cell_y))

    # Add cell obstructions
    num_obstructed_cells = int(routing_file.readline())
    for _ in range(num_obstructed_cells):
        obstruction_line = routing_file.readline()
        obstruction_x = int(obstruction_line.split(' ')[0])
        obstruction_y = int(obstruction_line.split(' ')[1])
        (routing_grid[obstruction_x][obstruction_y]).isObstruction = True

    # Add sources and sinks (Note that the routing array already has blank Cells)
    routing_file.readline()  # Discard (number of nets), data not needed
    for net_num, line in enumerate(routing_file):
        net_tokens = line.split(' ')
        num_pins = int(net_tokens[0])
        # Add source
        source_x = int(net_tokens[1])
        source_y = int(net_tokens[2])
        source_cell = routing_grid[source_x][source_y]
        source_cell.isSource = True
        source_cell.netGroups = [net_num]
        new_net = Net(source=source_cell, num=net_num)
        # Add sinks
        for idx in range(3, 3 + 2 * (num_pins - 1)):
            if idx % 2 == 1:
                # Create sink cell
                cell_x = int(net_tokens[idx])
                cell_y = int(net_tokens[idx + 1])
                sink_cell = routing_grid[cell_x][cell_y]
                sink_cell.isSink = True
                sink_cell.netGroups = [net_num]
                # Add sink cell to a net
                new_net.sinks.append(sink_cell)
                new_net.sinksRemaining += 1

        # Add the new net to the net dictionary
        _net_dict[new_net.num] = new_net
        # Place nets in FIFO (determines routing order)
        _net_queue.put(new_net)

    return routing_grid


def manhattan(cell1: Cell, cell2: Cell) -> int:
    """
    Return the Manhattan distance between two Cells
    :param cell1: Cell
    :param cell2: Cell
    :return: int
    """
    return abs(cell1.x - cell2.x) + abs(cell1.y - cell2.y)


if __name__ == "__main__":
    main()
