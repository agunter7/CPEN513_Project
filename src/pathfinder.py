"""
Solution to UBC CPEN 513 Assignment 1.
Implements Dijkstra's algorithm and A* for multi-pin nets.
Utilizes a simple rip-up and re-route scheme.
Uses Tkinter for GUI.
"""

import os
from tkinter import *
from enum import Enum
from queue import PriorityQueue
import random


# Constants
FILE_PATH = "../benchmarks_no_obst/kuma.infile"  # Path to the file with info about the circuit to route
NET_COLOURS = ["red", "yellow", "grey", "orange", "purple", "pink", "green", "medium purple", "white"]
MAX_NET_PRIORITY = 2
MIN_NET_PRIORITY = 0

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
        self.netGroups = [net_group]  # Int indicating which net the Cell belongs to
        self.id = -1  # Tkinter ID for a corresponding rectangle
        self.isRouted = False   # Has this cell been routed in the current attempt?
        self.isWire = False     # Is this cell part of a trace?
        self.isCandidate = False    # Is the cell a candidate for the current route?
        self.isCongested = False    # Is the cell congested?
        self.netCount = 0           # the number of nets routed through this cell
        self.hasPropagated = False  # Has this cell already been used in a wavefront propagation?
        self.dist_from_source = 0  # Routing distance from corresponding source Cell
        self.routingValue = 0  # Value used in wavefront for Dijkstra/A*
        self.next_cell = []  # Can have multiple "next" cells, because wavefront propagates in 4 directions
        self.prev_cell = None  # Reference to previous cell in route, for backtrace
        self.congest = 1        # congestion in previous round (initialize to 1)
        self.congest_hist = 0   # product of previous congestion numbers
        self.isOwned = False

    def get_coords(self): return self.x, self.y


class Net:
    """
    A collection of cells
    """
    def __init__(self, source: Cell = None, sinks: Cell = None, num=-1):
        if sinks is None:
            sinks = []
        self.priority = MAX_NET_PRIORITY  # priority for net ordering
        self.source = source  # Source Cell
        self.sinks = sinks  # List of sink Cells
        self.wireCells = []  # List of wire Cells
        self.congestedCells = [] # number of congested cells in trace
        self.num = num  # The net number for this net (effectively the net's identifier)
        self.sinksRemaining = len(self.sinks)  # Number of sinks left to be routed in this net
        self.initRouteComplete = False  # Has the net's source been routed to at least one sink?
        # self.failed_attempts = 0  # number of times we've tried to route this net unsuccessfully

        if self.num == -1:
            print("ERROR: assign a net number to the newly-created net!")


def main():
    """
    Top-level main function
    :return: void
    """
    global routing_array
    global array_width
    global array_height
    global FILE_PATH

    # Read input file
    script_path = os.path.dirname(__file__)
    true_path = os.path.join(script_path, FILE_PATH)
    routing_file = open(true_path, "r")

    # Setup the routing grid/array
    routing_array = create_routing_array(routing_file)
    array_width = len(routing_array)
    array_height = len(routing_array[0])

    # Create routing canvas in Tkinter
    root = Tk()
    root.columnconfigure(0, weight=1)
    root.rowconfigure(0, weight=1)
    cell_length = 15
    routing_canvas = Canvas(root, bg='white', width=array_width*cell_length, height=array_height*cell_length)
    routing_canvas.grid(column=0, row=0, sticky=(N, W, E, S))
    for x in range(array_width):
        for y in range(array_height):
            # Add a rectangle to the canvas
            top_left_x = cell_length * x
            top_left_y = cell_length * y
            bottom_right_x = top_left_x + cell_length
            bottom_right_y = top_left_y + cell_length
            if (routing_array[x][y]).isObstruction:
                rectangle_colour = "blue"
            elif (routing_array[x][y]).isCongested:
                rectangle_colour = "light blue"
            else:
                net_idx = (routing_array[x][y]).netGroups[-1]
                rectangle_colour = NET_COLOURS[net_idx]
            rectangle_coords = (top_left_x, top_left_y, bottom_right_x, bottom_right_y)
            (routing_array[x][y]).id = routing_canvas.create_rectangle(rectangle_coords, fill=rectangle_colour)

    # Event bindings and Tkinter start
    routing_canvas.focus_set()
    routing_canvas.bind("<Key>", lambda event: key_handler(routing_canvas, event))
    root.mainloop()


def key_handler(routing_canvas, event):
    """
    Accepts a key event and makes an appropriate decision.
    :param routing_canvas: Tkinter canvas
    :param event: Key event
    :return: void
    """

    e_char = event.char

    if e_char == '0':
        # print("running to completion")
        algorithm_to_completion(routing_canvas)
    elif str.isdigit(e_char):
        algorithm_multistep(routing_canvas, int(e_char))
    else:
        pass


def algorithm_to_completion(routing_canvas):
    """
    Execute the currently selected algorithm until completion.
    :param routing_canvas: Tkinter canvas
    :return: void
    """
    global done_circuit

    while not done_circuit:
        algorithm_multistep(routing_canvas, 1)


def get_congested_nets(routing_canvas):
    '''
    get the currently congested nets and the number of cells in each
    :param routing_canvas: Tkinter canvas
    :return: void
    '''
    congestion = {}
    for net_id, net in net_dict.items():
        if len(net.congestedCells) > 0:
            congestion[net_id] = len(net.congestedCells)

    return congestion

def rip_up_congested(routing_canvas):
    '''
    rip up nets until we reach 0 congestion
    :param routing_canvas: Tkinter canvas
    :return: void
    '''
    global all_nets_routed

    c_nets = get_congested_nets(routing_canvas)

    num_nets = len(c_nets)

    # if no congested nets, we are done
    all_nets_routed = (num_nets == 0)

    while num_nets > 0:
        # get the most congested net and rip it up
        # in case of tie, pick randomly
        # THIS IS WHERE THE RL AGENT WILL GO
        max_val = max(c_nets.values())
        max_nets = [k for k, v in c_nets.items() if v == max_val]

        # rip up a random max-valued net
        rip_net = random.choice(max_nets)

        rip_up_one(routing_canvas, rip_net)

        # get new congestion levels
        c_nets = get_congested_nets(routing_canvas)

        num_nets = len(c_nets)

    return


def algorithm_multistep(routing_canvas, n):
    """
    Generically perform multiple iterations of the currently selected algorithm
    :param routing_canvas: Tkinter canvas
    :param n: Number of iterations
    :return: void
    """
    global active_net
    global done_routing_attempt
    global net_dict
    global wavefront
    global target_sink
    global current_net_order_idx
    global all_nets_routed
    global failed_nets
    global done_circuit
    global final_route_initiated

    if done_circuit:
        return

    if active_net is None and wavefront is None:
        # This is the start of a routing attempt
        # Record the current net priorities
        del starting_priority_set[:]
        for net in net_dict.values():
            starting_priority_set.append((net.num, net.priority))

    # Set a net to route if none already set
    if active_net is None:
        if len(net_order) == 0:
            # Determine the order to route nets in from priority queue
            while not net_pq.empty():
                priority, next_net_num = net_pq.get()
                net_order.append(next_net_num)
        active_net = net_dict[net_order[current_net_order_idx]]

    # Check if the current routing attempt is complete
    if done_routing_attempt:
        # increase history values for congested cells
        set_history(routing_canvas)

        # rip up routes until we have 0 congestion
        rip_up_congested(routing_canvas)

        # if no congestion, we can be done
        if all_nets_routed:
            if len(failed_nets) > 0:
                # At least one route was blocked
                print("Circuit could not be fully routed. Routed " + str(num_segments_routed) + " segments.")
                done_circuit = True
                return
            else:
                # Successful route
                print("Circuit routed successfully.")
                done_circuit = True
                return
        else:
            # Try again with new congestion settings!
            done_routing_attempt = False

            # Pick new net!
            next_net_order_idx = -1
            # Get first net in order that is unrouted (or was ripped up)
            for next_net in range(len(net_order)):
                net = net_dict[next_net]
                if net.sinksRemaining > 0:
                    next_net_order_idx = next_net
                    break

            # If next index is valid
            if next_net_order_idx >= 0 and next_net_order_idx != current_net_order_idx:
                current_net_order_idx = next_net_order_idx
                active_net = net_dict[net_order[current_net_order_idx]]
                

    # Set wavefront if none is set
    if wavefront is None:
        target_sink, best_start_cell = find_best_routing_pair()
        # Start from source cell by default
        wavefront = [active_net.source.get_coords()]

        # Add routed cells for this net
        for cell in active_net.wireCells:
            wavefront.append((cell.x, cell.y))

    # Check if the wavefront still contains cells
    if len(wavefront) == 0:
        # No more available cells for wavefront propagation in this net
        # This net cannot be routed
        # Move on to next net

        if active_net.num not in failed_nets:
            failed_nets.append(active_net.num)  # Add this net to the list of failed nets

        print("Failed to route net " + str(active_net.num) + " with colour " + NET_COLOURS[active_net.num])
        all_nets_routed = False
        wavefront = None  # New wavefront will be created for next net

        cleanup_candidates(routing_canvas)

        next_net_order_idx = -1
        # Get first net in order that is unrouted (or was ripped up)
        for next_net in range(len(net_order)):
            net = net_dict[next_net]
            if net.sinksRemaining > 0:
                next_net_order_idx = next_net
                break

        if next_net_order_idx >= 0 and next_net_order_idx != current_net_order_idx:
            current_net_order_idx = next_net_order_idx
            active_net = net_dict[net_order[current_net_order_idx]]
        else:
            # All nets are routed
            print("Routing attempt complete")
            print("Failed nets: " + str(failed_nets))
            done_routing_attempt = True
        return

    # Run astar
    a_star_multistep(routing_canvas, n)


def adjust_congestion(routing_canvas):
    """
    Correct congestion lists in each net.
    :param routing_canvas: Tkinter canvas
    :return: void
    """
    for net_id, net in net_dict.items():
        remove_cells = []

        # figure out which cells are no longer congested
        for c in net.congestedCells:
            if not c.isCongested:
                remove_cells.append(c)

        # remove uncongested cells
        for r in remove_cells:
            net.congestedCells.remove(r)


def set_history(routing_canvas):
    '''
    add current congestion to history (for weighting)
    :param routing_canvas: Tkinter canvas
    :return: void
    '''

    for row in routing_array:
        for cell in row:
            if cell.isCongested or cell.congest > 1:
                # increase history of congestion
                cell.congest_hist += cell.congest
                # else:
                    # cell.congest_hist = cell.congest

                # set congestion for this cell this round
                cell.congest = max(1, cell.netCount)
            else:
                cell.congest = 1

            if cell.congest == 0:
                print("ERROR: Cell penalty should never be 0")


def rip_up_one(routing_canvas, net_id):
    """
    Rip-up the current circuit so that it can be rerouted.
    :param routing_canvas: Tkinter canvas
    :param net_id: The net to be ripped up
    :return: void
    """
    global num_segments_routed

    net = net_dict[net_id]

    if net_id in failed_nets:
        failed_nets.remove(net_id)

    for cell in net.wireCells:
        fill = 'white'
        cell.netGroups.remove(net_id)
        cell.isRouted = False
        cell.netCount -= 1
        if cell.netCount == 0:
            # if netCount is 0, cell is now free
            if cell.isOwned:
                cell.isOwned = False
            cell.isWire = False
            fill = 'white'
        elif cell.netCount == 1:
            # if netCount is 1, cell is now uncongested and owned by remaining net
            cell.isCongested = False
            cell.isOwned = True
            fill = NET_COLOURS[cell.netGroups[-1]]
        else:
            # else, cell is still congested
            fill = 'light blue'
        cell.isCandidate = False
        cell.hasPropagated = False
        cell.dist_from_source = 0
        cell.routingValue = 0
        cell.next_cell = []
        cell.prev_cell = None

        routing_canvas.itemconfigure(cell.id, fill=fill)

    adjust_congestion(routing_canvas)

    # Reset source and sink cells
    source = net.source
    source.isRouted = False
    source.next_cell = []
    source.prev_cell = None
    for sink in net.sinks:
        if sink.isRouted:
            num_segments_routed += -1
            sink.isRouted = False
        sink.hasPropagated = False
        sink.dist_from_source = 0
        sink.routingValue = 0
        sink.next_cell = []
        sink.prev_cell = None

    # Reset net
    net.wireCells = []
    net.sinksRemaining = len(net.sinks)
    net.initRouteComplete = False
    net.congestedCells = []


def find_best_routing_pair():
    """
    Returns a sink to be routed and the best cell to start propagating from for that sink.
    "Best" starting cell is determined by the manhattan distance to the sink and the cell's "Freedom".
    Freedom refers to the number of adjacent cells that are unoccupied.
    :return: tuple of (target_sink, best_start_cell)
    """
    global active_net

    if not isinstance(active_net, Net):
        return

    # Start wavefront from the routed point that is closest to the next net
    shortest_dist = float("inf")
    best_start_cell = None
    best_sink = None
    # Separate sinks into routed and unrouted
    unrouted_sinks = [unrouted_sink for unrouted_sink in active_net.sinks if not unrouted_sink.isRouted]
    routed_sinks = [routed_sink for routed_sink in active_net.sinks if routed_sink.isRouted]
    
    # Consider only manhattan distance
    for unrouted_sink in unrouted_sinks:
        if not unrouted_sink.isRouted:
            # Check source cell and routed sinks first
            dist = manhattan(unrouted_sink, active_net.source)
            if dist < shortest_dist:
                shortest_dist = dist
                best_start_cell = active_net.source
                best_sink = unrouted_sink
            for routed_sink in routed_sinks:
                dist = manhattan(unrouted_sink, routed_sink)
                if dist < shortest_dist:
                    shortest_dist = dist
                    best_start_cell = routed_sink
                    best_sink = unrouted_sink
            # Check wire cells
            for wire_cell in active_net.wireCells:
                dist = (manhattan(unrouted_sink, wire_cell) + wire_cell.congest_hist)*wire_cell.congest
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
    global array_width
    global array_height
    cell_x = cell.x
    cell_y = cell.y
    search_coords = [(cell_x, cell_y + 1), (cell_x, cell_y - 1),
                     (cell_x + 1, cell_y), (cell_x - 1, cell_y)]

    freedom = 0
    for (x, y) in search_coords:
        if 0 <= x < array_width and 0 <= y < array_height:
            neighbour = routing_array[x][y]
            if not (neighbour.isObstruction or neighbour.isSource or neighbour.isSink or neighbour.isCandidate):
                    # or neighbour.isWire or neighbour.isRouted):
                freedom += 1
    return freedom


def a_star_multistep(routing_canvas, n):
    """
    Perform n iterations of A*
    :param routing_canvas: Tkinter canvas
    :param n: number of iterations
    :return: void
    """
    for _ in range(n):
        a_star_step(routing_canvas)


def a_star_step(routing_canvas):
    """
    Perform a single iteration of A*
    :param routing_canvas: Tkinter canvas
    :return: void
    """
    global active_net
    global wavefront
    global target_sink
    global done_routing_attempt
    global current_net_order_idx
    global num_segments_routed

    if not isinstance(target_sink, Cell) or not isinstance(active_net, Net) or not isinstance(wavefront, list):
        return

    active_wavefront = wavefront.copy()  # Avoid overwrite and loss of data
    wavefront.clear()  # Will have a new wavefront after A* step

    # Perform a wavefront propagation
    sink_is_found = False
    sink_cell = None

    for cell_coords in active_wavefront:
        if not sink_is_found:
            # Get an active cell from the active wavefront
            cell_x = cell_coords[0]
            cell_y = cell_coords[1]
            active_cell = routing_array[cell_x][cell_y]
            # Try to propagate one unit in each cardinal direction from the active cell
            search_coords = [(cell_x, cell_y + 1), (cell_x, cell_y - 1),
                             (cell_x + 1, cell_y), (cell_x - 1, cell_y)]
            for (cand_x, cand_y) in search_coords:
                if 0 <= cand_x < array_width and 0 <= cand_y < array_height:
                    cand_cell = routing_array[cand_x][cand_y]  # Candidate cell for routing
                    # Check if a sink has been found
                    if cand_cell.isSink and active_net.source.netGroups[0] in cand_cell.netGroups and \
                           cand_cell.isRouted is False:
                        # This is a sink for the source cell
                        sink_is_found = True
                        sink_cell = cand_cell
                        active_net.sinksRemaining -= 1
                        sink_cell.routingValue = active_cell.dist_from_source+1
                        sink_cell.prev_cell = active_cell
                        active_cell.next_cell.append(sink_cell)
                        break

                    cell_is_viable = not cand_cell.isObstruction and not cand_cell.isCandidate and not cand_cell.isSource and \
                        not cand_cell.isSink #and not cand_cell.isOwned # and not cand_cell.isWire 

                    if cell_is_viable:
                        # Note cell as a candidate for the routing path and add it to the wavefront
                        cand_cell.isCandidate = True
                        cand_cell.dist_from_source = active_cell.dist_from_source+1
                        # cand_cell.routingValue = cand_cell.dist_from_source + manhattan(cand_cell, target_sink) + cand_cell.congest + cand_cell.congest_hist
                        # PathFinder does c = (b + h)*p where c is cost, b is base cost, h is history, and p is current usage
                        cand_cell.routingValue = (cand_cell.dist_from_source + manhattan(cand_cell, target_sink) + cand_cell.congest_hist)*cand_cell.congest
                       # if cand_cell.isOwned:
                        #     cand_cell.routingValue += 50
                        cand_cell.prev_cell = active_cell
                        active_cell.next_cell.append(cand_cell)
                        # Add routing value to rectangle
                        text_id = add_text(routing_canvas, cand_cell)
                        text_id_list.append(text_id)  # For later text deletion

    # Build wavefront for next step
    min_route_value = float("inf")
    for column in routing_array:
        for cell in column:
            if cell.isCandidate:
                if not cell.hasPropagated:
                    wavefront.append((cell.x, cell.y))
                    cell.hasPropagated = True
                    if cell.routingValue < min_route_value:
                        min_route_value = cell.routingValue

    # Remove cells from wavefront with large routing values
    wavefront_deletion_indices = []
    for idx, cell_coords in enumerate(wavefront):
        cell_x = cell_coords[0]
        cell_y = cell_coords[1]
        cell = routing_array[cell_x][cell_y]
        if cell.routingValue > min_route_value:
            wavefront_deletion_indices.append(idx)
            cell.hasPropagated = False
    wavefront_deletion_indices.reverse()  # Need to delete higher indices first to avoid index shifting between deletes
    for index in wavefront_deletion_indices:
        del wavefront[index]

    if sink_is_found:
        # Connect sink to source (or other cell in net)
        print("Connecting sink at " + str(sink_cell.x) + ", " + str(sink_cell.y))
        net_is_routed = False
        net_colour = NET_COLOURS[sink_cell.netGroups[-1]]  # Needed to colour wires
        backtrace_cell = sink_cell
        while not net_is_routed:
            # Backtrace
            if ((active_net.initRouteComplete and backtrace_cell in active_net.wireCells) or backtrace_cell.isSource) \
                    and active_net.num in backtrace_cell.netGroups:
                # Done
                net_is_routed = True
            elif backtrace_cell.isCandidate:
                backtrace_cell.isCandidate = False
                if backtrace_cell.isWire and active_net.num not in backtrace_cell.netGroups:
                    backtrace_cell.isCongested = True
                    active_net.congestedCells.append(backtrace_cell)
                    for net_id in backtrace_cell.netGroups:
                        if net_id != -1 and net_id != active_net.num and backtrace_cell not in net_dict[net_id].congestedCells:
                            net_dict[net_id].congestedCells.append(backtrace_cell)
                    routing_canvas.itemconfigure(backtrace_cell.id, fill="light blue")
                else:
                    backtrace_cell.isWire = True
                    routing_canvas.itemconfigure(backtrace_cell.id, fill=net_colour)
                
                backtrace_cell.netCount += 1 # increase net count through this cell
                backtrace_cell.netGroups.append(active_net.num)
                active_net.wireCells.append(backtrace_cell)
            elif backtrace_cell.isSink:
                backtrace_cell.isRouted = True # we only care if sinks are routed, as they can't be used by other routes
                pass
            else:
                print("ERROR: Bad backtrace occurred!")
            backtrace_cell = backtrace_cell.prev_cell

        # Increment counter for number of routed segments in current circuit routing attempt
        num_segments_routed += 1

        # Clear non-wire cells
        cleanup_candidates(routing_canvas)

        # Clear/increment active variables
        wavefront = None
        if active_net.sinksRemaining < 1:
            if net_order[current_net_order_idx] in failed_nets:
                failed_nets.remove(net_order[current_net_order_idx])

            next_net_order_idx = -1
            # Get first net in order that is unrouted (or was ripped up)
            for next_net in range(len(net_order)):
                net = net_dict[next_net]
                if net.sinksRemaining > 0:
                    next_net_order_idx = next_net
                    break

            if next_net_order_idx >= 0 and current_net_order_idx != next_net_order_idx:
                current_net_order_idx = next_net_order_idx
                active_net = net_dict[net_order[current_net_order_idx]]
            else:
                # All nets are routed
                print("Routing attempt complete")
                print("Failed nets: " + str(failed_nets))
                done_routing_attempt = True
        else:
            # Route the next sink
            active_net.initRouteComplete = True


def add_text(routing_canvas: Canvas, cell: Cell) -> int:
    """
    Add text for the routing value of a cell to the canvas.
    To be used when adding a cell to the wavefront.
    :param routing_canvas: Tkinter canvas
    :param cell: Cell
    :return: int - Tkinter ID for added text
    """
    # Edit rectangle in GUI to show it is in wavefront
    routing_canvas.itemconfigure(cell.id, fill='black')
    # Place text inside the rect to show its routing value
    cell_rect_coords = routing_canvas.coords(cell.id)
    text_x = (cell_rect_coords[0] + cell_rect_coords[2]) / 2
    text_y = (cell_rect_coords[1] + cell_rect_coords[3]) / 2
    if cell.routingValue > 99:
        text_size = 7
    else:
        text_size = 10
    return routing_canvas.create_text(text_x, text_y, font=("arial", text_size),
                                      text=str(cell.routingValue), fill='white')


def cleanup_candidates(routing_canvas):
    """
    Cleanup the canvas after wavefront propagation is complete.
    :param routing_canvas: Tkinter canvas
    :return: void
    """
    global routing_array
    global text_id_list

    # Change routing candidate cells back to default colour
    for column in routing_array:
        for cell in column:
            if cell.hasPropagated:
                cell.hasPropagated = False
            if cell.isCandidate:
                cell.isCandidate = False
                cell.routingValue = 0
                cell.hasPropagated = False
                if not cell.isWire:
                    routing_canvas.itemconfigure(cell.id, fill='white')
                elif cell.isCongested:
                    routing_canvas.itemconfigure(cell.id, fill='light blue')
                else:
                    net_idx = cell.netGroups[-1]
                    rectangle_colour = NET_COLOURS[net_idx]
                    routing_canvas.itemconfigure(cell.id, fill=rectangle_colour)

    # Remove text from all cells (including cells that formed a route)
    for text_id in text_id_list:
        routing_canvas.delete(text_id)


def create_routing_array(routing_file):
    """
    Create the 2D routing grid/array
    :param routing_file: Path to the file with circuit info
    :return: list[list[Cell]] - Routing grid
    """
    global net_order
    global net_pq

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
    num_nets = routing_file.readline()  # Discard, data not needed
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
        for idx in range(3, 3+2*(num_pins-1)):
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

        # for i in range(len(new_net.sinks)):
        #     mindex = i
        #     for j in range(i, len(new_net.sinks)):
        #         if manhattan(new_net.sinks[j], new_net.source) > manhattan(new_net.sinks[mindex], new_net.source):
        #             mindex = j
        #     if i != mindex:
        #         new_net.sinks[i], new_net.sinks[mindex] = new_net.sinks[mindex], new_net.sinks[i]


        # Add the new net to the net dictionary
        net_dict[new_net.num] = new_net
        # Place net numbers in priority queue (priority determines routing order)
        # Use net nums instead of nets themselves because net nums can be tie-broken when priority is equal
        net_pq.put((new_net.priority, new_net.num))

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
