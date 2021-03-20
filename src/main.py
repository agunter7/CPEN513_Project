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


class Algorithm(Enum):
    NONE = -1
    DIJKSTRA = 0
    A_STAR = 1


# Constants
FILE_PATH = "../benchmarks/test.infile"  # Path to the file with info about the circuit to route
NET_COLOURS = ["red", "yellow", "grey", "orange", "purple", "pink", "green", "medium purple", "white"]
MAX_NET_PRIORITY = 2
MIN_NET_PRIORITY = 0

# Global variables
active_algorithm = Algorithm.NONE  # Dijkstra or A*
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
net_priorities_changed = False  # Did the priority of any net change since the last rip-up?
final_route_initiated = False  # Is the current routing attempt definitely the last one?
circuit_is_hard = False  # Did the circuit fail to route on the first attempt?


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
        self.netGroup = net_group  # Int indicating which net the Cell belongs to
        self.id = -1  # Tkinter ID for a corresponding rectangle
        self.isRouted = False  # Has this cell been routed in the current attempt?
        self.isWire = False
        self.isCandidate = False  # Is the cell a candidate for the current route?
        self.hasPropagated = False  # Has this cell already been used in a wavefront propagation?
        self.dist_from_source = 0  # Routing distance from corresponding source Cell
        self.routingValue = 0  # Value used in wavefront for Dijkstra/A*
        self.next_cell = []  # Can have multiple "next" cells, because wavefront propagates in 4 directions
        self.prev_cell = None  # Reference to previous cell in route, for backtrace

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
            else:
                net_idx = (routing_array[x][y]).netGroup
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
    global active_algorithm

    e_char = event.char
    if e_char == 'a':
        if active_algorithm == Algorithm.NONE:
            active_algorithm = Algorithm.A_STAR
            print("Selected A*")
    elif e_char == 'd':
        if active_algorithm == Algorithm.NONE:
            active_algorithm = Algorithm.DIJKSTRA
            print("Selected Dijkstra")
    elif e_char == '0':
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
    global active_algorithm

    if active_algorithm is not Algorithm.A_STAR and active_algorithm is not Algorithm.DIJKSTRA:
        return

    while not done_circuit:
        algorithm_multistep(routing_canvas, 1)


def algorithm_multistep(routing_canvas, n):
    """
    Generically perform multiple iterations of the currently selected algorithm
    :param routing_canvas: Tkinter canvas
    :param n: Number of iterations
    :return: void
    """
    global active_algorithm
    global active_net
    global done_routing_attempt
    global net_dict
    global wavefront
    global target_sink
    global current_net_order_idx
    global all_nets_routed
    global failed_nets
    global net_priorities_changed
    global done_circuit
    global final_route_initiated

    if done_circuit or (active_algorithm is not Algorithm.A_STAR and active_algorithm is not Algorithm.DIJKSTRA):
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
        if final_route_initiated:
            # No more routing attempts are to be performed
            print("Circuit could not be fully routed. Routed " + str(num_segments_routed) + " segments.")
            done_circuit = True
            return
        if all_nets_routed:
            # Successful route
            print("Circuit routed successfully.")
            done_circuit = True
            return
        else:
            # Only rip-up and reroute if it is possible that a new net order could allow for a fully routed circuit
            # Determine this by whether or not net priorities changed on this routing attempt
            # Unchanged priorities imply that no new net order permutations will be attempted
            if net_priorities_changed:
                # Rip-up and reroute
                print("Rip-up")
                rip_up(routing_canvas)
                net_priorities_changed = False
            else:
                # Perform a final route with the best-performing net priorities
                print("Final rip-up")
                # Overwrite current net priorities with the best known values
                for (net_num, best_priority) in best_priority_set:
                    net = net_dict[net_num]
                    net.priority = best_priority
                rip_up(routing_canvas)
                final_route_initiated = True

            return

    # Set wavefront if none is set
    if wavefront is None:
        target_sink, best_start_cell = find_best_routing_pair()
        if active_net.initRouteComplete:
            # Start wavefront propagation from "best" cell in net
            wavefront = [best_start_cell.get_coords()]
        else:
            # Start from source cell by default
            wavefront = [active_net.source.get_coords()]

    # Check if the wavefront still contains cells
    if len(wavefront) == 0:
        # No more available cells for wavefront propagation in this net
        # This net cannot be routed
        # Move on to next net
        print("Failed to route net " + str(active_net.num) + " with colour " + NET_COLOURS[active_net.num])
        all_nets_routed = False
        if active_net.priority > MIN_NET_PRIORITY:
            # This net should be given higher priority in the next routing attempt
            active_net.priority -= 1
            net_priorities_changed = True
        failed_nets.append(active_net.num)  # Add this net to the list of failed nets
        cleanup_candidates(routing_canvas)
        wavefront = None  # New wavefront will be created for next net
        if current_net_order_idx + 1 < len(net_order):
            # Proceed to the next net
            current_net_order_idx += 1
            active_net = net_dict[net_order[current_net_order_idx]]
        else:
            # All nets are routed
            print("Routing attempt complete")
            print("Failed nets: " + str(failed_nets))
            done_routing_attempt = True
        return

    # Pick algorithm to execute
    if active_algorithm == Algorithm.DIJKSTRA:
        dijkstra_multistep(routing_canvas, n)
    elif active_algorithm == Algorithm.A_STAR:
        a_star_multistep(routing_canvas, n)
    else:
        return


def rip_up(routing_canvas):
    """
    Rip-up the current circuit so that it can be rerouted.
    :param routing_canvas: Tkinter canvas
    :return: void
    """
    global wavefront
    global active_net
    global text_id_list
    global done_routing_attempt
    global target_sink
    global net_order
    global current_net_order_idx
    global all_nets_routed
    global failed_nets
    global routing_array
    global num_segments_routed
    global best_num_segments_routed
    global best_priority_set
    global circuit_is_hard

    # Circuit failed to route and is therefore deemed "hard"
    circuit_is_hard = True

    # Check if the circuit being ripped up is the best routing attempt thus far
    if num_segments_routed > best_num_segments_routed:
        best_num_segments_routed = num_segments_routed
        # Save the net priorities that lead to this route
        del best_priority_set[:]
        for (net_num, priority) in starting_priority_set:
            best_priority_set.append((net_num, priority))

    # Restore the necessary global state variables to default values
    num_segments_routed = 0
    wavefront = None
    active_net = None
    done_routing_attempt = False
    target_sink = None
    net_order = []
    current_net_order_idx = 0
    all_nets_routed = True  # Assumed true and proven false
    failed_nets = []
    # Remove all wire cells
    for text_id in text_id_list:
        routing_canvas.delete(text_id)
    for column in routing_array:
        for cell in column:
            if cell.isWire:
                routing_canvas.itemconfigure(cell.id, fill='white')
                cell.netGroup = -1
                cell.isRouted = False
                cell.isWire = False
                cell.isCandidate = False
                cell.hasPropagated = False
                cell.dist_from_source = 0
                cell.routingValue = 0
                cell.next_cell = []
                cell.prev_cell = None
    # Reset all source and sink cells
    for net in net_dict.values():
        source = net.source
        source.isRouted = False
        source.next_cell = []
        source.prev_cell = None
        for sink in net.sinks:
            sink.isRouted = False
            sink.hasPropagated = False
            sink.dist_from_source = 0
            sink.routingValue = 0
            sink.next_cell = []
            sink.prev_cell = None
    # Reset all nets
    for net in net_dict.values():
        net.wireCells = []
        net.sinksRemaining = len(net.sinks)
        net.initRouteComplete = False

    # Setup net priority queue for next routing iteration
    for net in net_dict.values():
        net_pq.put((net.priority, net.num))


def find_best_routing_pair():
    """
    Returns a sink to be routed and the best cell to start propagating from for that sink.
    "Best" starting cell is determined by the manhattan distance to the sink and the cell's "Freedom".
    Freedom refers to the number of adjacent cells that are unoccupied.
    :return: tuple of (target_sink, best_start_cell)
    """
    global active_net
    global circuit_is_hard

    if not isinstance(active_net, Net):
        return

    # Start wavefront from the routed point that is closest to the next net
    shortest_dist = float("inf")
    best_start_cell = None
    best_sink = None
    # Separate sinks into routed and unrouted
    unrouted_sinks = [unrouted_sink for unrouted_sink in active_net.sinks if not unrouted_sink.isRouted]
    routed_sinks = [routed_sink for routed_sink in active_net.sinks if routed_sink.isRouted]
    if circuit_is_hard:
        # Consider manhattan distance and cell freedom
        for unrouted_sink in unrouted_sinks:
            if not unrouted_sink.isRouted:
                # Check source cell and routed sinks first
                greatest_freedom = get_cell_freedom(active_net.source)  # This is the greatest freedom by default
                dist = manhattan(unrouted_sink, active_net.source)
                if dist < shortest_dist:
                    shortest_dist = dist
                    best_start_cell = active_net.source
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
                for wire_cell in active_net.wireCells:
                    wire_cell_freedom = get_cell_freedom(wire_cell)
                    if wire_cell_freedom >= greatest_freedom:
                        greatest_freedom = wire_cell_freedom
                        dist = manhattan(unrouted_sink, wire_cell)
                        if dist < shortest_dist:
                            shortest_dist = dist
                            best_start_cell = wire_cell
                            best_sink = unrouted_sink
    else:
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
                    dist = manhattan(unrouted_sink, wire_cell)
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
            if not (neighbour.isObstruction or neighbour.isSource or neighbour.isSink or neighbour.isRouted
                    or neighbour.isWire or neighbour.isCandidate):
                freedom += 1
    return freedom


def dijkstra_multistep(routing_canvas, n):
    """
    Perform n iterations of Dijkstra's algorithm
    :param routing_canvas: Tkinter canvas
    :param n: number of iterations
    :return: void
    """
    for _ in range(n):
        dijkstra_step(routing_canvas)


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
                    if cand_cell.isSink and cand_cell.netGroup is active_net.source.netGroup and \
                            cand_cell.isRouted is False:
                        # This is a sink for the source cell
                        sink_is_found = True
                        sink_cell = cand_cell
                        active_net.sinksRemaining -= 1
                        sink_cell.routingValue = active_cell.dist_from_source+1
                        sink_cell.prev_cell = active_cell
                        active_cell.next_cell.append(sink_cell)
                        break
                    cell_is_viable = not cand_cell.isObstruction and not cand_cell.isCandidate \
                        and not cand_cell.isWire and not cand_cell.isSource and not cand_cell.isSink
                    if cell_is_viable:
                        # Note cell as a candidate for the routing path and add it to the wavefront
                        cand_cell.isCandidate = True
                        cand_cell.dist_from_source = active_cell.dist_from_source+1
                        cand_cell.routingValue = cand_cell.dist_from_source + manhattan(cand_cell, target_sink)
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
        net_colour = NET_COLOURS[sink_cell.netGroup]  # Needed to colour wires
        backtrace_cell = sink_cell
        while not net_is_routed:
            # Backtrace
            if ((backtrace_cell.isRouted and active_net.initRouteComplete) or backtrace_cell.isSource) \
                    and backtrace_cell.netGroup == active_net.num:
                # Done
                net_is_routed = True
            elif backtrace_cell.isCandidate:
                backtrace_cell.isCandidate = False
                backtrace_cell.isWire = True
                backtrace_cell.netGroup = active_net.num
                backtrace_cell.isRouted = True
                routing_canvas.itemconfigure(backtrace_cell.id, fill=net_colour)
                active_net.wireCells.append(backtrace_cell)
            elif backtrace_cell.isSink:
                backtrace_cell.isRouted = True
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
            if current_net_order_idx + 1 < len(net_order):
                # Move to the next net
                current_net_order_idx += 1
                active_net = net_dict[net_order[current_net_order_idx]]
            else:
                # All nets are routed
                print("Routing attempt complete")
                print("Failed nets: " + str(failed_nets))
                done_routing_attempt = True
        else:
            # Route the next sink
            active_net.initRouteComplete = True


def dijkstra_step(routing_canvas):
    """
    Perform one iteration of Dijkstra's algorithm.
    :param routing_canvas: Tkinter canvas
    :return: void
    """
    global active_net
    global wavefront
    global text_id_list
    global done_routing_attempt
    global current_net_order_idx
    global num_segments_routed

    if not isinstance(wavefront, list):
        return

    active_wavefront = wavefront.copy()  # Avoid overwrite and loss of data
    wavefront.clear()  # Will have a new wavefront after Dijkstra step

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
                    if cand_cell.isSink and cand_cell.netGroup is active_net.source.netGroup and \
                            cand_cell.isRouted is False:
                        # This is a sink for the source cell
                        sink_is_found = True
                        sink_cell = cand_cell
                        active_net.sinksRemaining -= 1
                        sink_cell.routingValue = active_cell.routingValue + 1  # Makes backtrace easier if sink has this
                        break
                    cell_is_viable = not cand_cell.isObstruction and not cand_cell.isCandidate \
                        and not cand_cell.isWire and not cand_cell.isSource and not cand_cell.isSink
                    if cell_is_viable:
                        # Note cell as a candidate for the routing path and add it to the wavefront
                        cand_cell.isCandidate = True
                        cand_cell.routingValue = active_cell.routingValue + 1
                        wavefront.append((cand_x, cand_y))
                        # Add routing value to rectangle
                        text_id = add_text(routing_canvas, cand_cell)
                        text_id_list.append(text_id)  # For later text deletion

    if sink_is_found:
        # Connect sink to source
        print("Connecting sink at " + str(sink_cell.x) + ", " + str(sink_cell.y))
        net_is_routed = False
        net_colour = NET_COLOURS[sink_cell.netGroup]  # Needed to colour wires
        search_cell = sink_cell
        routing_path = [sink_cell]
        while not net_is_routed:
            # Backtrace through shortest path from Dijkstra wavefront propagation
            search_x = search_cell.x
            search_y = search_cell.y
            search_coords = [(search_x, search_y+1), (search_x, search_y-1),
                             (search_x+1, search_y), (search_x-1, search_y)]
            for (route_x, route_y) in search_coords:
                if 0 <= route_x < array_width and 0 <= route_y < array_height:
                    backtrace_cell = routing_array[route_x][route_y]
                    if ((backtrace_cell.isRouted and active_net.initRouteComplete) or backtrace_cell.isSource) \
                            and backtrace_cell.netGroup == active_net.num:
                        # Done
                        net_is_routed = True
                        break
                    if backtrace_cell.isCandidate and backtrace_cell.routingValue == search_cell.routingValue-1:
                        # Cell is a valid wire location
                        backtrace_cell.isCandidate = False
                        backtrace_cell.isWire = True
                        backtrace_cell.netGroup = active_net.num
                        routing_path.append(backtrace_cell)
                        routing_canvas.itemconfigure(backtrace_cell.id, fill=net_colour)
                        active_net.wireCells.append(backtrace_cell)
                        # Continue backtrace from this cell
                        search_cell = backtrace_cell
                        break

        # Increment counter for number of routed segments in current circuit routing attempt
        num_segments_routed += 1

        # Mark routed cells as such
        for cell in routing_path:
            cell.isRouted = True

        # Clear non-wire cells
        cleanup_candidates(routing_canvas)

        # Clear/increment active variables
        wavefront = None
        if active_net.sinksRemaining < 1:
            if current_net_order_idx + 1 < len(net_order):
                # Move to the next net
                current_net_order_idx += 1
                active_net = net_dict[net_order[current_net_order_idx]]
            else:
                # All nets are routed
                print("Routing attempt complete")
                print("Failed nets: " + str(failed_nets))
                done_routing_attempt = True
        else:
            # Route the next sink
            active_net.initRouteComplete = True
            pass


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
            if cell.isCandidate:
                cell.isCandidate = False
                cell.routingValue = 0
                cell.hasPropagated = False
                routing_canvas.itemconfigure(cell.id, fill='white')
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
        source_cell.netGroup = net_num
        new_net = Net(source=source_cell, num=net_num)
        # Add sinks
        for idx in range(3, 3+2*(num_pins-1)):
            if idx % 2 == 1:
                # Create sink cell
                cell_x = int(net_tokens[idx])
                cell_y = int(net_tokens[idx + 1])
                sink_cell = routing_grid[cell_x][cell_y]
                sink_cell.isSink = True
                sink_cell.netGroup = net_num
                # Add sink cell to a net
                new_net.sinks.append(sink_cell)
                new_net.sinksRemaining += 1
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
