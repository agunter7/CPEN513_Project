from tkinter import *
from math import floor
import os
import time

BENCHMARK_NAME = "../benchmarks/custom/Testmark"
NET_COLOURS = ["red", "grey", "orange", "purple", "pink", "green", "medium purple", "yellow", "deep sky blue", "chocolate3"]
ARRAY_WIDTH = 25
ARRAY_HEIGHT = 25
CELL_LENGTH = 15

# Global variables
_array = [0] * (ARRAY_WIDTH * ARRAY_HEIGHT)
_routing_canvas = None


def main():
    global _routing_canvas

    # Setup Tkinter
    root = Tk()
    root.columnconfigure(0, weight=1)
    root.rowconfigure(0, weight=1)

    # Setup routing canvas
    _routing_canvas = Canvas(root, bg='white', width=ARRAY_WIDTH*CELL_LENGTH, height=ARRAY_HEIGHT*CELL_LENGTH)
    _routing_canvas.focus_set()
    _routing_canvas.bind("<Key>", lambda event: key_handler(event))
    _routing_canvas.bind("<Button-1>", create_obstruction)
    _routing_canvas.pack()
    for x in range(0, ARRAY_WIDTH):
        for y in range(0, ARRAY_HEIGHT):
            _routing_canvas.create_rectangle(CELL_LENGTH*x, CELL_LENGTH*y, CELL_LENGTH*(x+1), CELL_LENGTH*(y+1),
                                             fill="white")

    root.mainloop()


def key_handler(event):
    e_char = event.char
    if e_char == 's':
        save_benchmark()
    elif str.isdigit(e_char):
        # Add a node for a net
        # Net ID corresponds with digit entered
        cv = event.widget
        xloc = int(event.x / CELL_LENGTH)
        yloc = int(event.y / CELL_LENGTH)
        _array[yloc * ARRAY_WIDTH + xloc] = int(event.char)
        cv.create_rectangle(CELL_LENGTH * xloc, CELL_LENGTH * yloc, CELL_LENGTH * (xloc + 1), CELL_LENGTH * (yloc + 1),
                            fill=NET_COLOURS[int(event.char)])
        cv.create_text(CELL_LENGTH * (xloc + 0.5), CELL_LENGTH * (yloc + 0.5), text=int(event.char))
    else:
        pass


def save_benchmark():
    global _array
    global _routing_canvas

    print("Saving to file " + BENCHMARK_NAME + ".infile")

    with open(BENCHMARK_NAME + time.strftime("-%d-%m-%YT%H-%M-%S") + ".infile", 'w') as f:
        # Write grid dimensions to file
        f.write(str(ARRAY_WIDTH) + " " + str(ARRAY_HEIGHT) + "\n")

        # Gather all obstructed cells
        obstructions = []
        for i in range(0, len(_array)):
            if _array[i] == -1:
                obstructions.append([i - ARRAY_WIDTH * floor((i / ARRAY_WIDTH)), i / ARRAY_WIDTH])

        # Write obstructed cell data to file
        f.write(str(len(obstructions)) + "\n")
        for obstruction in obstructions:
            f.write(str(int(obstruction[0])) + " " + str(int(obstruction[1])) + "\n")

        # Gather all nets
        nets = []
        for j in range(1, 9):
            net = []
            for i in range(0, len(_array)):
                if _array[i] == j:
                    net.append([i - ARRAY_WIDTH * floor((i / ARRAY_WIDTH)), i / ARRAY_WIDTH])
            if len(net) > 0:
                nets.append(net)

        # Write net data to file
        f.write(str(len(nets)) + "\n")
        for net in nets:
            net_signature = [len(net)]
            for node in net:
                net_signature.append(node[0])
                net_signature.append(node[1])
            for item in net_signature:
                f.write(str(int(item)) + " ")
            f.write("\n")
    print("Save complete.")


def create_obstruction(event):
    global _array

    cv = event.widget
    xloc = int(event.x / CELL_LENGTH)
    yloc = int(event.y / CELL_LENGTH)
    _array[yloc * ARRAY_WIDTH + xloc] = -1
    cv.create_rectangle(CELL_LENGTH * xloc, CELL_LENGTH * yloc, CELL_LENGTH * (xloc + 1), CELL_LENGTH * (yloc + 1),
                        fill="blue")


if __name__ == "__main__":
    main()
