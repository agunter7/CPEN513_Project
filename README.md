# CPEN513_Project
UBC CPEN 513 CAD Algorithms for Integrated Circuits Project

A Reinforcement Learning
Approach to Rip-up and Reroute in 2D Maze
Routing through Resource Arbitration

Caroline White & Andrew Gunter

# Usage

## Dependencies
This project depends on [Stable-Baselines3](https://github.com/DLR-RM/stable-baselines3)

Python 3.8

## Running the RL Algorithm
Configure runtime parameters in source code:

FILE_PATH -- Path to the .infile circuit to route

TRAIN_TIME_STEPS -- If training an agent, number of time steps to train for

Run src/main_rl.py with a Python 3.8 interpreter

A routing window should be displayed upon running the file. This window will update when the agent has finished running, for each of the training and testing versions.

### Training

Press 'l' to run training. Number of time steps used can be configured within the file. Upon completion, a .zip file will appear in the directory from which you run.

### Testing

Rename the learning output .zip file to 'smart_model.zip' or change the appropriate variable (LOAD_MODEL_NAME) 
in the source file. Press 't' to run routing with the trained RL agent.

## Running PathFinder
Run src/pathfinder.py with a Python 3.8 interpreter

A routing window should be displayed upon running the file.

Press a number key indicating
how many steps you would like the algorithm to perform. 
E.g. Pressing '1' triggers a single step while '5' triggers five steps.
Pressing '0' will run the entire program to completion, including all rip-up and reroute stages.
