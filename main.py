import time
import math
from queue import Queue, PriorityQueue, LifoQueue  # LifoQueue will be used for DFS
import tkinter as tk
from tkinter import messagebox

# Define the goal state and possible moves
GOAL_STATE = [0, 1, 2, 3, 4, 5, 6, 7, 8]
DIRECTIONS = {'Up': -3, 'Down': 3, 'Left': -1, 'Right': 1}

def is_goal(state):
    return state == GOAL_STATE

def get_possible_moves(state):
    blank_idx = state.index(0)
    possible_moves = []
    for direction, offset in DIRECTIONS.items():
        neighbor = blank_idx + offset
        if 0 <= neighbor < 9:
            if direction in ['Left', 'Right'] and (blank_idx // 3 != neighbor // 3):
                continue
            new_state = state[:]
            new_state[blank_idx], new_state[neighbor] = new_state[neighbor], new_state[blank_idx]
            possible_moves.append((new_state, direction))
    return possible_moves

# Heuristic functions for A* Search
def manhattan_distance(state):
    distance = 0
    for idx, value in enumerate(state):
        if value != 0:
            target_idx = GOAL_STATE.index(value)
            distance += abs(idx // 3 - target_idx // 3) + abs(idx % 3 - target_idx % 3)
    return distance

# BFS Algorithm with node expansion count
def bfs(initial_state):
    queue = Queue()
    queue.put((initial_state, []))
    visited = set()
    visited.add(tuple(initial_state))
    nodes_expanded = 0
    while not queue.empty():
        current_state, path = queue.get()
        nodes_expanded += 1
        if is_goal(current_state):
            return path, len(path), nodes_expanded
        for next_state, move in get_possible_moves(current_state):
            if tuple(next_state) not in visited:
                visited.add(tuple(next_state))
                queue.put((next_state, path + [move]))
    return None, None, nodes_expanded

# DFS Algorithm with node expansion count
def dfs(initial_state):
    stack = LifoQueue()
    stack.put((initial_state, []))
    visited = set()
    visited.add(tuple(initial_state))
    nodes_expanded = 0
    while not stack.empty():
        current_state, path = stack.get()
        nodes_expanded += 1
        if is_goal(current_state):
            return path, len(path), nodes_expanded
        for next_state, move in get_possible_moves(current_state):
            if tuple(next_state) not in visited:
                visited.add(tuple(next_state))
                stack.put((next_state, path + [move]))
    return None, None, nodes_expanded

# Greedy Best-First Search Algorithm with node expansion count
def greedy(initial_state):
    frontier = PriorityQueue()
    frontier.put((manhattan_distance(initial_state), initial_state, []))
    visited = set()
    visited.add(tuple(initial_state))
    nodes_expanded = 0
    while not frontier.empty():
        heuristic_cost, current_state, path = frontier.get()
        nodes_expanded += 1
        if is_goal(current_state):
            return path, len(path), nodes_expanded
        for next_state, move in get_possible_moves(current_state):
            if tuple(next_state) not in visited:
                visited.add(tuple(next_state))
                frontier.put((manhattan_distance(next_state), next_state, path + [move]))
    return None, None, nodes_expanded

# A* Search Algorithm with node expansion count
def a_star(initial_state, heuristic_func):
    frontier = PriorityQueue()
    frontier.put((0, initial_state, []))
    visited = set()
    visited.add(tuple(initial_state))
    nodes_expanded = 0
    while not frontier.empty():
        cost, current_state, path = frontier.get()
        nodes_expanded += 1
        if is_goal(current_state):
            return path, len(path), nodes_expanded
        for next_state, move in get_possible_moves(current_state):
            if tuple(next_state) not in visited:
                visited.add(tuple(next_state))
                heuristic_cost = cost + 1 + heuristic_func(next_state)
                frontier.put((heuristic_cost, next_state, path + [move]))
    return None, None, nodes_expanded

# UCS Algorithm with node expansion count
def ucs(initial_state):
    frontier = PriorityQueue()
    frontier.put((0, initial_state, []))
    visited = set()
    visited.add(tuple(initial_state))
    nodes_expanded = 0
    while not frontier.empty():
        cost, current_state, path = frontier.get()
        nodes_expanded += 1
        if is_goal(current_state):
            return path, len(path), nodes_expanded
        for next_state, move in get_possible_moves(current_state):
            if tuple(next_state) not in visited:
                visited.add(tuple(next_state))
                frontier.put((cost + 1, next_state, path + [move]))
    return None, None, nodes_expanded

# GUI functions
def display_board(state):
    for i, tile in enumerate(state):
        buttons[i].config(text=str(tile) if tile != 0 else "", state=tk.NORMAL if tile == 0 else tk.DISABLED)

def solve_with_algorithm(algorithm):
    initial_state = [int(entry.get()) for entry in entries]
    if set(initial_state) != set(range(9)):
        messagebox.showerror("Invalid Input", "Please enter numbers 0-8 without repetition.")
        return

    start_time = time.time()
    path, cost, nodes_expanded = None, None, 0
    if algorithm == 'BFS':
        path, cost, nodes_expanded = bfs(initial_state)
    elif algorithm == 'DFS':
        path, cost, nodes_expanded = dfs(initial_state)
    elif algorithm == 'A* Manhattan':
        path, cost, nodes_expanded = a_star(initial_state, manhattan_distance)
    elif algorithm == 'UCS':
        path, cost, nodes_expanded = ucs(initial_state)
    elif algorithm == 'Greedy':
        path, cost, nodes_expanded = greedy(initial_state)

    elapsed_time = time.time() - start_time

    if path is None:
        messagebox.showinfo("No Solution", "No solution found.")
    else:
        display_solution(initial_state, path)
        path_label.config(text=f"Path: {' -> '.join(path)}")
        cost_label.config(text=f"Cost: {cost}")
        time_label.config(text=f"Time: {elapsed_time:.2f} seconds")
        nodes_label.config(text=f"Nodes Expanded: {nodes_expanded}")

def display_solution(initial_state, path):
    display_board(initial_state)
    current_state = initial_state[:]
    for move in path:
        apply_move_and_display(current_state, move)
        root.update()
        time.sleep(0.5)

def apply_move_and_display(state, move):
    blank_idx = state.index(0)
    offset = DIRECTIONS[move]
    new_blank_idx = blank_idx + offset
    state[blank_idx], state[new_blank_idx] = state[new_blank_idx], state[blank_idx]
    display_board(state)

# Create the Tkinter GUI
root = tk.Tk()
root.title("8-Puzzle Solver")
frame = tk.Frame(root)
frame.pack()

entries = []
buttons = []

# Create the puzzle grid
for i in range(9):
    entry = tk.Entry(frame, width=2, font=('Arial', 18), justify='center')
    entry.grid(row=i//3, column=i%3, padx=5, pady=5)
    entries.append(entry)

# Create algorithm buttons
tk.Button(root, text="Solve with BFS", command=lambda: solve_with_algorithm('BFS')).pack(pady=5)
tk.Button(root, text="Solve with DFS", command=lambda: solve_with_algorithm('DFS')).pack(pady=5)
tk.Button(root, text="Solve with Greedy", command=lambda: solve_with_algorithm('Greedy')).pack(pady=5)
tk.Button(root, text="Solve with A* Manhattan", command=lambda: solve_with_algorithm('A* Manhattan')).pack(pady=5)
tk.Button(root, text="Solve with UCS", command=lambda: solve_with_algorithm('UCS')).pack(pady=5)

# Display buttons for moves
display_frame = tk.Frame(root)
display_frame.pack()
for i in range(9):
    button = tk.Button(display_frame, width=2, height=2, font=('Arial', 18), state=tk.DISABLED)
    button.grid(row=i//3, column=i%3, padx=5, pady=5)
    buttons.append(button)

# Labels to show the path, cost, time, and nodes expanded
path_label = tk.Label(root, text="Path: ", font=('Arial', 12))
path_label.pack(pady=5)

cost_label = tk.Label(root, text="Cost: ", font=('Arial', 12))
cost_label.pack(pady=5)

time_label = tk.Label(root, text="Time: ", font=('Arial', 12))
time_label.pack(pady=5)

nodes_label = tk.Label(root, text="Nodes Expanded: ", font=('Arial', 12))
nodes_label.pack(pady=5)

root.mainloop()
