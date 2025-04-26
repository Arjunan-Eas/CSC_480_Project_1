import sys
from collections import deque

DIRECTIONS = {(-1,0) : 'N', (1,0) : 'S', (0,-1) : 'W', (0,1) : 'E'}

class Node:
    # Node class holds the coordinates of the node in the format (row, col)
    def __init__(self, y: int, x: int, prev: "Node"):
        self.y = y
        self.x = x
        self.coords = (y,x)
        self.prev = prev

def world_parser():
    with open(sys.argv[2], 'r', encoding="utf-16") as f:
        # Read in data from the file
        columns = int(f.readline())
        rows = int(f.readline())
        world = f.readlines()

        for i in range(rows):
            # Removes newline characters
            world[i] = world[i][:columns]
            for j in range(columns):
                # Finds robot start location
                if world[i][j] == "@":
                    vacuum_start = Node(i, j, None) # Format: (row, column)
                    break

        # Return a tuple of information to start the search
        return(world, vacuum_start, rows, columns)

def depth_first(path: list, world: list, current_location: Node, rows: int, columns: int, visited: list, nodes_generated: int, nodes_expanded: int):
    
    # If already visited, continue to backtrack
    if current_location.coords in visited:
        return path, current_location.prev, visited, nodes_generated, nodes_expanded

    # Otherwise, explore this point
    visited.append(current_location.coords)
    nodes_expanded += 1

    # If point is dirty, add vacuuming action to path
    if world[current_location.y][current_location.x] == '*':
        path.append("V")
    
    # Adds all possible neighbors to a list   
    neighbors = [Node(current_location.y - 1, current_location.x, current_location),
                 Node(current_location.y + 1, current_location.x, current_location),
                 Node(current_location.y, current_location.x - 1, current_location),
                 Node(current_location.y, current_location.x + 1, current_location)]

     # Filters out-of-bounds points, visited points, and blocked points
    for neighbor in neighbors:
        if((0 <= neighbor.y < rows) and (0 <= neighbor.x < columns) and (world[neighbor.y][neighbor.x] != "#") and (not neighbor.coords in visited)):
            # All traversable points count as generated
            nodes_generated += 1
            
            # Move in the direction of the neighbor. Appends cardinal direction of move to path
            path.append(DIRECTIONS[(neighbor.y - current_location.y, neighbor.x - current_location.x)])
            path, _, visited, nodes_generated, nodes_expanded = depth_first(path, world, neighbor, rows, columns, visited, nodes_generated, nodes_expanded)

            # Back track once recursive call returns. Moves in the opposite direction
            path.append(DIRECTIONS[(current_location.y - neighbor.y, current_location.x - neighbor.x)]) 
    
    return path, current_location, visited, nodes_generated, nodes_expanded


def uniform_cost(world: list, vacuum_start: Node, rows: int, columns: int):
    # List storing the directions for ideal vacuum traversal
    path = []
    
    # Keeps track of dirty cells that have been found
    dirty_cells = []
    # Keeps track of dirty cells that have been cleaned
    cleaned = []
    # Queue for BFS traversal
    queue = deque()
    queue.append(vacuum_start)

    # Initializes node counters
    nodes_generated = 0
    nodes_expanded = 0

    # Loops until entire world is traversed and all vacuum spots are found
    while True:
        # Resets visited every iteration
        visited = []
        # Traverses world
        while(len(queue) != 0):        
            current_location = queue.popleft()
            visited.append(current_location.coords)
            # If node is traversed, it is considered expanded
            nodes_expanded += 1

            # If a dirty cell is found, break out of the loop to capture path from start to the cell
            if(world[current_location.y][current_location.x] == "*" and not current_location.coords in dirty_cells):
                dirty_cells.append(current_location.coords)
                break

            # Adds all possible neighbors to a list   
            neighbors = [Node(current_location.y - 1, current_location.x, current_location),
                         Node(current_location.y + 1, current_location.x, current_location),
                         Node(current_location.y, current_location.x - 1, current_location),
                         Node(current_location.y, current_location.x + 1, current_location)]
            
            
            # Filters out-of-bounds points, visited points, and blocked points
            for neighbor in neighbors:
                if((0 <= neighbor.y < rows) and (0 <= neighbor.x < columns) and (world[neighbor.y][neighbor.x] != "#") and not neighbor.coords in visited):
                    # All traversable points count as generated
                    nodes_generated += 1
                    # All valid neighbor nodes are added to generated list
                    queue.append(neighbor)
        
        # Ends loop if entire world is traversed and no new vacuum spots are found
        if(len(queue) == 0 and world[current_location.y][current_location.x] != "*"):
            break
        
        # Stores ending location in tempNode
        tempNode = Node(current_location.y, current_location.x, None)
        
        # Traces back path from vacuum node to start
        temp_path = []
        while(not current_location.prev is None):
            # Makes sure to keep track of which spots are cleaned already
            if(world[current_location.y][current_location.x] == "*" and not (current_location.coords in cleaned)):
                cleaned.append(current_location.coords)
                temp_path.append("V")
            temp_path.append(DIRECTIONS[current_location.y - current_location.prev.y, current_location.x - current_location.prev.x])
            current_location = current_location.prev
        
        # Reverses segment so it goes from start -> finish, adds to overall path
        path += temp_path[::-1]
        
        # If the whole world was traversed, and the last node happened to be a vacuum, end loop
        if(len(queue) == 0):
            break
        else:
            # Reset the queue and search for more vacuum nodes
            queue.clear()
            queue.append(tempNode)

    return path, nodes_generated, nodes_expanded


if __name__ == "__main__":
    if(len(sys.argv) != 3):
        print("Please run like so: python3 vacuum_bot.py <algorithm> <world_file>")
    elif(sys.argv[1] == "depth_first"):
        # Parses world
        world, vacuum_start, rows, columns = world_parser()
        
        # Starts recursive DFS on the world. Returns the entire traversal
        path, current_location, visited, nodes_generated, nodes_expanded = depth_first([], world, vacuum_start, rows, columns, [], 0, 0)
        
        # We only care about traversing until the last vacuum
        for i in range(len(path)):
            # Finds last vacuuming action
            if path[len(path) - i - 1] == "V":
                # Slices path to only include up until that point
                # Note that we are guarenteed this is the last vacuuming action because the whole graph was traversed
                path = path[:len(path) - i]
                break

        # Prints out moves sequentially
        for move in path:
            print(move)
        print(f"Nodes generated: {nodes_generated}")
        print(f"Nodes expanded: {nodes_expanded}")

    elif(sys.argv[1] == "uniform_cost"):
        # Parses world
        world, vacuum_start, rows, columns = world_parser()

        path, nodes_generated, nodes_expanded = uniform_cost(world, vacuum_start, rows, columns)

        for dir in path:
            print(dir)
        print(f"Nodes generated: {nodes_generated}")
        print(f"Nodes expanded: {nodes_expanded}")