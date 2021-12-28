import numpy as np

class Node:
    """
        g = cost from start to current Node
        h = heuristic based estimated cost for current Node to goal Node
        f = total cost of present node i.e. :  f = g + h
    """

    def __init__(first, parent=None, position=None):
        first.parent = parent
        first.position = position

        first.g = 0
        first.h = 0
        first.f = 0
    def __eq__(first, other):
        return first.position == other.position
        
def return_path(current_node,maze):
    path = []
    no_rows, no_columns = np.shape(maze)
    # here we create the initialized result maze with -1 in every position
    result = [[-1 for i in range(no_columns)] for j in range(no_rows)]
    current = current_node
    while current is not None:
        path.append(current.position)
        current = current.parent
    # Return reversed path as we need to show from start to end path
    path = path[::-1]
    start_value = 1
  
    for i in range(len(path)):
        result[path[i][0]][path[i][1]] = start_value
        start_value = 1
    return result


def astar(maze, cost, start, end):

    # Create start and end node with initized values for g, h and f
    starting_node = Node(None, tuple(start))
    starting_node.g = starting_node.h = starting_node.f = 0
    goal_node = Node(None, tuple(end))
    goal_node.g = goal_node.h = goal_node.f = 0

    # Initialize both yet_to_visit and visited list
    # in this list we will put all node that are yet_to_visit for exploration. 
    # From here we will find the lowest cost node to expand next
    open_list = []  
    # in this list we will put all node those already explored so that we don't explore it again
    closed_list = [] 
    
    # Add the start node
    open_list.append(starting_node)
    
    # Adding a stop condition. This is to avoid any infinite loop and stop 
    # execution after some reasonable number of steps
    outer_iterations = 0
    max_iterations = (len(maze) // 2) ** 10

    # what squares do we search . serarch movement is left-right-top-bottom 
    #(4 movements) from every positon

    move  =  [[-1, 0 ], # go up
              [ 0, -1], # go left
              [ 1, 0 ], # go down
              [ 0, 1 ]] # go right

    #find maze has got how many rows and columns 
    no_rows, no_columns = np.shape(maze)
    
    # Loop until you find the end
    
    while len(open_list) > 0:
        
        # Every time any node is referred from yet_to_visit list, counter of limit operation incremented
        outer_iterations += 1    

        
        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index
                
        # if we hit this point return the path such as it may be no solution or 
        # computation cost is too high
        if outer_iterations > max_iterations:
            print ("giving up on pathfinding too many iterations")
            return return_path(current_node,maze)

        # Pop current node out off yet_to_visit list, add to visited list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # test if goal is reached or not, if yes then return the path
        if current_node == goal_node:
            return return_path(current_node,maze)

        # Generate children from all adjacent squares
        children = []

        for new_position in move: 

            # Getting node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range (check if within maze boundary)
            if (node_position[0] > (no_rows - 1) or 
                node_position[0] < 0 or 
                node_position[1] > (no_columns -1) or 
                node_position[1] < 0):
                continue

            if maze[node_position[0]][node_position[1]] != 0:
                continue

            # New node Creation
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # for Loop
        for child in children:
            
            # Child is on the closed list 
            if len([visited_child for visited_child in closed_list if visited_child == child]) > 0:
                continue

            # Create the f, g, and h values
            child.g = current_node.g + cost
            ## Heuristic costs calculated here, this is using eucledian distance
            child.h = (((child.position[0] - goal_node.position[0]) ** 2) + 
                       ((child.position[1] - goal_node.position[1]) ** 2)) 

            child.f = child.g + child.h

            # Child is already in the open list and g cost is already lower
            if len([i for i in open_list if child == i and child.g > i.g]) > 0:
                continue

            # Add the child to the open_list
            open_list.append(child)


if __name__ == '__main__':

    maze = [[0, 1, 0, 1, 0, 1],
            [0, 1, 0, 0, 0, 0],
            [0, 1, 0, 1, 1, 0],
            [0, 1, 0, 0, 1, 0],
            [0, 0, 0, 0, 1, 0]]
    
    start = [0, 0] # starting position
    end = [4,5] # ending position
    cost = 1 # cost per movement

    path = astar(maze,cost, start, end)

    print(path)
