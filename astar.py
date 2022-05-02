class Node():
    """ A node closs for A* Search """

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.f = 0
        self.h = 0

    def __eq__(self, other):
        return self.position == other.position

def astar(maze, start, end):
    """
    This function returns a list of points, (X,Y), as a path from a given start point to a given end point
    The maze is also provided to the algorithm in the form of 0's and 1's
    0 for available nodes and 1 for obstacles
    """

    # Initialize the starting and ending locations
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0

    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Create the open and closed lists and add "Start"
    open_nodes = []
    closed_nodes = []
    open_nodes.append(start_node)

    # Run search until end_node is reached
    # This is achieved by monitoring the length of the open nodes list

    while len(open_nodes) > 0:

        current_node = open_nodes[0]
        current_index = 0
        for index, item in enumerate(open_nodes):
            # Check f value of new node vs. current mode
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Manage the population of the open and closed nodes lists
        open_nodes.pop(current_index)
        closed_nodes.append(current_node)

        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path

        # This is where we test the next set of possible moves
        children = []

        # This is a list of all possible moves from the current location
        possible = [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]

        for new_position in possible:
            # Get the next node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure the move is within range
            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) -1) or node_position[1] < 0:
                continue

            # Make sure we have not crossed an obstacle terrain
            if maze[node_position[0]][node_position[1]] != 0:
                continue

            # Create new node
            new_node = Node(current_node, node_position)
            children.append(new_node)

        for child in children:

            # Child is on the closed list
            for closed_child in closed_nodes:
                if child == closed_child:
                    continue

            # Create the f, g, and h values
            child.g = current_node.g + 1

            # H is calculated as the euclidean distance
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h

            # Child is already in the open list
            for open_node in open_nodes:
                if child == open_node and child.g > open_node.g:
                    continue

            # Add the child to the open list
            open_nodes.append(child)


def main():

    maze = [[0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]

    start = (0, 0)
    end = (7, 6)

    path = astar(maze, start, end)
    print(path)


if __name__ == '__main__':
    main()
