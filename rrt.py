
import random
import math

from typing import Tuple, List, Dict, Any


class RRT:
    """
    RRT Path Planning Algorithm
    :param layout, a list of lists that generate a search space for the algorithm
            layout uses "0" for usable space and "1" for obstacles
            ex. Layout
            layout = [
                    [0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0]
                    ]
    """
    class Node:
        """
        Node for RRT search
        """
        def __init__(self, x: Tuple):
            self.x = x[0]
            self.y = x[1]
            self.path_x = []
            self.path_y = []
            self.parent = None

        @property
        def get_position(self) -> Tuple:
            return self.x, self.y

        @property
        def get_int_position(self) -> Tuple:
            return round(self.x), round(self.y)

        @property
        def get_position_list(self) -> List:
            return [round(self.x), round(self.y)]

    def __init__(self, layout):
        self.layout = list(reversed(layout))
        self.goal = None
        self.start = None
        self.limit = None
        self.resolution = None
        self.max_distance = None
        self.node_list = None
        self.Xlimit = len(self.layout[0])
        self.Ylimit = len(self.layout)
        # Initialize an list of obstacle coordinates
        self.obstacles = [(i, j) for j, row in enumerate(self.layout) for i, val in enumerate(row) if row[i] != 0]

    def search(self, params: Dict[str, Any]):
        """
        RRT Search Algorithm
        :param params, a dictionary of search parameters
                params = {"start": (x1, y1), (Start coordinate)
                        "goal": (x2, y2),    (Goal Coordinate)
                        "max_iterations": n,
                        "resolution": dstep, (Max Step Size for path construction)
                        "max_distance": dmax (Max Length for path segments)
                        }
        """

        # Set search parameters
        self.start = self.Node(params.get("start", (0, 0)))
        self.goal = self.Node(params.get("goal", (self.Xlimit, self.Ylimit)))
        self.limit = params.get("max_iterations", 1000)
        self.resolution = params.get("resolution", 0.5)
        self.max_distance = params.get("max_distance", 3)

        self.node_list = [self.start]

        for i in range(self.limit):
            # Generate a random node, Qnew
            Xnew = random.randint(0, self.Xlimit)
            Ynew = random.randint(0, self.Ylimit)
            Qnew = self.Node((Xnew, Ynew))

            # Find the index of the node that is closest to Qnew
            nearest = [((Xnew - row.x)**2 + (Ynew - row.y)**2)**0.5 for row in self.node_list]
            nearest_idx = nearest.index(min(nearest))

            # Create a path between Qnew and the nearest node that
            # is not longer than the parameter "max_distance"
            Qnext = self.create_path(self.node_list[nearest_idx], Qnew)

            # If node isn't an obstacle, append the last path node to list
            if Qnext.get_int_position not in self.obstacles:
                self.node_list.append(Qnext)

            # Check distance between end of current path and goal
            dgoal, _ = self.distance_heading(self.node_list[-1], self.goal)

            # If finalized, create a path by chaining together the "parents"
            # of all the nodes in node_list

            if dgoal <= self.resolution:
                Qfinal = self.create_path(self.node_list[-1], self.goal)
                if Qfinal.get_int_position not in self.obstacles:
                    return self.create_final(len(self.node_list) - 1)
        return None

    def create_path(self, from_node: Node, to_node: Node) -> Node:
        # Create a path between nodes
        # Length of the path cannot surpass "max_distance"

        extend_length = self.max_distance
        node1 = self.Node(from_node.get_position)
        d, theta = self.distance_heading(node1, to_node)

        node1.path_x = [node1.x]
        node1.path_y = [node1.y]

        if extend_length > d:
            extend_length = d

        n_exp = math.floor(extend_length / self.resolution)

        for _ in range(n_exp):
            node1.x += self.resolution * math.cos(theta)
            node1.y += self.resolution * math.sin(theta)
            node1.path_x.append(node1.x)
            node1.path_y.append(node1.y)

        d, theta = self.distance_heading(node1, to_node)
        if d <= self.resolution:
            node1.path_x.append(to_node.x)
            node1.path_y.append(to_node.y)
            node1.x = to_node.x
            node1.y = to_node.y
        node1.parent = from_node

        return node1

    def create_final(self, goal_idx) -> List:
        # Work backwards from the final node to the beginning
        # Nodes are chained together by their "parent" attribute

        path = [self.goal.get_position_list]
        node = self.node_list[goal_idx]
        while node.parent:
            path.append(node.get_position_list)
            node = node.parent
        path.append(node.get_position_list)

        return path

    @staticmethod
    def distance_heading(n1: Node, n2: Node):
        dx = n2.x - n1.x
        dy = n2.y - n1.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)

        return d, theta
