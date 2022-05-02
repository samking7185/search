
import random
import math

from typing import Tuple, List, Dict, Any


class RRTstar:
    """
    RRT* Path Planning Algorithm
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
            self.cost = 0.0

        @property
        def get_position(self) -> Tuple:
            return self.x, self.y

        @property
        def get_int_position(self) -> Tuple:
            return round(self.x), round(self.y)

        @property
        def get_floor_position(self) -> Tuple:
            return math.floor(self.x), math.floor(self.y)

        @property
        def get_ceil_position(self) -> Tuple:
            return math.ceil(self.x), math.ceil(self.y)

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
        self.search_radius = None
        self.Xlimit = len(self.layout[0])
        self.Ylimit = len(self.layout)
        # Initialize an list of obstacle coordinates
        self.obstacles = [(i, j) for j, row in enumerate(self.layout) for i, val in enumerate(row) if row[i] != 0]

    def search(self, params: Dict[str, Any]):
        """
        RRTstar Search Algorithm
        :param params, a dictionary of search parameters
                params = {"start": (x1, y1), (Start coordinate)
                        "goal": (x2, y2),    (Goal Coordinate)
                        "max_iterations": n,
                        "resolution": dstep, (Max Step Size for path construction)
                        "max_distance": dmax (Max Length for path segments)
                        "search_radius": r (Search radius for finding lower cost
                        }
        """

        # Set search parameters
        self.start = self.Node(params.get("start", (0, 0)))
        self.goal = self.Node(params.get("goal", (self.Xlimit, self.Ylimit)))
        self.limit = params.get("max_iterations", 100000)
        self.resolution = params.get("resolution", 0.5)
        self.max_distance = params.get("max_distance", 3)
        self.search_radius = params.get("search_radius", 50.0)
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
            Qnear = self.node_list[nearest_idx]

            Qnext.cost = Qnear.cost + math.hypot(Qnew.x - Qnext.x, Qnew.y - Qnext.y)
            if Qnext.get_int_position or Qnext.get_floor_position or Qnext.get_ceil_position not in self.obstacles:
                self.node_list.append(Qnext)
                near_idx = self.find_near_nodes(Qnext)
                Qparent = self.choose_parent(Qnext, near_idx)
                if Qparent:
                    self.rewire(Qparent, near_idx)
                    self.node_list.append(Qparent)
                else:
                    self.node_list.append(Qnext)
            error_list = [(x - y)**2 for x,y in zip(Qnext.get_position_list, self.goal.get_position_list)]
            error = sum(error_list)
            if error**0.5 < 0.001:  # if reaches goal
                print('made it')
                last_index = self.search_best_goal_node()
                if last_index is not None:
                    return self.generate_final_course(last_index)

        print("reached max iteration")

        last_index = self.search_best_goal_node()
        if last_index is not None:
            return self.generate_final_course(last_index)

        return None

    def search_best_goal_node(self):
        dist_to_goal_list = [
            self.calc_dist_to_goal(n.x, n.y) for n in self.node_list
        ]
        goal_inds = [
            dist_to_goal_list.index(i) for i in dist_to_goal_list
            if i <= self.max_distance
        ]

        safe_goal_inds = []
        for goal_ind in goal_inds:
            t_node = self.create_path(self.node_list[goal_ind], self.goal)
            if t_node.get_int_position or t_node.get_floor_position or t_node.get_ceil_position not in self.obstacles:
                safe_goal_inds.append(goal_ind)

        if not safe_goal_inds:
            return None

        min_cost = min([self.node_list[i].cost for i in safe_goal_inds])
        for i in safe_goal_inds:
            if self.node_list[i].cost == min_cost:
                return i

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

    def find_near_nodes(self, new_node):
        """
        1) defines a ball centered on new_node
        2) Returns all nodes of the three that are inside this ball
           :param new_node: Node
                    new randomly generated node, without collisions between
                    its nearest node
        """
        nnode = len(self.node_list) + 1
        r = self.search_radius * math.sqrt((math.log(nnode) / nnode))
        # if expand_dist exists, search vertices in a range no more than
        # expand_dist
        if hasattr(self, 'max_distance'):
            r = min(r, self.max_distance)
        dist_list = [(node.x - new_node.x)**2 + (node.y - new_node.y)**2 for node in self.node_list]
        near_idx = [dist_list.index(i) for i in dist_list if i <= r**2]
        return near_idx

    def choose_parent(self, new_node, near_idx):
        """
        Computes the cheapest point to new_node contained in the list
        near_idx and set such a node as the parent of new_node.
        :param new_node, Node
            randomly generated node with a path from its neared point
        :param near_idx: list
            Indices of indices of the nodes what are near to new_node
        """
        if not near_idx:
            return None

        # search nearest cost in near_idx
        costs = []
        for i in near_idx:
            near_node = self.node_list[i]
            t_node = self.create_path(near_node, new_node)
            if t_node.get_int_position or t_node.get_floor_position or t_node.get_ceil_position not in self.obstacles:
                d, _ = self.distance_heading(near_node, new_node)
                costs.append(near_node.cost + d)
            else:
                costs.append(float("inf"))  # the cost of collision node
        min_cost = min(costs)

        if min_cost == float("inf"):
            print("There is no good path.(min_cost is inf)")
            return None

        min_ind = near_idx[costs.index(min_cost)]
        new_node = self.create_path(self.node_list[min_ind], new_node)
        new_node.cost = min_cost

        return new_node

    def rewire(self, new_node, near_idx):
        """
            For each node in near_idx, this will check if it is cheaper to
            arrive to them from new_node.
            In such a case, this will re-assign the parent of the nodes in
            near_idx to new_node.
            Parameters:
           :param new_node, Node
                Node randomly added which can be joined to the tree
           :param near_idx, list of uints
                    A list of indices of the self.new_node which contains
                    nodes within a circle of a given radius.
        """
        for i in near_idx:
            near_node = self.node_list[i]
            edge_node = self.create_path(new_node, near_node)
            if not edge_node:
                continue
            d, _ = self.distance_heading(near_node, new_node)

            edge_node.cost = d + new_node.cost

            no_collision = edge_node.get_int_position or edge_node.get_floor_position or edge_node.get_ceil_position not in self.obstacles
            improved_cost = near_node.cost > edge_node.cost

            if no_collision and improved_cost:
                near_node.x = edge_node.x
                near_node.y = edge_node.y
                near_node.cost = edge_node.cost
                near_node.path_x = edge_node.path_x
                near_node.path_y = edge_node.path_y
                near_node.parent = edge_node.parent
                self.propagate_cost_to_leaves(new_node)

    def propagate_cost_to_leaves(self, parent_node):
        # Use recursion to update cost of all leaves
        for node in self.node_list:
            if node.parent == parent_node:
                d, _ = self.distance_heading(parent_node, node)
                node.cost = d + parent_node.cost
                self.propagate_cost_to_leaves(node)

    def calc_dist_to_goal(self, x, y):
        dx = x - self.goal.x
        dy = y - self.goal.y
        return math.hypot(dx, dy)

    def generate_final_course(self, goal_ind):
        path = [[self.goal.x, self.goal.y]]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])

        return path
