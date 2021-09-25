import math
import numpy


# When you write your methods correctly this cell will execute
# without problems
class RoutePlanner:
    """Construct a PathPlanner Object"""

    def __init__(self, M, start=None, goal=None):
        """ """
        self.map = M
        self.start = start
        self.goal = goal
        self.closedSet = self.create_closedSet() if goal != None and start != None else None
        self.openSet = self.create_openSet() if goal != None and start != None else None
        self.cameFrom = self.create_cameFrom() if goal != None and start != None else None
        self.gScore = self.create_gScore() if goal != None and start != None else None
        self.fScore = self.create_fScore() if goal != None and start != None else None
        self.path = self.run_search() if self.map and self.start != None and self.goal != None else None

    def reconstruct_path(self, current):
        """ Reconstructs path after search """
        print(current, 'current in recons')
        print(self.cameFrom, 'came FROM')
        total_path = [current]
        while current in self.cameFrom.keys():
            current = self.cameFrom[current]
            total_path.append(current)
        return total_path

    def _reset(self):
        """Private method used to reset the closedSet, openSet, cameFrom, gScore, fScore, and path attributes"""
        self.closedSet = None
        self.openSet = None
        self.cameFrom = None
        self.gScore = None
        self.fScore = None
        self.path = self.run_search() if self.map and self.start and self.goal else None

    def run_search(self):
        """ """
        if self.map == None:
            raise (ValueError, "Must create map before running search. Try running PathPlanner.set_map(start_node)")
        if self.goal == None:
            raise (
                ValueError, "Must create goal node before running search. Try running PathPlanner.set_goal(start_node)")
        if self.start == None:
            raise (
                ValueError,
                "Must create start node before running search. Try running PathPlanner.set_start(start_node)")

        self.closedSet = self.closedSet if self.closedSet != None else self.create_closedSet()
        self.openSet = self.openSet if self.openSet != None else self.create_openSet()
        self.cameFrom = self.cameFrom if self.cameFrom != None else self.create_cameFrom()
        self.gScore = self.gScore if self.gScore != None else self.create_gScore()
        self.fScore = self.fScore if self.fScore != None else self.create_fScore()

        while not self.is_open_empty():
            current = self.get_current_node()
            print(current, 'Current node')

            if current == self.goal:
                print("SKAD PRZYSZLIUM")
                # self.cameFrom[0] = 1
                print(self.cameFrom)
                if self.path is None:
                    print("SCIEZKA JEST NICZYM")
                # print(numpy.array(self.path))
                self.path = [x for x in reversed(self.reconstruct_path(current))]
                return self.path
            else:
                print("REMOVE FROM OPEN ADD TO CLOSED", current)
                self.openSet.remove(current)
                self.closedSet.add(current)

            for neighbor in self.get_neighbors(current):
                if neighbor in self.closedSet:
                    print("CLOSED ALREADY EVALUATED NEIBOR: ", neighbor)
                    continue  # Ignore the neighbor which is already evaluated.

                if not neighbor in self.openSet:  # Discover a new node
                    print("ADD NEW NEIGHBOR TO OPEN: ", neighbor)
                    self.openSet.add(neighbor)

                print('NEIGHBOR GO NEXT: ', neighbor)
                # The distance from start to a neighbor
                # the "dist_between" function may vary as per the solution requirements.
                print(self.get_tentative_gScore(current, neighbor), 'SELF TANTATI')
                print(self.get_gScore(neighbor), 'SCORE NEIGHBOR')
                if self.get_tentative_gScore(current, neighbor) >= self.get_gScore(neighbor):
                    print("NIE JEST LEPSZA SCIEZKA")
                    continue  # This is not a better path.

                print('RECORDING')
                # This path is the best until now. Record it!
                self.record_best_path_to(current, neighbor)
        print("No Path Found")
        self.path = None
        return False

    def create_closedSet(self):
        """ Creates and returns a data structure suitable to hold the set of nodes already evaluated"""
        # EXAMPLE: return a data structure suitable to hold the set of nodes already evaluated
        return set()

    def create_openSet(self):
        """ Creates and returns a data structure suitable to hold the set of currently discovered nodes
        that are not evaluated yet. Initially, only the start node is known."""

        open_set = set()
        if self.start is not None:
            # TODO: return a data structure suitable to hold the set of currently discovered nodes
            # that are not evaluated yet. Make sure to include the start node.
            open_set.add(self.start)
            self.openSet = open_set
            return self.openSet

        raise (
            ValueError,
            "Must create start node before creating an open set. Try running PathPlanner.set_start(start_node)")

    def create_cameFrom(self):
        """Creates and returns a data structure that shows which node can most efficiently be reached from another,
        for each node."""
        # TODO: return a data structure that shows which node can most efficiently be reached from another,
        # for each node.
        return {}

    def create_gScore(self):
        """Creates and returns a data structure that holds the cost of getting from the start node to that node,
        for each node. The cost of going from start to start is zero."""
        # TODO:  return a data structure that holds the cost of getting from the start node to that node, for each node.
        # for each node. The cost of going from start to start is zero. The rest of the node's values should
        # be set to infinity.
        return {}

    def create_fScore(self):
        """Creates and returns a data structure that holds the total cost of getting from the start node to the goal
        by passing by that node, for each node. That value is partly known, partly heuristic.
        For the first node, that value is completely heuristic."""
        # TODO: return a data structure that holds the total cost of getting from the start node to the goal
        # by passing by that node, for each node. That value is partly known, partly heuristic.
        # For the first node, that value is completely heuristic. The rest of the node's value should be
        # set to infinity.
        return {}

    def set_map(self, M):
        """Method used to set map attribute """
        self._reset()
        self.start = None
        self.goal = None
        self.map = M

    def set_start(self, start):
        """Method used to set start attribute """
        self._reset()
        self.start = start
        self.goal = None
        self.closedSet = None
        self.openSet = None
        self.cameFrom = None
        self.gScore = None
        self.fScore = None
        self.path = None

    def set_goal(self, goal):
        """Method used to set goal attribute """
        self._reset()
        self.goal = goal

    def is_open_empty(self):
        """returns True if the open set is empty. False otherwise. """
        if self.openSet is None:
            return True
        return False

    def get_current_node(self):
        """ Returns the node in the open set with the lowest value of f(node)."""
        minimum = float("inf")
        print("GET CURRET NODE _ OPEN SET", self.openSet)
        for val in self.openSet:
            fscore = self.calculate_fscore(val)
            if fscore < minimum:
                new_min_node = val
                minimum = fscore
        return new_min_node

    def get_neighbors(self, node):
        """Returns the neighbors of a node"""
        return self.map.roads[node]

    def get_gScore(self, node):
        """Returns the g Score of a node"""
        self.gScore = self.distance(self.start, node)
        return self.gScore

    def distance(self, node_1, node_2):
        """ Computes the Euclidean L2 Distance"""
        x = self.map.intersections[node_1][0] - self.map.intersections[node_2][0]
        y = self.map.intersections[node_1][1] - self.map.intersections[node_2][1]
        dist = math.sqrt(x ** 2 + y ** 2)
        return dist

    def get_tentative_gScore(self, current, neighbor):
        """Returns the tentative g Score of a node"""
        # plus distance from the current node to it's neighbors
        return self.get_gScore(current) + self.distance(current, neighbor)

    def heuristic_cost_estimate(self, node):
        """ Returns the heuristic cost estimate of a node """
        return self.distance(self.goal, node)

    def calculate_fscore(self, node):
        """Calculate the f score of a node. """
        # REMEMBER F = G + H
        return self.get_gScore(node) + self.heuristic_cost_estimate(node)

    def record_best_path_to(self, current, neighbor):
        """Record the best path to a node """
        self.cameFrom[neighbor] = current
        self.gScore = self.get_tentative_gScore(current, neighbor)
        self.fScore = self.calculate_fscore(neighbor)
        print(self.cameFrom, 'saving came from')
