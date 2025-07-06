import heapq
import math

class Dijkstra:
    
    def __init__(self, grmap):
        self._grmap = grmap
        self._open = []
        self._closed = {}
        self._closed_gval = {}

    def run(self, start, goal):
        self._closed.clear()
        self._open.clear()
        self._closed_gval.clear()

        self._closed[start.state_hash()] = start
        self._closed_gval[start.state_hash()] = 0.0
        heapq.heappush(self._open, start)

        while len(self._open) > 0:
            n = heapq.heappop(self._open)

            if n == goal:
                path = []
                exp_dik = []
                for value in self._closed.values():
                    xval = value.get_x()
                    yval = value.get_y()
                    exp_dik.append((xval, yval))

                goal_hash = goal.state_hash()
                next_node = self._closed[goal_hash]
                tot_cost = next_node.get_cost()
                while next_node is not None:
                    path.append(next_node)
                    next_node = next_node.get_parent()
                path = path[::-1]
                return path, tot_cost, len(exp_dik)
            
            children = self._grmap.successors(n)
            for child in children:
                child_hash = child.state_hash()

                child.set_cost(child.get_g())
                child.set_parent(n)

                if child_hash not in self._closed:
                    self._closed[child.state_hash()] = child
                    heapq.heappush(self._open, child)
                    self._closed_gval[child.state_hash()] = child.get_cost()
 
                if child_hash in self._closed:
                    cl_gval = self._closed_gval[child.state_hash()]
                    if child.get_cost() < cl_gval:
                        self._closed_gval[child.state_hash()] = child.get_cost()
                        child.set_parent(n)
                        heapq.heapify(self._open)       
                    
        return None, -1, None
        
    def get_closed_data(self):
        return self._closed
        
        
class AStar:
    
    def __init__(self, grmap):
        self._grmap = grmap
        self._open = []
        self._closed = {}
        self._closed_gval = {}

    def get_h(self, goal, node):
        delx = abs(goal.get_x() - node.get_x())
        dely = abs(goal.get_y() - node.get_y())
        minv = 1.5*min(delx, dely)
        absv = abs(delx - dely)

        return (minv+absv)  

    def run(self, start, goal):
        
        self._closed.clear()
        self._open.clear()
        self._closed_gval.clear()

        self._closed[start.state_hash()] = start
        self._closed_gval[start.state_hash()] = 0.0

        hval = self.get_h(goal, start)
        start.set_cost(start.get_g() + hval)
        heapq.heappush(self._open, start)

        while len(self._open) > 0:
            n = heapq.heappop(self._open)

            if n == goal:
                path = []
                exp_astar = []
                for value in self._closed.values():
                    xval = value.get_x()
                    yval = value.get_y()
                    exp_astar.append((xval, yval))

                goal_hash = goal.state_hash()
                next_node = self._closed[goal_hash]
                tot_cost = next_node.get_cost()
                while next_node is not None:
                    path.append(next_node)
                    next_node = next_node.get_parent()
                path = path[::-1]
                return path, tot_cost, len(exp_astar)
            
            children = self._grmap.successors(n)
            for child in children:
                child_hash = child.state_hash()
                hval = self.get_h(goal, child)
                child.set_cost(child.get_g() + hval)
                child.set_parent(n)

                if child_hash not in self._closed:
                    self._closed[child.state_hash()] = child
                    heapq.heappush(self._open, child)
                    self._closed_gval[child.state_hash()] = child.get_cost()
 
                if child_hash in self._closed:
                    cl_gval = self._closed_gval[child.state_hash()]
                    if child.get_cost() < cl_gval:
                        self._closed_gval[child.state_hash()] = child.get_cost()
                        child.set_parent(n)
                        heapq.heapify(self._open)       
                    
        return None, -1, None

    def get_closed_data(self):
        return self._closed    
        
class State:
    """
    Class to represent a state on grid-based pathfinding problems. The class contains two static variables:
    map_width and map_height containing the width and height of the map. Although these variables are properties
    of the map and not of the state, they are used to compute the hash value of the state, which is used
    in the CLOSED list. 

    Each state has the values of x, y, g, h, and cost. The cost is used as the criterion for sorting the nodes
    in the OPEN list for both Dijkstra's algorithm and A*. For Dijkstra the cost should be the g-value, while
    for A* the cost should be the f-value of the node. 
    """
    map_width = 0
    map_height = 0
    
    def __init__(self, x, y):
        """
        Constructor - requires the values of x and y of the state. All the other variables are
        initialized with the value of 0.
        """
        self._x = x
        self._y = y
        self._g = 0
        self._cost = 0
        self._parent = None
        
    def __repr__(self):
        """
        This method is invoked when we call a print instruction with a state. It will print [x, y],
        where x and y are the coordinates of the state on the map. 
        """
        state_str = "[" + str(self._x) + ", " + str(self._y) + "]"
        return state_str
    
    def __lt__(self, other):
        """
        Less-than operator; used to sort the nodes in the OPEN list
        """
        return self._cost < other._cost
    
    def state_hash(self):
        """
        Given a state (x, y), this method returns the value of x * map_width + y. This is a perfect 
        hash function for the problem (i.e., no two states will have the same hash value). This function
        is used to implement the CLOSED list of the algorithms. 
        """
        return self._y * State.map_width + self._x
    
    def __eq__(self, other):
        """
        Method that is invoked if we use the operator == for states. It returns True if self and other
        represent the same state; it returns False otherwise. 
        """
        return self._x == other._x and self._y == other._y

    def get_x(self):
        """
        Returns the x coordinate of the state
        """
        return self._x
    
    def set_parent(self, parent):
        """
        Sets the parent of a node in the search tree
        """
        if parent is None:
            print("None")
        self._parent = parent

    def get_parent(self):
        """
        Returns the parent of a node in the search tree
        """
        return self._parent
    
    def get_y(self):
        """
        Returns the y coordinate of the state
        """
        return self._y
    
    def get_g(self):
        """
        Returns the g-value of the state
        """
        return self._g
        
    def set_g(self, g):
        """
        Sets the g-value of the state
        """
        self._g = g

    def get_cost(self):
        """
        Returns the cost of a state; the cost is determined by the search algorithm
        """
        return self._cost
    
    def set_cost(self, cost):
        """
        Sets the cost of the state; the cost is determined by the search algorithm 
        """
        self._cost = cost
        