# search.py
# ---------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

"""
In search.py, you will implement generic search algorithms which are called 
by Pacman agents (in searchAgents.py).
"""

import util
import random
from game import Directions

directions_dict = {"North": Directions.NORTH,
                   "East": Directions.EAST,
                   "South": Directions.SOUTH,
                   "West": Directions.WEST}


class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples,
        (successor, action, stepCost), where 'successor' is a
        successor to the current state, 'action' is the action
        required to get there, and 'stepCost' is the incremental
        cost of expanding to that successor
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.  The sequence must
        be composed of legal moves
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other
    maze, the sequence of moves will be incorrect, so only use this for tinyMaze
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return [s, s, w, s, w, w, s, w]


# Delete multiple values from a list, given values we want to delete, and return new list. lst not modified.
def delete_by_values(lst, values):
    values_as_set = set(values)
    return [x for x in lst if x not in values_as_set]


def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first
    [2nd Edition: p 75, 3rd Edition: p 87]

    Depth-First Search (instance of Graph-Search Algorithm)
    - DFS uses LIFO queue (where most recently generated node is chosen to be expansion),
    which must be deepest unexpanded node (one deeper than its parent)
    - Proceeds to deepest levels nodes in current frontier first
    (where nodes have no successors)
    - Expand deepest level nodes, then remove them from frontier, then
    "back up" the search in reverse to the next deepest node (that still
    has unexplored successors)
    - Graph-Search version avoids repeated states and redundant paths, and is
    complete (since all nodes eventually expanded)

    Your search algorithm needs to return a list of actions that reaches
    the goal.  Make sure to implement a graph search algorithm
    [2nd Edition: Fig. 3.18, 3rd Edition: Fig 3.7].

    function GRAPH-SEARCH(problem) returns a solution, or failure
      initialize the frontier using the initial state of problem
      initialize the explored set to be empty
      loop do
        if the frontier is empty then return failure
        choose a leaf node and remove it from the frontier
        if the node contains a goal state then return corresponding solution
        add the node to explored set
        expand chosen node, adding resulting nodes to frontier
          only if not in frontier or explored set

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    print("Start: " + str(problem.getStartState()))
    print("Is the start a goal? " + str(problem.isGoalState(problem.getStartState())))
    print("Start's successors: " + str(problem.getSuccessors(problem.getStartState())))
    # todo convert sudo code

    # function GRAPH-SEARCH(problem) returns a solution, or failure
    root = problem.getStartState()
    solution = {}
    #   initialize the frontier using the initial state of problem
    frontier = {root: {"d": "root", "i": 0}}

    #   initialize the explored set to be empty
    explored = []

    # loop do
    i = 0
    while True:
        i += 1
        # if the frontier is empty then return failure
        if not len(frontier):
            result = "failure"
            break
        # choose a leaf node and remove it from the frontier
        node = max(frontier.keys(), key=lambda k: frontier[k]["i"])
        node_data = frontier.pop(node)

        # Maintain solution dict after backing up
        if len(solution):  # Must have one node at least
            for sol in sorted(solution.keys(), key=lambda k: solution[k]["i"], reverse=True):
                # pull successors of solution node: loc, dir, cost, store (l, d) list
                sol_successors = [(l, d) for l, d, c in problem.getSuccessors(sol)]
                if (node, node_data["d"]) not in sol_successors:
                    solution.pop(sol)
                else:
                    break

        # add current node to solution
        solution[node] = node_data

        # if the node contains a goal state then return corresponding solution
        if problem.isGoalState(node):
            # convert solution to the correct output
            # ensure in proper order
            order = sorted(solution.keys(), key=lambda k: solution[k]["i"])
            result = [directions_dict[solution[sol]["d"]] for sol in order if solution[sol]["i"] > 0]
            break
        # add the node to explored set
        explored.append(node)
        # expand chosen node, adding resulting nodes to frontier
        node_successors = problem.getSuccessors(node)
        for loc, direction, dist in node_successors:
            # only if not in frontier or explored set
            if loc not in frontier.keys() and loc not in explored:
                frontier[loc] = {"d": direction, "i": i}

    return result

    # todo: if not set(getSuccessors(node)).issubset(frontier):
    # todo:     solution.pop()
    # todo for each in solution sorted reversed:
    # todo  if current node is not in list of successors, pop
    # todo: what happens if there are multiple ways to get to a route?
    # todo they must have entered that state from the same direction...
    # todo store loc and direction

    # todo use set to determine if at a node different from the last added
    # todo remove the top from the solution list
    # todo add direction of path it took to get to current node
    # todo determine if backing up... remove back to that node of directions
    # todo: solution needs to wind up in a list of list of lists...
    # todo every node is going to be the root of a large branch of nodes
    # todo when it backs up to that node it needs to know


    # todo add node to solution, how can this be a shared variable and be maintained properly?

    # todo need to use a graph implementation
    # todo build the graph as it searches
    # todo clean the graph when reaching dead ends
    # todo retrieve the solution from the graph
    # todo build the solution as it searches
    # todo when it reaches a point where there are no more unexplored nodes, back up by removing last move
    # todo from solution
    # todo back up and remove node from list if it wasn't newly explored (or goal state)
    # todo anytime there are additional
    # todo it can be in the frontier, but we don't pull from that until we back up to where a node was adjacent to it to
    # todo: maintain the solution


def breadthFirstSearch(problem):
    """
    Search the shallowest nodes in the search tree first.
    [2nd Edition: p 73, 3rd Edition: p 82]

    Breadth-First Search (instance of Graph-Search Algorithm)
    - BFS uses FIFO queue
    """

    # Note: Try using Queue or PriorityQueue data structure since DFS uses FIFO

    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()


def uniformCostSearch(problem):
    "Search the node of least total cost first. "
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
    "Search the node that has the lowest combined cost and heuristic first."
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
