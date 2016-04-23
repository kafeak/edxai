# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]


def genericSearch(problem, valueAlgorithm):
    """
    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """

    def pathifyQueue(path):
        steps = []
        while path.list:
            node = path.pop()
            if node[1] != "start":
                steps.append(node[1])
        return steps

    def getSortedSuccessors(state):
        """ Tie breaking here if any """
        # provide as is
        sux = problem.getSuccessors(state)
        # native python sorting by state
        # sux = sorted(problem.getSuccessors(state), key=lambda node: node[0])
        return sux

    print_my_debug_messages = 0
    goal = False  # will be transformed into appropriate path
    # MANUALLY EXPAND START STATE AND CREATE VISITED STATES LIST
    start_state = problem.getStartState()
    successors = util.Stack()
    for node in getSortedSuccessors(start_state):
        successors.push(node)
    visited = [start_state]
    if print_my_debug_messages >= 1:
        print start_state, " expanded: ", successors.list

    # CREATE FRINGE FROM START STATE EXPANSION
    fringe = util.PriorityQueue()
    while successors.list:
        fresh_node = successors.pop()
        new_path = util.Queue()
        # new_path.push(start_state)
        new_path.push(fresh_node)
        if print_my_debug_messages >= 1:
            print "Pushing ", new_path.list, " to fringe with Priority Queue value of ", valueAlgorithm(new_path)
        """
        if print_my_debug_messages >= 0:
            print "Goal test: ", fresh_node[0], " ? ", problem.isGoalState(fresh_node[0])
        if problem.isGoalState(fresh_node[0]):
            goal = new_path
            break
        """
        if print_my_debug_messages >= 1:
            print "Pushing ", new_path.list, " to fringe with Priority Queue value of ", valueAlgorithm(new_path)
        fringe.push(new_path, valueAlgorithm(new_path))
    """
    if goal:
        path_to_goal = pathifyQueue(goal)
        if print_my_debug_messages >= 0:
            print "Path found: ", path_to_goal
        return path_to_goal
    """
    # EXPAND FROM FRINGE UNTIL GOAL HAS BEEN FOUND
    while 1:
        if print_my_debug_messages >= 1:
            print "==FRINGE size",len(fringe.heap)
            for value, number, path in fringe.heap:
                print [[x[0] for x in reversed(path.list)], value, number]
        # Only expand non-visited states.
        # Test for goal before expansion.
        expandee_found = False
        while not expandee_found:
            expandee = fringe.pop()
            expandee_state = expandee.list[0][0]
            if print_my_debug_messages >= 1:
                print "Goal test: ", expandee_state, " ? ", problem.isGoalState(expandee_state)
            if problem.isGoalState(expandee_state):
                goal = expandee
                break
            if expandee_state in visited:
                if print_my_debug_messages >= 1:
                    print "Expanded state ", expandee_state, " has already been visited (", visited, ")"
                continue
            expandee_found = True
        # break again, goal found.
        if goal:
            break
        if print_my_debug_messages >= 1:
            print "<-FRINGE", expandee_state, " from ", [i for i in reversed(expandee.list)]
        expanded_path = []
        while expandee.list:
            expanded_path.append(expandee.pop())
        expanded_state = expanded_path[-1][0]
        if print_my_debug_messages >= 1:
            print "From path", [x[0] for x in expanded_path], "selecting state ", expanded_state, " for expansion"
        visited.append(expanded_state)
        successors = util.Stack()
        for node in getSortedSuccessors(expanded_state):
            successors.push(node)
        if print_my_debug_messages >= 1:
            print expanded_path, " expanded: ", successors.list
        while successors.list:
            fresh_node = successors.pop()
            new_path = util.Queue()
            for node in expanded_path:
                new_path.push(node)
            new_path.push(fresh_node)
            pqvalue = valueAlgorithm(new_path)
            if print_my_debug_messages >= 1:
                print "++FRINGE",[i for i in reversed(new_path.list)],"pqvalue",pqvalue
            fringe.push(new_path, pqvalue)

        if goal:
            break

    path_to_goal = pathifyQueue(goal)
    if print_my_debug_messages >= 0:
        print "Path found:", path_to_goal
    return path_to_goal


def depthFirstSearch(problem):
    return genericSearch(problem,
                         valueAlgorithm=lambda path: -len(path.list))


def breadthFirstSearch(problem):
    return genericSearch(problem, valueAlgorithm=lambda path: len(path.list))


def uniformCostSearch(problem):
    def cumulativeStepCost(path):
        cost = 0
        for node in path.list:
            cost += node[2]
        return cost
    return genericSearch(problem, valueAlgorithm=cumulativeStepCost)


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    def fCost(path):
        """
        logi = 0
        logstr = "  A*fCost path: "
        for s,a,f in reversed(path.list):
            logi += 1
            logstr += s
            if logi != len(path.list):
                logstr += " -> "
        print logstr
        """
        cost = 0
        for (st, act, fcost) in reversed(path.list):
            cost += fcost
            #print "  A*fCost step cost",st,":",fcost,"(+=",cost,")"
        current_state = path.list[0][0]
        cost += heuristic(current_state, problem)
        #print "  A*fCost heuristic",current_state,":",heuristic(current_state, problem),"+=",cost,"total"
        return cost
    return genericSearch(problem, valueAlgorithm=fCost)


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
