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

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"
    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    start = problem.getStartState()
    if problem.isGoalState(start):
        return []

    visited = {start:True}
    #(r, acs) = dfsSolver(problem, start, visited)
    (r, acs) = dfsSolverStack(problem, start, visited)
    if r:
        acs.reverse()
        print("solution:", acs)
        return acs

    print("no solutions")
    return []

def dfsSolver(problem,state,visited):
    if problem.isGoalState(state):
        return (True,[])

    visited[state] = True
    successors = problem.getSuccessors(state)
    successors.reverse()
    for s in successors:
        if (s[0] in visited and visited[s[0]]) :
            continue

        (r,acs) = dfsSolver(problem,s[0],visited)
        if r:
            acs.append(s[1])
            return (True,acs)
    return (False,[])


def dfsSolverStack(problem,state,ignore):
    visited={}
    stack = util.Stack()
    stack.push((state,'',-1))
    visited[state] = True
    while not stack.isEmpty():
        (s,a,idx) = stack.pop()
        if problem.isGoalState(s):
            r=[a]
            while not stack.isEmpty():
                t = stack.pop()
                if t[1] != '':
                    r.append(t[1])
            return (True,r)

        # find the next child which is not visited
        successors = problem.getSuccessors(s)
        successors.reverse()
        next = idx + 1
        ss = successors[next] if next < len(successors) else None
        while ss and ss[0] in visited and visited[ss[0]]:
            next+=1
            ss = successors[next] if next < len(successors) else None

        if not ss:
            continue

        stack.push((s,a,next))
        visited[ss[0]] = True
        stack.push((ss[0],ss[1],-1))

    return (False,[])

def dfsSolverStackMin(problem,state,ignore):
    visited={}
    stack = util.Stack()
    stack.push((state,'',0,-1))
    visited[str(state)] = True
    best = (False,[],-1)
    while not stack.isEmpty():
        (s,a,w,idx) = stack.pop()
        if problem.isGoalState(s):
            if a!='':
                r=[a]
            else:
                r=[]
            tot = w
            l = [x for x in stack.list]
            l.reverse()
            for e in l:
                if e[1]!='':
                    r.append(e[1])
                    tot+=e[2]
            if  best[2] <0 or tot < best[2]:
                best = (True,r,tot)
            continue

        # find the next child which is not visited
        successors = problem.getSuccessors(s)
        successors.reverse()
        next = idx + 1
        ss = successors[next] if next < len(successors) else None
        while ss and str(ss[0]) in visited and visited[str(ss[0])]:
            next+=1
            ss = successors[next] if next < len(successors) else None

        if not ss:
            continue

        stack.push((s,a,w,next))
        visited[str(ss[0])] = True
        stack.push((ss[0],ss[1],ss[2],-1))

    return best

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    start = problem.getStartState()
    print("bfs",start   )
    p = bfsSolver(start,problem)
    print(p)
    return p

def bfsSolver(state,problem):
    visited ={str(state):True}
    q = util.Queue()
    q.push((state,[]))
    while not q.isEmpty():
        r = q.pop()
        if problem.isGoalState(r[0]):
            return r[1]
        else:
            successors = problem.getSuccessors(r[0])
            for s in successors:
                if str(s[0]) not in visited:
                    visited[str(s[0])]=True
                    r1=[x for x in r[1]]
                    r1.append(s[1])
                    t = (s[0],r1)
                    q.push(t)

    return []


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    start = problem.getStartState()
    if problem.isGoalState(start):
        return []

    visited = {}
    cached = {}
    #(r, acs) = dfsSolver(problem, start, visited)
    print("uniformCostSearch: start solving")
    (r, acs,w) = dfsSolverMin(problem, start, visited,cached,0)
    #(r, acs, w) = dfsSolverStackMin(problem, start, visited)
    if r:
        acs.reverse()
        print("solution:", w,acs)
        return acs

    print("no solutions")
    return []

def dfsSolverMin(problem,state,visited,cached,cost):
    if problem.isGoalState(state):
        return (True,[],cost)

    if str(state) in cached and cached[str(state)][2] > 0 and cost > cached[str(state)][2]:
        return cached[str(state)]

    visited[str(state)] = 1

    successors = problem.getSuccessors(state)
    best = (False,[],-1)

    for s in successors:
        if (str(s[0]) in visited and visited[str(s[0])] >0) :
            continue

        (r,acs,w) = dfsSolverMin(problem,s[0],visited,cached,cost + s[2])
        if r:
            if best[2]<0 or best[2] > w:
                acs1=[x for x in acs]
                acs1.append(s[1])
                best = (True,acs1,w)

    visited[str(state)] = 0
    cached[str(state)] = (best[0],[x for x in best[1]],best[2])
    return best

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""

    start = problem.getStartState()
    if problem.isGoalState(start):
        return ['Stop']

    visited = {}
    cached ={}

    print("aStarSearch: start solving",start)
    (r, acs,w) = dfsSolverAstar(problem, start, visited,cached,heuristic,0)
    if r:
        acs.reverse()
        print("solution:", w,acs)
        return acs

    print("no solutions")
    return []

def dfsSolverAstar(problem,state,visited,cached,heuristic,cost):
    if (str(state) in visited and visited[str(state)] > 0):
        return (False, [], -1)

    if problem.isGoalState(state):
        return (True,[],cost)



    if str(state) in cached and cached[str(state)][2] > 0 and cost > cached[str(state)][2]:
        print(cached[str(state)][2])
        return cached[str(state)]

    visited[str(state)] = 1

    q = util.PriorityQueue()
    successors = problem.getSuccessors(state)
    for s in successors:
        h = heuristic(s[0],problem)
        q.update((s,h),h)

    best = (False,[],-1)
    last = None
    while not q.isEmpty():
        s,h = q.pop()

        if last is not None and h > last and best[0]:
            break

        if (str(s[0]) in visited and visited[str(s[0])] > 0):
            continue

        last = h

        (r,acs,w) = dfsSolverAstar(problem,s[0],visited,cached,heuristic,cost + s[2])
        if r:
            if best[2]<0 or best[2] > w:
                acs1=[x for x in acs]
                acs1.append(s[1])
                best = (True,acs1,w)

    cached[str(state)] = (best[0],[x for x in best[1]],best[2])
    return best

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
