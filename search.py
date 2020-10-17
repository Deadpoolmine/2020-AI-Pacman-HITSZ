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

from os import curdir
from tkinter.constants import TRUE
from game import Directions
import time
import util
from util import PriorityQueue, Queue, Stack
from enum import Enum

class SearchType(Enum):
    DFS = 0
    BFS = 1
    AStar = 2
    UCS = 3


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

def mDfs(problem, statue, openTable, path, closedTable):
    if(problem.isGoalState(statue)):
        while not openTable.isEmpty():
            successor = openTable.pop()
            path.insert(0, successor[1])
        return
    else:
        successors = problem.getSuccessors(statue)
        if successors: 
            for successor in successors:
                successorStatue = successor[0]
                if successorStatue not in closedTable:
                    #递归前
                    openTable.push(successor)
                    closedTable.append(successorStatue)
                    mDfs(problem, successorStatue,openTable,path, closedTable)
                    #递归后
                    if not openTable.isEmpty(): 
                        closedTable.pop()
                        openTable.pop()
        else:
            return



def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"
    # 0.跟随PPT伪代码
    # openTable = Stack()
    # path = []
    # closedTable = []
    # initStatue = problem.getStartState()
    # currentStatue = ((initStatue, '', 0), []) #(currentStatue, path)
    # openTable.push(currentStatue)
    # while not problem.isGoalState(currentStatue[0][0]):
    #     if openTable.isEmpty():
    #         break
    #     else:
    #         currentStatue = openTable.pop()
    #         closedTable.append(currentStatue[0][0])
    #         successors = problem.getSuccessors(currentStatue[0][0])
    #         successors.reverse()
    #         if successors:
    #             for successor in successors:
    #                 successorStatue = successor[0]
    #                 successorDir = successor[1]
    #                 if successorStatue not in closedTable:
    #                     openTable.push((successor, currentStatue[1] + [successorDir]))
    
    # path = currentStatue[1]
    # 1.递归实现方式
    # mDfs(problem, initStatue, openTable, path, closedTable)
    # print(path)
    # 2.另一种实现方式
    # currentPopSuccesor = None
    # openTable.push(currentStatue)
    # while not problem.isGoalState(currentStatue):
    #     succesors = problem.getSuccessors(currentStatue)
    #     hasValidSuccesor = False
    #     for succesor in succesors:
    #         succesorStatue = succesor[0]
    #         #可拓展
    #         if succesorStatue not in closedTable:
    #             #因为后面被Pop出来了，我们需要加回去
    #             if currentPopSuccesor != None:
    #                 openTable.push(currentPopSuccesor)
    #                 #清除记录
    #                 currentPopSuccesor = None
    #             hasValidSuccesor = True
    #             #加入closed表
    #             closedTable.append(succesorStatue)
    #             #更新当前状态
    #             currentStatue = succesorStatue
    #             #加入open表
    #             openTable.push(succesor)
    #             break
    #     #没有后路了，回溯
    #     if not hasValidSuccesor:
    #         #记录一下回溯的结点    
    #         currentPopSuccesor = openTable.pop()
    #         #更新当前状态
    #         currentStatue = currentPopSuccesor[0]

    # #输出结果    
    # while not openTable.isEmpty():
    #     successor = openTable.pop()
    #     path.insert(0, successor[1])
    path = normalSearch(problem, SearchType.DFS, nullHeuristic)
    return path



def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    path = normalSearch(problem, SearchType.BFS, nullHeuristic)
    return path

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    path = normalSearch(problem, SearchType.UCS, nullHeuristic)
    return path

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    path = normalSearch(problem, SearchType.AStar, heuristic)
    return path

class State():
    """
    docstring
    """
    customState = None  #(x, y); ((x,y), []);
    fromDir = None      #来自哪一个方向
    cost = 0            #cost
    path = []           #path
    def __init__(self, customState, path, cost, fromDir):
        self.customState = customState
        self.path = path
        self.cost = cost
        self.fromDir = fromDir
'''
参数说明：
problem:传入problem即可
searchType: 传入SearchType.xxx即可
heuristic:估价函数 heuristic(statue, problem)
'''
def normalSearch(problem, searchType, heuristic=nullHeuristic):
    path = []
    openTable = None
    closedTable = []
    #初始化
    if searchType == SearchType.BFS:
        openTable = Queue()
    elif searchType == SearchType.DFS:
        openTable = Stack()
    else:
        openTable = PriorityQueue()
    initStatue = problem.getStartState()       #customState
    currentStatue = State(cost=0, path=[], fromDir="",customState=initStatue)
    if searchType == SearchType.BFS or searchType == SearchType.DFS:
        openTable.push(currentStatue)
        while not problem.isGoalState(currentStatue.customState):
            if openTable.isEmpty():
                break
            else:
                currentStatue = openTable.pop()
                if currentStatue.customState in closedTable:
                    continue
                if problem.isGoalState(currentStatue.customState):
                    break
                closedTable.append(currentStatue.customState)
                successors = problem.getSuccessors(currentStatue.customState)
                if successors:
                    for successorStatue, successorDir, successorCost in successors:
                        if successorStatue not in closedTable:
                            successorStatue = State(path=currentStatue.path+[successorDir], 
                            customState=successorStatue, cost=0, fromDir=successorDir)
                            openTable.push(successorStatue)
        path = currentStatue.path
    else:
        if searchType == SearchType.AStar:
            currentStatue.cost = 0 + heuristic(initStatue, problem)
        elif searchType == SearchType.UCS:
            currentStatue.cost = currentStatue.cost
        openTable.push(currentStatue, currentStatue.cost)
        while not problem.isGoalState(currentStatue.customState):
            if openTable.isEmpty():
                break
            else:
                currentStatue = openTable.pop()
                if currentStatue.customState in closedTable:
                    continue
                if problem.isGoalState(currentStatue.customState):
                    break
                closedTable.append(currentStatue.customState)
                successors = problem.getSuccessors(currentStatue.customState)
                for successorStatue,successorDir,successorCost in successors:
                    if searchType == searchType.AStar:
                        successorCost = problem.getCostOfActions(currentStatue.path) + successorCost + heuristic(successorStatue,problem)
                    elif searchType == searchType.UCS:
                        successorCost = problem.getCostOfActions(currentStatue.path) + successorCost
                    if successorStatue not in closedTable:
                        successorStatue = State(path=currentStatue.path+[successorDir], 
                            customState=successorStatue, cost=successorCost, fromDir=successorDir)
                        openTable.push(successorStatue, successorCost)
        path = currentStatue.path
    return path
    

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
