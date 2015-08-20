import math
import heapq

# each state represents a distinct position of robot in the C-space
class State:

    def __init__(self, x, y, goalX, goalY):
	    self.X = x
	    self.Y = y
	    self.goalX = goalX
	    self.goalY = goalY
	    
    def manhattan(self):
	    return math.fabs(self.X-self.goalX) + math.fabs(self.Y-self.goalY)

    def euclidean(self):
		return math.sqrt(math.pow((self.X-self.goalX),2) + math.pow((self.Y-self.goalY),2))

    def isGoal(self):
	    return self.manhattan() == 0

	# generate successor states
    def successors(self, n_connected, steplenX, steplenY):
	    yield State(self.X+steplenX, self.Y, self.goalX, self.goalY)
	    yield State(self.X-steplenX, self.Y, self.goalX, self.goalY)
	    yield State(self.X, self.Y+steplenY, self.goalX, self.goalY)
	    yield State(self.X, self.Y-steplenY, self.goalX, self.goalY)

	    if n_connected == 8:
	    	yield State(self.X+steplenX, self.Y+steplenY, self.goalX, self.goalY)
	    	yield State(self.X+steplenX, self.Y-steplenY, self.goalX, self.goalY)
	    	yield State(self.X-steplenX, self.Y-steplenY, self.goalX, self.goalY)
	    	yield State(self.X-steplenX, self.Y+steplenY, self.goalX, self.goalY)

	# State objects are equal if they have similar X and Y
    def __eq__(self, other):
		return self.X == other.X and self.Y == other.Y


# search node for path finding
class SearchNode:

	def __init__(self, initial_state):
		self.state = initial_state
		self.cost = 0
		self.prev = None

	def __cmp__(self, other):
		return cmp(self.priority(), other.priority())

	# defines f(n) = h(n)+g(n) for each search node
	def priority(self):

		# if heuristic == "manhattan":
		# return self.state.manhattan()+self.cost
		return self.state.euclidean()+self.cost
       	# else:
       	# 	return self.state.euclidean()+self.cost

	def __eq__(self, other):
		return self.state == other.state



class Solver:

	def __init__(self, initial_state):

		self.initial = initial_state

		self.path = [] # a stack of states representing the optimal path

		self.closedSet = [] # closed set holding states already expanded
		self.collisionSet = [] # collision states

	def search(self):

		searchNode = SearchNode(self.initial) # initialize start state

		searchTree = [] # open set to hold nodes to be expanded

		heapq.heappush(searchTree, searchNode) # put initial node into open set

		# keep searching until open set is empty
		while len(searchTree) != 0: 
		
			searchNode = heapq.heappop(searchTree) # extract from open set the node with min f(n)

			# if node is goal node, stop searching and construct path
			if searchNode.state.isGoal() == True:
				self.constructPath(searchNode)
				return 1

			# if node is not goal, move from open set to closed set
			if searchNode.state not in self.closedSet:
				self.closedSet.append(searchNode.state)
				# print "\nclosed set: " + str(searchNode.state.X) + str(searchNode.state.Y)

			# generate successor states by specified connecting method and steplengths
			successors = searchNode.state.successors(n_connected = 8, steplenX = 1, steplenY = 1)

			for _ in successors:

				nextState = _

                # if successor has been explored, discard it
				if nextState in self.closedSet:
					continue

				# if similar successor has been put in open set, do not put again
				# substantially reduces running time
				testNode = SearchNode(nextState)
				if testNode in searchTree:
					continue

				# if colliding with boundary of environment, discard current successor
				if nextState.X < -34 or nextState.Y < -14 or nextState.X > 28 or nextState.Y > 14:
					if nextState not in self.collisionSet:
						self.collisionSet.append(nextState)
					continue

				# if colliding with wall, discard current successor
				if nextState.X > 3 and nextState.X < 13 and nextState.Y != 11:
					if nextState not in self.collisionSet:
						self.collisionSet.append(nextState)
					continue

				# if colliding with tables, discard current successor
				if nextState.X > -33 and nextState.X < -2 and nextState.Y > -10 and nextState.Y < 10:
					if nextState not in self.collisionSet:
						self.collisionSet.append(nextState)
					continue

				# put successor into open set
				newNode = SearchNode(nextState)
				newNode.cost = searchNode.cost + math.sqrt(math.pow((nextState.X-searchNode.state.X),2) 
					                                      +math.pow((nextState.Y-searchNode.state.Y),2))
				newNode.prev = searchNode

				# print str(newNode.state.X) + str(newNode.state.Y) + " is put into open set."
				# print "successor's evaluation: " + str(newNode.priority())
				
				heapq.heappush(searchTree, newNode)

		# if open set is empty, not solvable
		print "No Solution Found."
		return 0

	# construct path by backtracking the previous states of successful searchNode and store them into list
	def constructPath(self, searchNode):

		self.path.append(searchNode.state)
		searchStep = searchNode.prev
		while searchStep is not None:
			self.path.append(searchStep.state)
			# print str(searchNode.prev.state.X) + str(searchNode.prev.state.Y) + " is put into path."
			searchStep = searchStep.prev
		
	# get path by extracting states from list in a stack manner
	def getPath(self):
		output = []
		for i in range(len(self.path)):
			wayPoint = self.path.pop()
			output.append((wayPoint.X*0.1, wayPoint.Y*0.1, 0.03))
		return output

	def getSet(self, Set):
		output = []
		for state in Set:
			output.append((state.X*0.1, state.Y*0.1, 0.02))
		return output

