#!/usr/bin/python3


from CS312Graph import *
import time
import math

class ArrayQueue:

    def __init__(self, networkNodes, dist):
        self.networkNodes = networkNodes
        self.dist = dist
        pass

    # Make a list of items where the key is
    # the node in string form and the value is the
    # minimum distance to the node.
    #
    # INPUT: srcIndex - the index of the src node
    # RETURN: H - a priority queue using an array
    #             implementation
    def makeQueue(self):
        H = [None] * len(self.networkNodes)
        for node in self.networkNodes:
            key = str(node)
            value = self.dist[node.node_id]
            H[node.node_id] = (key, value)
        return H

    # Remove the node with the smallest distance value
    # from H and return it for use
    #
    # INPUT: H - a priority queue using an array
    #            implementation
    # RETURN: the node with the smallest distance; H
    def deleteMin(self, H):
        minIndex = 0

        for i in range(len(H)):
            tempDist = H[i][1]
            if tempDist < H[minIndex][1] and tempDist != -1:
                minIndex = i

        key = H[minIndex][0]
        value = -1
        H[minIndex] = (key,value)
        return self.networkNodes[minIndex], H

    # We need to update our arrayQueue's distance value
    # if we find a shorter distance to that node
    #
    # INPUT: H - the array priority queue
    #        neighborIndex - the location of the node distance to update
    #        newDistance - the new distance, duh.
    # RETURN: H - the array priority queue
    def decreaseKey(self, H, neighborIndex, newDistance):
        key = H[neighborIndex][0]
        newValue = newDistance
        H[neighborIndex] = (key,newValue)
        return H

class NetworkRoutingSolver:
    def __init__(self):
        self.infinity = float('inf')
        pass

    def initializeNetwork( self, network ):
        assert( type(network) == CS312Graph )
        self.network = network

    def getShortestPath( self, destIndex ):
        self.dest = destIndex
        # TODO: RETURN THE SHORTEST PATH FOR destIndex
        #       INSTEAD OF THE DUMMY SET OF EDGES BELOW
        #       IT'S JUST AN EXAMPLE OF THE FORMAT YOU'LL 
        #       NEED TO USE
        path_edges = []
        total_length = 0
        startNode = self.network.nodes[self.source]
        minPathLength = min(self.dist[1:])
        minPathIndex = 0
        for item in self.dist:
            if item == minPathLength:
                break
            minPathIndex += 1

        destNode = self.network.nodes[minPathIndex]
        # destNode.src = the next item in the list that pointed to it

        foundSrc = False
        maxSearches = len(self.network.nodes)
        while maxSearches > 0:

            prevIndex = self.prev[minPathIndex]
            prevEdge = None
            prevNode = None
            for item in destNode.neighbors:
                if item.dest.node_id == prevIndex:
                    prevEdge = item
                    prevNode = item.dest

            total_length += prevEdge.length
            path_edges.append((prevEdge.src.loc, prevEdge.dest.loc, '{:.0f}'.format(prevEdge.length)))

            if prevNode == startNode:
                foundSrc = True
                break

            maxSearches += 1
            destNode = prevNode
            minPathIndex = prevNode.node_index


        if not foundSrc:
            return {'cost': "unreachable", 'path': []}
        else:
            return {'cost':total_length, 'path':path_edges}

        # edges_left = 3
        # while edges_left > 0:
        #     edge = node.neighbors[2]
        #     path_edges.append( (edge.src.loc, edge.dest.loc, '{:.0f}'.format(edge.length)) )
        #     total_length += edge.length
        #     node = edge.dest
        #     edges_left -= 1
        # return {'cost':total_length, 'path':path_edges}

    def computeShortestPaths( self, srcIndex, use_heap=False ):
        self.source = srcIndex
        t1 = time.time()

        if use_heap: # use heapQueue
            print("Implement the heapQueue later.")
        else: # use arrayQueue

            # Initialize all node distances to infinity.
            # Set src node distance to zero
            self.dist = [self.infinity] * len(self.network.nodes)
            self.prev = [None] * len(self.network.nodes)
            self.dist[srcIndex] = 0

            arrayQueue = ArrayQueue(self.network.nodes, self.dist)
            H = arrayQueue.makeQueue()
            minsDeleted = 0
            while minsDeleted < len(H): # While H is not empty
                u, H = arrayQueue.deleteMin(H)
                minsDeleted += 1
                for neighborEdge in u.neighbors:
                    neighborNode = neighborEdge.dest;
                    newDistance = self.dist[u.node_id] + neighborEdge.length
                    if self.dist[neighborNode.node_id] > newDistance:
                        self.dist[neighborNode.node_id] = newDistance
                        self.prev[neighborNode.node_id] = u.node_id
                        H = arrayQueue.decreaseKey(H, neighborNode.node_id, newDistance)


        # Run dikstras, init the arrays, done.

        # TODO: RUN DIJKSTRA'S TO DETERMINE SHORTEST PATHS.
        #       ALSO, STORE THE RESULTS FOR THE SUBSEQUENT
        #       CALL TO getShortestPath(dest_index)
        t2 = time.time()
        return (t2-t1)

    # This function just uses the pythagorean theorem
    # to find the distance between two nodes
    # INPUT: two nodes for which to find the distance
    # RETURN: the euclidean distance between two nodes
    def getDistance(self, node1, node2):
        a = abs(node1.loc.x() - node2.loc.x())
        b = abs(node1.loc.y() - node2.loc.y())

        # euclideanDistance is the c part of the
        # pythagorean theorem
        euclideanDistance = math.sqrt((a**2) + (b**2))

        return euclideanDistance

