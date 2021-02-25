#!/usr/bin/python3


from CS312Graph import *
import time
import math

class HeapQueue:

    def __init__(self, networkNodes, dist):
        self.networkNodes = networkNodes
        self.dist = dist
        self.map = {}
        pass

    # Make a list of items where the key is
    # the minimum distance to the node and
    # the value is the node's id
    #
    # INPUT: None
    # RETURN: H - a priority queue using an array
    #             implementation
    def makeHeap(self):
        H = None * len(self.networkNodes)
        for node in self.networkNodes:
            self.map[node.node_id] = node.node_id
            key = self.dist[node.node_id]
            value = node.node_id
            H[node.node_id] = (key,value)
        return H

    # Returns the tuple with the smallest distance to it
    # from the minHeap and then corrects the min heap.
    #
    # INPUT: H - The min heap
    # RETURN: The tuple with the smallest distance to it
    def deleteMin(self, H):
        # Swap the first and last element in the min heap
        temp = H[0]
        H[0] = H[-1]
        H[-1] = temp

        # Pop off the min element
        returnMe = H.pop(len(H) - 1)
        self.map[returnMe[1]] = None # [1] refers to the node_id. Say the returned
                                     # node no longer exists in the min heap.
        self.map[H[0][1]] = 0 # Set the new root node's heap position to 0

        # Correct the min heap because it is now weird
        if len(H) > 0:
            H = self.bubbleDown(H)
        return returnMe, H

    # Insert a new tuple into the min heap H at the correct position
    #
    # INPUT: H - the min heap H
    #        nodeTuple - the node to insert into the min heap H
    # RETURN: H - the min heap H with the correctly positioned min nodeTuple
    def insert(self, H, nodeTuple):
        if len(H) == 0: # If the heap is empty, then make the nodeTuple the root
            H.append(nodeTuple)
            self.map[nodeTuple[1]] = 0 # nodeTuple[1] = the node_id of the nodeTuple
        else:
            # add the node to the bottom level of the heap
            H.append(nodeTuple)
            childIndex = len(H) - 1
            self.map[nodeTuple[1]] = childIndex # nodeTuple[1] = the node_id of the nodeTuple

            # move the inserted node into the right place
            H = self.bubbleUp(H)
            return H

    # Update the distance value for a nodeTuple in the min heap H
    # and then correct the positioning of the min heap H
    #
    # INPUT: H - the min heap
    #        oldTuple - the old tuple to replace
    #        newTuple - the tuple to replace the old tuple
    # RETURN: H - the updated min heap
    def decreaseKey(self, H, oldTuple, newTuple):
        posInH = self.map[oldTuple[1]] # oldTuple[1] gives us the node_id

        # replace the old value with the new value
        H[posInH] = newTuple

        # Fix the min heap
        # We only bubble up because we have DECREASED the key
        # and this is a min heap, not a max heap
        H = self.bubbleUp(H, posInH)
        return H

    # Moves the low value tuple up the min heap to
    # correct an out of order min heap
    #
    # INPUT: H - The out of order min heap
    #        childIndex - the index of the inserted node in the min heap
    # RETURN: H - The corrected min heap
    def bubbleUp(self, H, childIndex):
        child = H[childIndex]
        parentIndex = (childIndex - 1) // 2
        parent = H[parentIndex]

        while child[0] < parent[0]: # tuple[0] refers to the distance value
            # change places in H
            H = self.swap(parent, parentIndex, child, childIndex, H)

            # Update the parent and the indices
            childIndex = parentIndex
            parentIndex = (childIndex - 1) // 2
            parent = H[parentIndex]
        return H

    # Moves the high value tuple down the min heap to
    # correct an out of order min heap
    #
    # INPUT: H - The out of order min heap
    # RETURN: H - The corrected min heap
    def bubbleDown(self, H):
        parentIndex = 0
        done = False

        while not done:
            parent = H[parentIndex]
            leftChild, leftChildIndex = self.getLeftChild(parent, H)
            rightChild, rightChildIndex = self.getRightChild(parent, H)

            if leftChild and rightChild:
                if leftChild[0] < rightChild[0]: # Compare their distance values
                    # Swap the parent and leftChild in the min heap H
                    H, parentIndex = self.swapLeftChild(parent, parentIndex, leftChild, leftChildIndex, H)
                else: # RightChild has smaller distance
                    # Swap the parent and the right child in the min heap H
                    H, parentIndex = self.swapRightChild(parent, parentIndex, rightChild, rightChildIndex)
            elif leftChild and not rightChild:
                # Swap the parent and the left child in the min heap H
                H, parentIndex = self.swapLeftChild(parent, parentIndex, leftChild, leftChildIndex)
            elif not leftChild and rightChild:
                # Swap the parent and the right child in the min heap H
                H, parentIndex = self.swapRightChild(parent, parentIndex, rightChild, rightChildIndex)
            else:
                done = True
        return H

    # Swaps the parent and the left child in the min Heap H
    #
    # RETURN: The changed min heap H, and the leftChildIndex
    # because the leftChildIndex in the corrected min heap H
    # is now the parentIndex
    def swapLeftChild(self, parent, parentIndex, leftChild, leftChildIndex, H):
        H = self.swap(parent, parentIndex, leftChild, leftChildIndex, H)
        return H, leftChildIndex

    # Swaps the parent and the right child in the min Heap H
    #
    # RETURN: The changed min heap H, and the rightChildIndex
    # because the rightChildIndex in the corrected min heap H
    # is now the parentIndex
    def swapRightChild(self, parent, parentIndex, rightChild, rightChildIndex, H):
        H = self.swap(parent, parentIndex, rightChild, rightChildIndex, H)
        return H, rightChildIndex

    # Performs the actual swapping of elements and updates their min heap positions
    # in the map
    #
    # RETURN: H, the corrected min heap H
    def swap(self, parent, parentIndex, child, childIndex, H):
        H[parentIndex] = child
        self.map[child[1]] = parentIndex # child[1] gives us the node_id from the tuple
        H[childIndex] = parent
        self.map[parent[1]] = childIndex # parent[1] gives us the node_id from the tuple
        return H


    # Get the left child of the parent tuple in the mean heap
    #
    # INPUT: parent - the parent tuple in the min heap
    #        H - the min heap
    # RETURN: the leftChild tuple in the heap, or None if it doesn't exist
    def getLeftChild(self, parent, H):
        try:
            parentNodeId = parent[1] # [0] = distValue [1] = node_id
            leftChildIndexInH = (2 * self.map[parentNodeId]) + 1
            leftChildTuple = H[leftChildIndexInH]
            return leftChildTuple, leftChildIndexInH
        except:
            return None

    # Get the right child of the parent tuple in the mean heap
    #
    # INPUT: parent - the parent tuple in the min heap
    #        H - the min heap
    # RETURN: the rightChild tuple in the heap, or None if it doesn't exist
    def getRightChild(self, parent, H):
        try:
            parentNodeId = parent[1] # [0] = distValue [1] = node_id
            rightChildIndexInH = (2 * self.map[parentNodeId]) + 2
            rightChildTuple = H[rightChildIndexInH]
            return rightChildTuple, rightChildIndexInH
        except:
            return None


class ArrayQueue:

    def __init__(self, networkNodes, dist):
        self.networkNodes = networkNodes
        self.dist = dist
        pass

    # Make a list of items where the key is
    # the node in string form and the value is the
    # minimum distance to the node.
    #
    # INPUT: None
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
        minIndex = None
        minIndexFound = False
        for i in range(len(H)):
            tempDist = H[i][1]
            if not minIndexFound and tempDist != -1:
                minIndex = i
                minIndexFound = True
            elif minIndexFound:
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
        # TODO: Basically, I mixed up src and dest. I can fix that.
        self.dest = destIndex
        # TODO: RETURN THE SHORTEST PATH FOR destIndex
        #       INSTEAD OF THE DUMMY SET OF EDGES BELOW
        #       IT'S JUST AN EXAMPLE OF THE FORMAT YOU'LL 
        #       NEED TO USE
        path_edges = []
        total_length = 0
        destNode = self.network.nodes[self.dest]
        foundSrc = False
        maxSearches = len(self.network.nodes)
        while maxSearches > 0:

            if self.prev[destIndex] == None:
                foundSrc = True
                break
            total_length += self.dist[destIndex]
            prevIndex = self.prev[destIndex]
            prevNode = self.network.nodes[prevIndex]
            prevEdge = None
            for i in self.network.nodes[prevIndex].neighbors:
                if i.dest.node_id == destNode.node_id:
                    prevEdge = i
            total_length += prevEdge.length

            # Add to the edge to the path
            path_edges.append((prevEdge.src.loc, prevEdge.dest.loc, '{:.0f}'.format(prevEdge.length)))

            # Update values for next iteration
            destIndex = prevNode.node_id
            destNode = prevNode
            maxSearches -= 1
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

