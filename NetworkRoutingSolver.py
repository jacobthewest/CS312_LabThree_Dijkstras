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
    # ---TIME COMPLEXITY---
    # Time complexity---: O(log(V))
    # Discussion---: The insert operation is called V times.
    # the insert operation on a min heap queue is O(log(V)).
    # Even though we are calling the insert operation V times,
    # the time complexity will be just O(log(V)) because as V
    # tends towards infinity, the min heap will grow bigger and
    # we won't have to inspect the entire heap on each insertion.
    def makeHeap(self):
        H = []
        for node in self.networkNodes:
            key = self.dist[node.node_id]
            value = node.node_id
            nodeTuple = (key,value)
            H = self.insert(H, nodeTuple)
        return H

    # Returns the tuple with the smallest distance to it
    # from the minHeap and then corrects the min heap.
    #
    # INPUT: H - The min heap
    # RETURN: The node with the smallest distance to it
    # ---TIME COMPLEXITY---
    # Time complexity---: O(log(V))
    # Discussion---: To return the minimum value off the min
    # heap is just O(1) time because the minimum value is
    # always the root value. To return the min value, we
    # swap the min value and the end value of the min heap.
    # Then we pop off the last index of the min heap (which
    # is now the min value). This is O(1) time. Then we have to trickle down
    # the higher out-of-order value back into a good place
    # in the tree. This trickling down of the value is O(log(V))
    # time. O(log(V)) dominates the O(1) time, therefore the time
    # complexity of the deleteMin() function is O(log(V)).
    def deleteMin(self, H):
        # Swap the first and last element in the min heap
        temp = H[0]
        H[0] = H[-1]
        H[-1] = temp

        # Pop off the min element
        returnMe = H.pop(len(H) - 1)
        self.map[returnMe[1]] = None # [1] refers to the node_id. Say the returned
                                     # node no longer exists in the min heap.
        if len(H) > 0:
            self.map[H[0][1]] = 0 # Set the new root node's heap position to 0

            # Correct the min heap because it is now weird
            H = self.bubbleDown(H)

        # returnMe is a tuple, find its node pair is self.networkNodes
        returnMe = self.networkNodes[returnMe[1]] # returnMe[1] because that is the node_id
        return returnMe, H

    # Insert a new tuple into the min heap H at the correct position
    #
    # INPUT: H - the min heap H
    #        nodeTuple - the node to insert into the min heap H
    # RETURN: H - the min heap H with the correctly positioned min nodeTuple
    # ---TIME COMPLEXITY---
    # Time complexity---: O(log(V))
    # Discussion---: We insert by adding the node to the end of the heap array.
    # This is O(1) time. Then we have to make sure the node is in the correct place.
    # More often than not, the node is too low in the tree and needs to be bubbled
    # up the min heap. Bubbling the node up the tree takes O(log(V)) time. Because
    # O(log(V)) dominates O(1) time, the total time complexity of the insert operation
    # is O(log(V)) time.
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
    #        node_id - the old tuple to replace
    #        newDistance - the newDistance to the node.
    #                      also the key of the tuple
    # RETURN: H - the updated min heap
    # Time complexity---: O(log(V))
    # Discussion---: The decreaseKey() function is O(log(V)) time because
    # once we change the key value, we need to put the key in the correct
    # place. Because we DECREASED the value of the key, and we are
    # working with a min heap, we have only to worry about bubbling the
    # value up the min heap. This bubbling up function takes O(log(V))
    # time. Changing the value of the key is O(1) time. Because
    # O(log(V)) time dominates O(1) time, the time complexity
    # of the function is O(log(V)).
    def decreaseKey(self, H, node_id, newDistance):
        key = newDistance
        value = node_id
        newTuple = (key, value)

        posInH = self.map[node_id]

        # replace the old value with the new value
        if len(H) != 0:
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
    #        childIndex - the index of the nodeTuple to bobble up
    # RETURN: H - The corrected min heap
    # Time complexity---: O(log(V))
    # Discussion---: The bubbleUp() function is O(log(V)) time
    # because it only requires us to check the nodes above
    # the current node, we never have to look out for the nodes
    # left of or below the current node. Therefore, the time
    # complexity of this function is O(log(V)).
    def bubbleUp(self, H, childIndex=None):
        if childIndex == None:
            childIndex = len(H) - 1 # It will always be the last index
        child = H[childIndex]
        parentIndex = (childIndex - 1) // 2
        parent = H[parentIndex]

        while child[0] < parent[0]: # tuple[0] refers to the distance value
            # change places in H
            H = self.swap(parent, parentIndex, child, childIndex, H)

            # Update the parent and the indices
            childIndex = parentIndex
            parentIndex = (childIndex - 1) // 2
            if(parentIndex < 0):
                parent = (float("-inf"), "This is the root node")
            else:
                parent = H[parentIndex]
        return H

    # Moves the high value tuple down the min heap to
    # correct an out of order min heap
    #
    # INPUT: H - The out of order min heap
    # RETURN: H - The corrected min heap
    # Time complexity---: O(log(V))
    # Discussion---: The bubbleDown() function is O(log(V)) time
    # because it only requires us to check the nodes below
    # the current node, we never have to look out for the nodes
    # above or on the other side of the min heap. Therefore, the time
    # complexity of this function is O(log(V)).
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
                elif leftChild[0] > rightChild[0]: # RightChild has smaller distance
                    # Swap the parent and the right child in the min heap H
                    H, parentIndex = self.swapRightChild(parent, parentIndex, rightChild, rightChildIndex, H)
                else: # They are the same size.
                    done = True
            elif leftChild and not rightChild:
                # Swap the parent and the left child in the min heap H
                H, parentIndex = self.swapLeftChild(parent, parentIndex, leftChild, leftChildIndex, H)
            elif not leftChild and rightChild:
                # Swap the parent and the right child in the min heap H
                H, parentIndex = self.swapRightChild(parent, parentIndex, rightChild, rightChildIndex, H)
            else:
                done = True
        return H

    # Swaps the parent and the left child in the min Heap H
    #
    # RETURN: The changed min heap H, and the leftChildIndex
    # because the leftChildIndex in the corrected min heap H
    # is now the parentIndex
    # Time complexity---: O(1)
    # Discussion---: The function is only O(1) time because
    # we are performing setting of nodes and retrieval of
    # nodes with known indices.
    def swapLeftChild(self, parent, parentIndex, leftChild, leftChildIndex, H):
        H = self.swap(parent, parentIndex, leftChild, leftChildIndex, H)
        return H, leftChildIndex

    # Swaps the parent and the right child in the min Heap H
    #
    # RETURN: The changed min heap H, and the rightChildIndex
    # because the rightChildIndex in the corrected min heap H
    # is now the parentIndex
    # Time complexity---: O(1)
    # Discussion---: The function is only O(1) time because
    # we are performing setting of nodes and retrieval of
    # nodes with known indices.
    def swapRightChild(self, parent, parentIndex, rightChild, rightChildIndex, H):
        H = self.swap(parent, parentIndex, rightChild, rightChildIndex, H)
        return H, rightChildIndex

    # Performs the actual swapping of elements and updates their min heap positions
    # in the map
    #
    # RETURN: H, the corrected min heap H
    # Time complexity---: O(1)
    # Discussion---: The function is only O(1) time because
    # we are performing setting of nodes and retrieval of
    # nodes with known indices.
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
    # Time complexity---: O(1)
    # Discussion---: The function is only O(1) time because
    # we are performing the retrieval of
    # nodes with known indices.
    def getLeftChild(self, parent, H):
        try:
            parentNodeId = parent[1] # [0] = distValue [1] = node_id
            leftChildIndexInH = (2 * self.map[parentNodeId]) + 1
            if leftChildIndexInH < 0 or leftChildIndexInH > len(H) - 1:
                return None, None
            leftChildTuple = H[leftChildIndexInH]
            return leftChildTuple, leftChildIndexInH
        except:
            return None, None

    # Get the right child of the parent tuple in the mean heap
    #
    # INPUT: parent - the parent tuple in the min heap
    #        H - the min heap
    # RETURN: the rightChild tuple in the heap, or None if it doesn't exist
    # Time complexity---: O(1)
    # Discussion---: The function is only O(1) time because
    # we are performing the retrieval of
    # nodes with known indices.
    def getRightChild(self, parent, H):
        try:
            parentNodeId = parent[1] # [0] = distValue [1] = node_id
            rightChildIndexInH = (2 * self.map[parentNodeId]) + 2
            if rightChildIndexInH < 0 or rightChildIndexInH > len(H) - 1:
                return None, None
            rightChildTuple = H[rightChildIndexInH]
            return rightChildTuple, rightChildIndexInH
        except:
            return None, None


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
    # Time complexity---: O(V)
    # Discussion---: The function is O(V) because
    # we are creating a list of nodes and inserting
    # each node into its respective location based
    # on node_id. We need to place every node in its
    # own index. Therefore, the time complexity is O(V).
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
    # Time complexity---: O(V)
    # Discussion---: The function is O(V) because
    # we are searching the entire list and seeing
    # if the value is lower than the smallest value
    # that we have seen. Therefore, the time complexity
    # is O(V).
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
    # Time complexity---: O(1)
    # Discussion---: The function is O(1) because
    # we are retrieving a node based off of a known
    # index. Then we are changing that value.
    # These are constant time operations. Therefore,
    # the complexity of this function is O(1).
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

    # Time complexity---: O(n^3)
    # Discussion---: This function is O(n^3) because
    # we are checking every node, and for every node
    # we are checking each of its three neighbors
    # to see what the shortest path may be. We are
    # checking each of the three neighbors inside
    # of a while loop that iterates for every node.
    # Because of the nested nature of this three
    # neighbor check, the worst case time complexity
    # is O(n^3).
    def getShortestPath( self, destIndex ):
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


    # Time complexity---:
    # Discussion---: O(E*log(V)) and O(V^2 + V*E)
    # min heap priority queue: The operation is executed
    # O(V + E) times. The decreaseKey(), insert(), and deleteMin()
    # operations all take O(log(V)) time. Therefore, the time
    # complexity is O(V + E) * O(log(V)) => O(E*log(V))
    #
    # array priority queue: The operation is executed O(V + E)
    # times. deleteMin() is the most time heavy operation at
    # O(V) time. Therefore, the time complexity is
    # O(V + E) * O(V) => O(V^2 + V*E)
    def computeShortestPaths( self, srcIndex, use_heap=False ):
        self.source = srcIndex
        t1 = time.time()

        # Initialize all node distances to infinity.
        # Set src node distance to zero
        self.dist = [self.infinity] * len(self.network.nodes)
        self.prev = [None] * len(self.network.nodes)
        self.dist[srcIndex] = 0

        if use_heap: # use heapQueue
            minHeapQueue = HeapQueue(self.network.nodes, self.dist)
            H = minHeapQueue.makeHeap()
            minsDeleted = 0
            originalLengthOfH = len(H)
            while minsDeleted < originalLengthOfH: # While H is not empty
                u, H = minHeapQueue.deleteMin(H)
                minsDeleted += 1
                for neighborEdge in u.neighbors:
                    neighborNode = neighborEdge.dest;
                    newDistance = self.dist[u.node_id] + neighborEdge.length
                    if self.dist[neighborNode.node_id] > newDistance:
                        self.dist[neighborNode.node_id] = newDistance
                        self.prev[neighborNode.node_id] = u.node_id
                        H = minHeapQueue.decreaseKey(H, neighborNode.node_id, newDistance)

        else: # use arrayQueue
            arrayQueue = ArrayQueue(self.network.nodes, self.dist)
            H = arrayQueue.makeQueue()
            minsDeleted = 0
            originalLengthOfH = len(H)
            while minsDeleted < originalLengthOfH: # While H is not empty
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

