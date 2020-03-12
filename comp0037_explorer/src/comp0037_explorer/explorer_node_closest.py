import rospy

from explorer_node_base import ExplorerNodeBase

# This class implements a super dumb explorer. It goes through the
# current map and marks the first cell it sees as the one to go for

class ExplorerNode(ExplorerNodeBase):

    def __init__(self):

        self.blackList = []
        self.frontierList=[]
        self.coordinates=[0,0]

        ExplorerNodeBase.__init__(self)

    def updateFrontiers(self): # TODO: implement this !!!
        new_frontierList=[]
        for x in range(0, self.occupancyGrid.getWidthInCells()):
            for y in range(0, self.occupancyGrid.getHeightInCells()):
                candidate=(x,y)
                if self.isFrontierCell(x, y) is True and candidate not in self.blackList:
                    new_frontierList.append(candidate)

        # print "The frontierList is:"+str(self.frontierList) # debug del

        if len(new_frontierList)!=0:
            self.frontierList=new_frontierList[:]
            return True

        return False




    def chooseNewDestination(self):

#         print 'blackList:'
#         for coords in self.blackList:
#             print str(coords)

        if self.frontierList:

            minD=99999999
            minCell=self.frontierList[0]
            print "Current position:"+str(self.coordinates)


            for i in self.frontierList:
                deltaD=(i[0]-self.coordinates[0])**2+(i[1]-self.coordinates[1])**2
                if deltaD<minD:
                    minD=deltaD
                    minCell=i

            return True,minCell

        return False,None


    def destinationReached(self, goal, goalReached):
        if goalReached is False:
#             print 'Adding ' + str(goal) + ' to the naughty step'
            self.blackList.append(goal)
