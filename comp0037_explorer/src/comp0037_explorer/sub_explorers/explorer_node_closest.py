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

    def chooseNewDestination(self):

#         print 'blackList:'
#         for coords in self.blackList:
#             print str(coords)

        if self.updateFrontiers()==True:

            minD=99999
            minCell=[0,0]
            print "Current position:"+str(self.coordinates)

            for i in self.frontierList:
                deltaD=(i[0]-self.coordinates[0])**2+(i[1]-self.coordinates[1])**2
                if deltaD<minD:
                    minD=deltaD
                    minCell=i

            return True,minCell

        return False,None
