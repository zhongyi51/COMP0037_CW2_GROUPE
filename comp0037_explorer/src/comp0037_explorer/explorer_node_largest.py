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
        
        print "The frontierList is:"+str(self.frontierList)

        if len(new_frontierList)!=0:
            self.frontierList=new_frontierList[:]
            return True

        return False
                
	
    def isnear(self,cell,newcell):
        if abs(newcell[0]-cell[0])<=1 and abs(newcell[1]-cell[1])<=1 and newcell!=cell
            return True
        return False

    def chooseNewDestination(self):

#         print 'blackList:'
#         for coords in self.blackList:
#             print str(coords)
        
        if self.updateFrontiers()==True:
            
            maxcellnumber=0
            maxCell=[0,0]
            for i in self.frontierList:
                nearcellnumber=0
                for j in self.frontierList:
                    if isnear(i,j)==True:
                        nearcellnumber+=1
                
                if nearcellnumber>maxcellnumber:
                    maxcellnumber=nearcellnumber
                    maxCell=i

            print "max cell is:"+str(maxCell)
            return True,maxCell
            
        return False,None
    

    def destinationReached(self, goal, goalReached):
        if goalReached is False:
#             print 'Adding ' + str(goal) + ' to the naughty step'
            self.blackList.append(goal)
        else:
            self.coordinates=goal
