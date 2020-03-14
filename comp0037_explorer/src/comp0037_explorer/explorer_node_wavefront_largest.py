import rospy
from explorer_node_wavefront_base import ExplorerNodeWavefrontBase

class ExplorerNode(ExplorerNodeWavefrontBase):

    def __init__(self):
        ExplorerNodeWavefrontBase.__init__(self)

    def isnear(self,cell,newcell):
        if abs(newcell[0]-cell[0])<=1 and abs(newcell[1]-cell[1])<=1 and newcell != cell:
            return True
        return False

    def chooseNewDestination(self):
        if self.updateFrontiers():
            maxcellnumber=0
            maxCell= self.frontierList[0]
            for i in self.frontierList:
                nearcellnumber=0
                for j in self.frontierList:
                    if self.isnear(i,j)==True:
                        nearcellnumber+=1

                if nearcellnumber>maxcellnumber:
                    maxcellnumber=nearcellnumber
                    maxCell=i

            print "max cell is:"+str(maxCell)
            return True,maxCell

        return False,None
