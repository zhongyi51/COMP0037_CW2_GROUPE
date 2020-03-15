import rospy
from explorer_node_wavefront_base import ExplorerNodeWavefrontBase

class ExplorerNode(ExplorerNodeWavefrontBase):

    def __init__(self):
        ExplorerNodeWavefrontBase.__init__(self)

    def chooseNewDestination(self):
        # print 'blackList:', [coords for coords in self.blackList]

        if self.updateFrontiers():
            minD=float('inf')
            minCell=(-1,-1)

            for i in self.frontierList:
                if i in self.blackList:
                    continue
                deltaD=(i[0]-self._pose[0])**2+(i[1]-self._pose[1])**2
                if deltaD<0.5: # dont choose very close one for some ploblems
                    continue
                if deltaD<minD:
                    minD=deltaD
                    minCell=i

            return True,minCell

        return False,None
