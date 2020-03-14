import rospy
from nav_msgs.msg import Odometry
from explorer_node_base import ExplorerNodeBase

# This class implements a super dumb explorer. It goes through the
# current map and marks the first cell it sees as the one to go for

class ExplorerNode(ExplorerNodeBase):

    def __init__(self):
        self.blackList = []
        self.frontierList=[]

        self._searchStartPos = None
        self._pose = None # current robot pos
        ExplorerNodeBase.__init__(self)
        self._odometrySubscriber = rospy.Subscriber('/robot0/odom', Odometry, self._callback) # added for tracking current robot position

    def _callback(self, odometry):
        odometryPose = odometry.pose.pose
        position = odometryPose.position

        try:
            self.occupancyGrid
        except AttributeError: # wait for the setup
            return
        pose = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates((position.x, position.y,))
        # print 'callback received, current pos is: ', pose #debug del
        self._pose = pose


    def updateFrontiers(self): # TODO: implement this !!!
        # print 'current frontier list is:', self.frontierList
        new_frontierList = []
        self._dps(self.occupancyGrid, new_frontierList)
        if new_frontierList:
            print 'new list is: ', new_frontierList
            self.frontierList = new_frontierList[:]
            return True

    def chooseNewDestination(self):

        print 'blackList:', [coords for coords in self.blackList]

        if self.updateFrontiers() or self.frontierList:

            minD=99999999
            minCell=(-1,-1)

            for i in self.frontierList:
                deltaD=(i[0]-self._pose[0])**2+(i[1]-self._pose[1])**2
                if deltaD<0.5: # dont choose itself for a special case
                    continue
                if deltaD<minD:
                    minD=deltaD
                    minCell=i

            return True,minCell

        return False,None


    def destinationReached(self, goal, goalReached):
        if goalReached is False:
#             print 'Adding ' + str(goal) + ' to the naughty step'
            self.blackList.append(goal)
            if goal in self.frontierList: #my mod: There is a case where the single logic flow in other objects due to original implementation caused never updating the frontier list. This solved the problem
                self.frontierList.remove(goal)

    # my mod: wavefront dps
    def _dps(self, map, lst):
        if not self._searchStartPos:
            self._init_searchStartPos()

        if not self._pose:
            self._pose = self._searchStartPos

        w, h = map.getWidthInCells(), map.getHeightInCells()
        x, y = self._pose
        print 'pose:', self._pose
        visited = [] # ref
        self._dpsHelper(x, y, visited, w, h, map, lst)

    # my mod: helper
    def _dpsHelper(self, x, y, visited, width, height, map, lst):
        stack = [(x+1, y,), (x-1, y,), (x, y+1,), (x, y-1,)]

        while stack:
            # print 'debug stack:', stack
            candidate = stack.pop()
            x, y = candidate
            if x < 0 or y < 0 or x > width or y > height:
                # print x,y,'out of bound'
                continue

            if candidate in visited:
                continue

            if candidate in self.blackList or map.getCell(x,y) != 0.0 or map.getCell(x,y) != 0:
                # print x,y, 'others'
                continue

            visited.append(candidate)
            if self.isFrontierCell(x,y):
                lst.append(candidate) # the actuation

            # floodfill 4 angles only, I dont really like diagonal route, possibly try it seperatly later
            stack += [(x+1, y,), (x-1, y,), (x, y+1,), (x, y-1,)]
        return True

    def _init_searchStartPos(self): # maybe not needed if the initialization of cur robot position initializatoin is handled well ennough
        for x in range(0, self.occupancyGrid.getWidthInCells()):
            for y in range(0, self.occupancyGrid.getHeightInCells()):
                if self.occupancyGrid.getCell(x,y) == 0: # if the cell is free
                    self._searchStartPos = (x,y,)
                    return True

        print '_init_searchStartPos Failed ' #debug del
        return False
