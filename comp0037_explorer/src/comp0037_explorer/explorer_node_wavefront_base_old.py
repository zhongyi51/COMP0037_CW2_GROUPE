import rospy
from nav_msgs.msg import Odometry
from explorer_node_base import ExplorerNodeBase

class ExplorerNodeWavefrontBase(ExplorerNodeBase):

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
        except AttributeError: # pass for the setup
            return
        pose = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates((position.x, position.y,))
        self._pose = pose

    def chooseNewDestination(self):
        pass

    def updateFrontiers(self): # TODO: implement this !!!
        # print 'current frontier list is:', self.frontierList
        new_frontierList = []
        self._dps(self.occupancyGrid, new_frontierList)
        if new_frontierList:
            print 'blackList is: ', self.blackList
            print 'new list is: ', new_frontierList
            self.frontierList = new_frontierList[:]
            return True



    def destinationReached(self, goal, goalReached):
        if goalReached is False:
#             print 'Adding ' + str(goal) + ' to the naughty step'
            self.blackList.append(goal)
            # if goal in self.frontierList: #my mod: There is a case where the single logic flow in other objects due to original implementation caused never updating the frontier list. This solved the problem
            #     self.updateFrontiers()

    # my mod: wavefront dps
    def _dps(self, map, lst):
        if not self._searchStartPos:
            self._init_searchStartPos()

        if not self._pose:
            self._pose = self._searchStartPos

        w, h = map.getWidthInCells(), map.getHeightInCells()
        x, y = self._pose
        # print 'pose:', self._pose
        visited = [] # ref
        self._dpsHelper(x, y, visited, w, h, map, lst)

    def _dpsHelper(self, x, y, visited, width, height, map, lst):
        stack = [(x+1, y,), (x-1, y,), (x, y+1,), (x, y-1,)]

        while stack:
            # print 'debug stack:', stack
            candidate = stack.pop()
            x, y = candidate
            if x < 0 or y < 0 or x >= width or y >= height:
                # print x,y,'out of bound'
                continue

            if candidate in visited:
                continue

            if map.getCell(x,y) != 0:
                # print x,y, 'others'
                continue

            visited.append(candidate)
            if self.isFrontierCell(x,y) and not candidate in self.blackList:
                lst.append(candidate) # the actuation

            stack += [(x+1, y,), (x-1, y,), (x, y+1,), (x, y-1,)]


    # maybe not needed if the initialization of cur robot position initializatoin is handled well ennough
    def _init_searchStartPos(self):
        for x in range(0, self.occupancyGrid.getWidthInCells()):
            for y in range(0, self.occupancyGrid.getHeightInCells()):
                if self.occupancyGrid.getCell(x,y) == 0: # if the cell is free
                    self._searchStartPos = (x,y,)
                    return True

        print '_init_searchStartPos Failed ' #debug del
        return False
