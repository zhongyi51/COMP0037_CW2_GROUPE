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

        self._reset_wavefront_attribute_states()

    def _reset_wavefront_attribute_states(self):
        self.q_m = []
        self.q_f = []
        self.map_open = []
        self.map_close = []
        self.frontier_open = []
        self.frontier_close = []
        self.new_frontiers = []

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
        self._wavefront(new_frontierList)
        if new_frontierList:
            print 'blackList is: ', self.blackList
            print 'new list is: ', new_frontierList
            self.frontierList = new_frontierList[:]
            return True



    def destinationReached(self, goal, goalReached):
        if goalReached is False:
#             print 'Adding ' + str(goal) + ' to the naughty step'
            self.blackList.append(goal)

    # my mod: wavefront dps
    def _wavefront(self, lst):
        if not self._searchStartPos:
            self._init_searchStartPos()

        if not self._pose:
            self._pose = self._searchStartPos

        pose = self._pose
        # print 'pose:', self._pose
        self._reset_wavefront_attribute_states()
        self._wavefrontHelper(pose, lst)

    def _wavefrontHelper(self, pose, lst):
        self.q_m = [pose]
        self.map_open.append(pose)

        while self.q_m:
            # print 'q_m: ', self.q_m
            p = self.q_m.pop(0)
            x_p, y_p = p

            if p in self.map_close or p in self.blackList:
                continue

            if self.isFrontierCell(x_p,y_p):
                self.q_f = []
                self.new_frontiers = []
                self.q_f.append(p)
                self.frontier_open.append(p)

                while self.q_f:
                    # print 'q_f: ', self.q_f
                    q = self.q_f.pop(0)
                    if q in self.map_close or q in self.frontier_close or q in self.blackList:
                        continue

                    x_q, y_q = q
                    if self.isFrontierCell(x_q, y_q):
                        self.new_frontiers.append(q)
                        for w in self._neighbours(q):
                            if not w in self.frontier_open and not w in self.frontier_close and not w in self.map_close:
                                self.q_f.append(w)
                                self.frontier_open.append(w)

                    self.frontier_close.append(q)
                lst += self.new_frontiers # save data
                self.map_close += self.new_frontiers # mark frontiers to map close list

            for v in self._neighbours(p):
                if not v in self.map_open and not v in self.map_close and self._hasAtLeastOneOpenNeighbour(v):
                    self.q_m.append(v)
                    self.map_open.append(v)
            self.map_close.append(p)

    def _hasAtLeastOneOpenNeighbour(self, coord):
        for neighbour in self._neighbours(coord):
            x, y = neighbour
            if not 0 <= x < self.occupancyGrid.getWidthInCells() or not 0 <= y < self.occupancyGrid.getHeightInCells(): # out of range
                return False
            if self.occupancyGrid.getCell(x,y) == 0:
                return True

    def _neighbours(self, coord):
        x, y = coord
        return [(x + 1, y,), (x - 1, y,), (x, y + 1,), (x, y - 1,)]

    # maybe not needed if the initialization of cur robot position initializatoin is handled well ennough
    def _init_searchStartPos(self):
        for x in range(0, self.occupancyGrid.getWidthInCells()):
            for y in range(0, self.occupancyGrid.getHeightInCells()):
                if self.occupancyGrid.getCell(x,y) == 0: # if the cell is free
                    self._searchStartPos = (x,y,)
                    return True

        print '_init_searchStartPos Failed ' #debug del
        return False
