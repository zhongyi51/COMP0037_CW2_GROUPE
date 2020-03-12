# This class manages the key logic for the reactive planner and
# controller. This monitors the the robot mtion.

import rospy
import threading
from cell import CellLabel
from planner_controller_base import PlannerControllerBase
from comp0037_mapper.msg import *
from copy import deepcopy

# from the definitions in occupancy_grid.py
BLOCKED = 1.0
FREE = 0.0

class ReactivePlannerController(PlannerControllerBase):

    def __init__(self, occupancyGrid, planner, controller):
        PlannerControllerBase.__init__(self, occupancyGrid, planner, controller)

        self.mapUpdateSubscriber = rospy.Subscriber('updated_map', MapUpdate, self.mapUpdateCallback)
        self.gridUpdateLock =  threading.Condition()
        self.currentPlannedPath = None
        self.lastPlannedPath = None # my mod for bad path. ie. to entering always trying to get into a unreachable goal problem.

    def mapUpdateCallback(self, mapUpdateMessage):

        # Update the occupancy grid and search grid given the latest map update
        self.gridUpdateLock.acquire()
        self.occupancyGrid.updateGridFromVector(mapUpdateMessage.occupancyGrid)
        self.planner.handleChangeToOccupancyGrid()
        self.gridUpdateLock.release()

        # If we are not currently following any route, drop out here.
        if self.currentPlannedPath is None:
            return

        self.checkIfPathCurrentPathIsStillGood()

    def checkIfPathCurrentPathIsStillGood(self):

        # This methods needs to check if the current path, whose
        # waypoints are in self.currentPlannedPath, can still be
        # traversed

        # my mod: check through the waypoints on the current planned path, to see if any one is blocked from the perceptions of robot
        for wp in self.currentPlannedPath.waypoints:
            x,y = wp.coords
            print 'checking is x,y good: ', x, y #debug del
            status = self.occupancyGrid.getCell(x,y)
            if status == BLOCKED or status == 1: # check status from the continuously updating occupancy grid
                print 'coords = ', x, y, 'status = ', status, 'Now stop and replan' # debug del
                self.controller.stopDrivingToCurrentGoal() # stop the robot
                break

    def driveToGoal(self, goal):

        # Get the goal coordinate in cells
        goalCellCoords = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates((goal.x,goal.y))

        # Reactive planner main loop - keep iterating until the goal is reached or the robot gets
        # stuck.

        goalReached = False

        while (goalReached is False) & (rospy.is_shutdown() is False):

            # Set the start conditions to the current position of the robot
            pose = self.controller.getCurrentPose()
            start = (pose.x, pose.y)
            startCellCoords = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates(start)

            print 'Planning a new path: start=' + str(start) + '; goal=' + str(goal)

            # Plan a path using the current occupancy grid
            self.gridUpdateLock.acquire()
            pathToGoalFound = self.planner.search(startCellCoords, goalCellCoords)
            self.gridUpdateLock.release()

            # my mod: debug
            if self._isWaypointsOfPathsEqual(self.lastPlannedPath, self.currentPlannedPath):         # my mod: for discovering a intrigueing situation
                rospy.logwarn('Two same path decided to a goal detected. they are', list(self.lastPlannedPath.waypoints), 'and', list(self.currentPlannedPath.waypoints))

            # If we can't reach the goal, give up and return
            if pathToGoalFound is False:
                rospy.logwarn("Could not find a path to the goal at (%d, %d)", \
                              goalCellCoords[0], goalCellCoords[1])
                self.controller.stopDrivingToCurrentGoal() # my mod: stop here
                return False

            # Extract the path
            self.lastPlannedPath = deepcopy(self.currentPlannedPath) # my mod: make a copy to compare later
            self.currentPlannedPath = self.planner.extractPathToGoal()

            # Drive along the path towards the goal. This returns True
            # if the goal was successfully reached. The controller
            # should stop the robot and return False if the
            # stopDrivingToCurrentGoal method is called.
            goalReached = self.controller.drivePathToGoal(self.currentPlannedPath, \
                                                          goal.theta, self.planner.getPlannerDrawer())

            rospy.logerr('goalReached=%d', goalReached)

        return goalReached

        # my mod: for discovering a intrigueing situation
        def _isWaypointsOfPathsEqual(fromPath, toPath):
            '''check if two routes are the same, to prevent entering a forever loop setting unreachable goal when the map is fully explored'''
            print 'Checking for wps equaility.' # debug del
            if not fromPath or not toPath or not fromPath.waypoints or not toPath.waypoints or len(fromPath) != len(toPath):
                return False

            fromWps, toWps = fromPath.waypoints, toPath.waypoints
            for wp1, wp2 in zip(fromWps, toWps):
                print wp1.coords, wp2.coords # debug del
                if wp1.coords != wp2.coords:
                    return False

            return True
