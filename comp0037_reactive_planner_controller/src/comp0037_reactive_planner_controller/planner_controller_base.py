# This base class defines the API of a class which wraps around the planner and
# controller to produce reactive operations.

class PlannerControllerBase(object):

    def __init__(self, occupancyGrid, planner, controller):
        self.occupancyGrid = occupancyGrid
        self.planner = planner
        self.controller = controller
        self.currentPlannedPath = None

    def mapUpdateCallback(self, mapUpdateMessage):
        raise NotImplementedError()
    
    def driveToGoal(self, goal):
        raise NotImplementedError()
