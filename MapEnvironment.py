import numpy
from IPython import embed
from matplotlib import pyplot as plt

class MapEnvironment(object):
    
    def __init__(self, mapfile, start, goal):

        # Obtain the boundary limits.
        # Check if file exists.
        self.map = numpy.loadtxt(mapfile)
        self.xlimit = [1, numpy.shape(self.map)[0]]
        self.ylimit = [1, numpy.shape(self.map)[1]]
        
        self.goal = goal
        # Check if start and goal are within limits and collision free
        if not self.state_validity_checker(start) or not self.state_validity_checker(goal):
            raise ValueError('Start and Goal state must be within the map limits');
            exit(0)

        # Display the map
        plt.imshow(self.map, interpolation='nearest')

    def compute_distance(self, start_config, end_config):
        
        #
        # Implement a function which computes the distance between
        # two configurations.
        #
        return numpy.power(numpy.sum(numpy.power( numpy.array(start_config,dtype=numpy.float32) - numpy.array(end_config,dtype=numpy.float32),2)),0.5)


    def state_validity_checker(self, config):

        #
        # Implement a state validity checker
        # Return true if valid.
        #
        if not self.edge_validity_checker(config,self.map):
            return False
        if self.map[int(round(config[0])),int(round(config[1]))] == 1.0:
            return False
        return True

    def edge_validity_checker(self, config1, config2):

        #
        # Implement an edge validity checker
        #
        #
        if config1[0] < 0 or config1[0] >= config2.shape[0]:
            return False
        if config1[1] < 0 or config1[1] >= config2.shape[1]:
            return False
        return True

    def compute_heuristic(self, config):
        
        #
        # Implement a function to compute heuristic.
        #
        return self.compute_distance(config,self.goal)

    def visualize_plan(self, plan):
        '''
        Visualize the final path
        @param plan Sequence of states defining the plan.
        '''
        plt.imshow(self.map, interpolation='nearest')
        for i in range(numpy.shape(plan)[0] - 1):
            x = [plan[i,0], plan[i+1, 0]]
            y = [plan[i,1], plan[i+1, 1]]
            plt.plot(y, x, 'k')
        plt.show()