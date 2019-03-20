import numpy
from RRTTree import RRTTree
import random
import time
class RRTPlanner(object):

    def __init__(self, planning_env):
        self.planning_env = planning_env
        self.tree = RRTTree(self.planning_env)
        self.step_size = 5.0
        self.one_step = False
        self.p_threshold = 0.20
        self.cost = {}

    def Plan(self, start_config, goal_config, epsilon = 0.001):
        
        # Initialize an empty plan.
        plan = []
        self.cost[0] = 0
        # Start with adding the start configuration to the tree.
        start = time.time()
        
        self.tree.AddVertex(start_config)

        # Implement your planner here.
        plan.append(start_config)

        for i in range(8000):
            while True:
                if random.random() <= self.p_threshold:
                    x_rand = goal_config
                else:
                    x_rand = [random.randint(0,self.planning_env.xlimit[1]), random.randint(0,self.planning_env.ylimit[1])]
                if self.planning_env.state_validity_checker(x_rand):
                    break
            x_near_id,x_near_vertiex,vdist = self.tree.GetNearestVertex(x_rand)
            x_new = self.extend(x_near_vertiex,x_rand)
            
            if (not len(x_new)) or (not self.planning_env.state_validity_checker(x_new)):
                continue

            x_new_id = self.tree.AddVertex(x_new)
            self.tree.AddEdge(x_near_id, x_new_id)
            self.cost[x_new_id] = self.cost[x_near_id] + vdist
            if self.planning_env.compute_distance(x_new,goal_config) < epsilon:
                s_id = x_new_id
                temp_list = [x_new]
                while self.tree.edges[s_id] != self.tree.GetRootID():
                    temp_list.append(self.tree.vertices[s_id])
                    s_id = self.tree.edges[s_id]
                plan += temp_list[::-1]
                break
            
        plan.append(goal_config)
        end = time.time()
        print('The time is',str(end - start))
        print('The cost is ',self.cost[len(self.tree.vertices)-2])
        return numpy.array(plan)

    def extend(self,x_near,x_rand):
        # Implement an extend logic.
        # The output x_new is a list.
        x_new = []
        dis = self.planning_env.compute_distance(x_near,x_rand)
        if dis == 0:
            return []
        x_near = numpy.array(x_near,dtype = numpy.float32)
        x_rand = numpy.array(x_rand,dtype = numpy.float32)
        delta_x_new = x_rand - x_near
        if self.one_step:
            step_size = 1.0
            delta_x = delta_x_new[0]*(step_size/dis)
            delta_y = delta_x_new[1]*(step_size/dis)
            for i in range(1,int(dis/step_size)):
                temp = [x_near[0] + delta_x*i,x_near[1] + delta_y*i]
                if not self.planning_env.state_validity_checker(temp):
                    return []
            x_new = x_rand
        else:
            if dis  >= self.step_size:
                x_new.append(x_near[0] + delta_x_new[0]*(self.step_size/dis))
                x_new.append(x_near[1] + delta_x_new[1]*(self.step_size/dis))
            else:
                x_new = x_rand
        return x_new

    def ShortenPath(self, path):
        # Postprocessing of the plan.
        start_point = 0
        end_point = 1
        new_path = [path[start_point]]
        while end_point < len(path):
            if self.cut(start_point,end_point,path):
                end_point += 1
            else:
                new_path.append(path[end_point])
                start_point = end_point
                end_point += 1
        return new_path

    def cut(self,start_point,end_point,path):
        dis = self.planning_env.compute_distance(path[start_point],path[end_point])
        x_start = numpy.array(path[start_point],dtype = numpy.float32)
        x_end = numpy.array(path[end_point],dtype = numpy.float32)
        delta_x_new = x_start - x_end
        
        step_size = 0.1
        delta_x = delta_x_new[0]*(step_size/dis)
        delta_y = delta_x_new[1]*(step_size/dis)
        for i in range(1,int(dis/step_size)):
            temp = [x_start[0] + delta_x*i,x_start[1] + delta_y*i]
            if not self.planning_env.state_validity_checker(temp):
                return False
        return True
