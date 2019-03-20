import numpy
from RRTTree import RRTTree
import random
import time

class RRTStarPlanner(object):

    def __init__(self, planning_env):
        self.planning_env = planning_env
        self.tree = RRTTree(self.planning_env)
        self.step_size = 5.0
        self.one_step = True
        self.cost = {}
        self.neighbers = 3
        self.p_threshold = 0.05

    def Plan(self, start_config, goal_config, epsilon = 0.001):
        
        # Initialize an empty plan.
        plan = []

        # Start with adding the start configuration to the tree.
        start = time.time()
        self.tree.AddVertex(start_config)
        
        plan.append(start_config)
        self.cost[0] = 0

        for i in range(8000):
            while True:
                if random.random() <= self.p_threshold:
                    x_rand = goal_config
                else:
                    x_rand = [random.randint(0,self.planning_env.xlimit[1]), random.randint(0,self.planning_env.ylimit[1])]
                if self.planning_env.state_validity_checker(x_rand):
                    break
                
            x_near_id,x_near_vertex,vdist = self.tree.GetNearestVertex(x_rand)
            x_new = self.extend(x_near_vertex,x_rand)
            
            if (not len(x_new)) or (not self.planning_env.state_validity_checker(x_new)):
                continue
            
            if len(self.tree.vertices) > self.neighbers:
                # Change father
                knnIDs , x_new_neighbers_vertices,knnDists = self.tree.GetKNN(x_new,self.neighbers)
                total_dis = []
                for i in range(self.neighbers):
                    total_dis.append( self.cost[knnIDs[i]] + knnDists[i])
                new_father_id = knnIDs[total_dis.index(min(total_dis))]
                x_new_id = self.tree.AddVertex(x_new)
                self.cost[x_new_id] = round(min(total_dis),2)
                # rewire
                for i in range(self.neighbers):
                    if self.cost[x_new_id] + knnDists[i] < self.cost[knnIDs[i]]:
                        self.tree.AddEdge(x_new_id,knnIDs[i])
                        self.cost[knnIDs[i]] = self.cost[x_new_id] + round(knnDists[i],2)
            else:
                new_father_id = x_near_id
                x_new_id = self.tree.AddVertex(x_new)
                self.cost[x_new_id] = self.cost[new_father_id] + round(vdist,2)
            self.tree.AddEdge(new_father_id, x_new_id)
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
