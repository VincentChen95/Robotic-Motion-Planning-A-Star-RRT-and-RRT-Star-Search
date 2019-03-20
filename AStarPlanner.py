import sys
import time
import numpy

import collections

class AStarPlanner(object):    
    def __init__(self, planning_env):
        self.planning_env = planning_env
        self.nodes = collections.defaultdict(list)
        self.epsilon = 20
        self.shorten = False
        
    def Plan(self, start_config, goal_config):

        plan = []

        # Implement your planner here.
        start = time.time()
        self.nodes[(start_config[0],start_config[1])] = [start_config]
        
        plan.append(start_config)
        
        closed_set = set()
        open_set = set()
        open_set.add((start_config[0],start_config[1]))
        
        g_score = 0
        h_score = self.epsilon*self.planning_env.compute_heuristic(start_config)
        f_score = g_score + h_score
        
        open_set_f = {}
        open_set_g = {}
        
        open_set_f[(start_config[0],start_config[1])] = f_score
        open_set_g[(start_config[0],start_config[1])] = g_score
        final_node = None
        
        while len(open_set):
            # the type of cur_node is tuple
            cur_node = sorted(open_set_f.items(),key = lambda item:item[1])[0][0]

            if cur_node[0] == goal_config[0] and cur_node[1] == goal_config[1]:
                plan += self.nodes[cur_node]
                final_node = cur_node
                break 
            
            open_set.remove(cur_node)
            closed_set.add(cur_node)
            del open_set_f[cur_node]
            
            for next_node_x,next_node_y in [[-1,0],[1,0],[0,1],[0,-1],[-1,-1],[1,1],[-1,1],[1,-1]]:
                temp_x = cur_node[0] + next_node_x
                temp_y = cur_node[1] + next_node_y
                
                if (temp_x,temp_y) in closed_set:
                    continue
                
                if not self.planning_env.state_validity_checker([temp_x,temp_y]):
                    continue
                
                temp_g_score = open_set_g[cur_node] + (next_node_x**2 + next_node_y**2)**0.5
                
                tentative_is_better = False
                if (temp_x,temp_y) not in open_set:
                    tentative_is_better = True
                # (temp_x,temp_y) already in open_set, if temp_gscore is better, we need to replace
                elif temp_g_score < open_set_g[(temp_x,temp_y)]:
                    tentative_is_better = True
                else:
                    tentative_is_better = False
                    
                if tentative_is_better:
                    # record the father node
                    self.nodes[(temp_x,temp_y)] = self.nodes[cur_node] + [[cur_node[0],cur_node[1]]]
                    open_set_g[(temp_x,temp_y)] = temp_g_score
                    h_score = self.epsilon*self.planning_env.compute_heuristic([temp_x,temp_y])
                    open_set_f[(temp_x,temp_y)] = h_score + temp_g_score
                    open_set.add((temp_x,temp_y))
                    
            del open_set_g[cur_node] 
            
        plan.append(goal_config)
        print(plan)
        if self.shorten:
            new_path = self.ShortenPath(plan)
            return numpy.array(new_path)
        end = time.time()
        print('The time is',str(end - start))
        print('The f cost is',open_set_f[final_node])
        print('The extend state is ',len(plan))
        return numpy.array(plan)

    def ShortenPath(self, path):
        # Postprocess the planner.
        start_point = 0
        end_point = 1
        new_path = [path[start_point]]
        while end_point < len(path):
            if self.extend(start_point,end_point,path):
                end_point += 1
            else:
                new_path.append(path[end_point])
                start_point = end_point
                end_point += 1
        return new_path

    def extend(self,start_point,end_point,path):
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
        
        