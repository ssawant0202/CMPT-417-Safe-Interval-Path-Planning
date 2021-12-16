import time as timer
import heapq
import random
import itertools
import copy

from sipp import compute_heuristics, sipp, get_location, get_sum_of_cost


def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.

    first_collision = []
    max_length = max(len(path1), len(path2))
    for timestep in range(max_length):
        loc1 = get_location(path1, timestep)
        loc2 = get_location(path2, timestep)

        # if vertex collision
        if loc1 == loc2:
            first_collision = {'loc': [loc1], 'timestep': timestep}
            return first_collision

        # if edge collision
        if timestep:
            previous_loc1 = get_location(path1, timestep - 1)
            previous_loc2 = get_location(path2, timestep - 1)
            if previous_loc1 == loc2 and previous_loc2 == loc1:
                first_collision = {'loc': [previous_loc1, loc1], 'timestep': timestep}
                return first_collision

    return first_collision  # No collision found


def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.
    all_collisions = []
    length_paths = len(paths)
    for a1 in range(length_paths):
        for a2 in range(a1 + 1, length_paths):
            collision_found = detect_collision(paths[a1], paths[a2])
            if collision_found:
                all_collisions.append({
                    'a1': a1,
                    'a2': a2,
                    'loc': collision_found['loc'],
                    'timestep': collision_found['timestep']
                })
    return all_collisions


def standard_splitting(collision):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep

    to_constraint = []
    constraint1 = {
        'agent': collision['a1'],
        'loc': collision['loc'],
        'timestep': collision['timestep'],
        'positive': False
    }
    constraint2 = {
        'agent': collision['a2'],
        'loc': collision['loc'],
        'timestep': collision['timestep'],
        'positive': False
    }
    if len(collision['loc']) == 2:
        constraint2['loc'] = [collision['loc'][1], collision['loc'][0]]

    to_constraint.append(constraint1)
    to_constraint.append(constraint2)

    return to_constraint


def disjoint_splitting(first_collision):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly
    to_constraint = []
    agent = ''
    if random.randint(0, 1) == 0:
        agent = first_collision['a1']
    else:
        agent = first_collision['a2']

    constraint1 = {
        'agent': agent,
        'loc': first_collision['loc'],
        'timestep': first_collision['timestep'],
        'positive': True
    }
    constraint2 = {
        'agent': agent,
        'loc': first_collision['loc'],
        'timestep': first_collision['timestep'],
        'positive': False
    }

    to_constraint.append(constraint1)
    to_constraint.append(constraint2)

    return to_constraint


class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def eliminate_duplicate_listsInList(self, mylist):
        mylist.sort()
        temp = list(mylist for mylist, _ in itertools.groupby(mylist))
        return temp

    def eliminate_duplicate_dictInList(self, test_list):
        res_list = [i for n, i in enumerate(test_list) if i not in test_list[n + 1:]]

        return res_list

    def paths_violate_constraint(self, constraint, paths):
        assert constraint['positive'] is True
        rst = []
        for i in range(len(paths)):
            if i == constraint['agent']:
                continue
            curr = get_location(paths[i], constraint['timestep'])
            prev = get_location(paths[i], constraint['timestep'] - 1)
            if len(constraint['loc']) == 1:  # vertex constraint
                if constraint['loc'][0] == curr:
                    rst.append(i)
            else:  # edge constraint
                if constraint['loc'][0] == prev or constraint['loc'][1] == curr \
                        or constraint['loc'] == [curr, prev]:
                    rst.append(i)
        return rst

    def remove_extra_locations(self, path):

        if path is None:
            return path
        counter = 0
        n = len(path) - 1
        while n >= 1:
            prev = path[n - 1]
            curr = path[n]
            if (prev == curr):
                counter += 1
                n -= 1
            else:
                break
        for pop in range(counter):
            path.pop()
        return path

    def find_solution(self, disjoint=True, curr=None):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = sipp(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                        i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)

        # Task 3.1: Testing
        # print(root['collisions'])

        # Task 3.2: Testing
        # for collision in root['collisions']:
        # print(standard_splitting(collision))

        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit


        while len(self.open_list) > 0:
            parent = self.pop_node()
            if len(parent['collisions']) == 0:
                self.print_results(parent)
                return parent['paths']
            first_collision = parent['collisions'][0]

            if disjoint == False:
                new_constraints = standard_splitting(first_collision)
            else:
                new_constraints = disjoint_splitting(first_collision)

            for constraint in new_constraints:
                deepCopy_constraints = copy.deepcopy(parent['constraints'])
                if constraint not in deepCopy_constraints:
                    deepCopy_constraints.append(constraint)
                deepCopy_paths = copy.deepcopy(parent['paths'])
                q = {
                    'constraints': deepCopy_constraints,  # might add duplicate constraints
                    'paths': deepCopy_paths,
                    'collisions': [],
                    'cost': 0
                }

                q_agent = constraint['agent']
                agentIDs = []
                if constraint['positive'] == True:
                    agentIDs = self.paths_violate_constraint(constraint, parent['paths'])
                if q_agent not in agentIDs:
                    agentIDs.append(q_agent)

                no_solution = False
                for ID in agentIDs:
                    q_path = sipp(self.my_map, self.starts[ID], self.goals[ID], self.heuristics[ID],
                                  ID, q['constraints'])
                    q_path = self.remove_extra_locations(q_path)
                    if q_path:
                        q['paths'][ID] = q_path
                    else:
                        no_solution = True
                        break

                if not no_solution:
                    q['collisions'] = detect_collisions(q['paths'])
                    q['cost'] = get_sum_of_cost(q['paths'])
                    self.push_node(q)

        raise BaseException('No solutions')

    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
