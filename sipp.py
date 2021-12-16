import copy
import heapq
from collections import defaultdict

infinity = -1


def move(loc, dir):  # loc[0] means the first valor of the tuplet and if dir = 3 then (-1, 0 )
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]  # added wait direction (0,0)
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst


def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(4):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
                    or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
                continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values


def build_constraint_table(constraints, agent):
    ##############################
    # Task 1.2/1.3: Return a table that contains the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the
    #               is_constrained function.
    # build constraints only for that specific agent in the func parameter
    table = defaultdict(list)
    allConstraints = copy.deepcopy(constraints)  # modified constraints for disjoint splitting

    for constraint in allConstraints:
        if constraint['positive'] == 1 and constraint['agent'] != agent:
            temp = {
                'agent': agent,
                'loc': constraint['loc'],
                'timestep': constraint['timestep'],
                'positive': 0
            }
            if len(constraint['loc']) == 2:
                temp['loc'] = [constraint['loc'][1], constraint['loc'][0]]
            allConstraints.append(temp)

    for constraint in allConstraints:
        if constraint['agent'] == agent:
            timestep = constraint['timestep']
            if timestep not in table.keys():
                table[timestep] = []
            table[timestep].append(constraint)

    return table

def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    ##############################
    # Task 1.2/1.3: Check if a move from curr_loc to next_loc at time step next_time violates
    #               any given constraint. For efficiency the constraints are indexed in a constraint_table
    #               by time step, see build_constraint_table. TEST
    flag = False
    for key, value in constraint_table.items():
        if key == next_time:
            for index in range(len(value)):  # loop through all the dict's with key == next_time
                # negative constraint
                if value[index]['positive'] == 0:
                    if len(value[index]['loc']) == 1:
                        # vertex collision
                        if value[index]['loc'] == [next_loc]:
                            return True
                    else:
                        # edge collision
                        if value[index]['loc'] == [curr_loc, next_loc]:
                            return True
                # positive constraint
                else:
                    if len(value[index]['loc']) == 1:
                        # vertex collision
                        if value[index]['loc'] == [next_loc]:
                            continue
                        else:
                            return True
                    else:
                        # edge collision
                        if value[index]['loc'] == [curr_loc, next_loc]:
                            continue
                        else:
                            return True
    return flag


def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path


def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def earliest_time_possible(safe_interval, curr_time):
    # check if safe interval is not past yet
    if curr_time > safe_interval[1] or safe_interval[1] == -1:
        return None
    # check how long the agent needs to wait
    wait_time = safe_interval[0] - curr_time
    if wait_time < 0:
        return -1  # bit unsure about this
    else:
        return wait_time


def get_safe_intervals(current_loc, cfg, curr_timestep, constraint_table):
    # need to add code here
    safe_intervals = [0, 0]
    return safe_intervals


def get_successors(my_map, h_values, constraint_table, curr):
    s_prime = []
    # loop through all the available motions except for wait.
    for motion in range(4):
        cfg = move(curr['loc'], motion)
        # check for boundary constraints
        if cfg[0] < 0 or cfg[0] >= len(my_map) or cfg[1] < 0 or cfg[1] >= len(my_map):
            continue
        # check if the config is available in my map if not then continue
        if my_map[cfg[0]][cfg[1]]:
            continue
        m_time = 1
        start_time = curr['timestep'] + m_time
        # set it to the last safe interval available.
        end_time = curr['safe_interval'][1]
        if end_time >= 0:
            end_time += m_time
        # calculate all safe intervals for the current cfg (configuration)
        # need to modify the next line
        safe_intervals_cfg = get_safe_intervals(curr['loc'], cfg, curr['timestep'] + 1, constraint_table)
        for safe_interval in safe_intervals_cfg:
            if (safe_interval[0] > end_time and end_time != infinity) or (safe_interval[1] < start_time and safe_interval[1] != infinity):
                continue
            earliest_time = earliest_time_possible(safe_interval, curr['timestep'])
            if earliest_time < 0:
                continue
            temp = {'loc': cfg, 'g_val': curr['g_val']+earliest_time, 'h_val': h_values[cfg], 'parent': curr,
            'timestep': curr['timestep']+earliest_time, 'safe_interval': safe_interval, 'wait_time': 0}

             # substract the time it takes to get there
            if earliest_time != 0:
                temp['wait_time'] += (earliest_time - m_time)
            s_prime.append(temp)

    return s_prime







def sipp(my_map, start_loc, goal_loc, h_values, agent, constraints):
    open_list = []
    closed_list = dict()

    # build constraint table
    defaultDict_constraint_table = build_constraint_table(constraints, agent)  # constraint table
    constraint_table = dict(defaultDict_constraint_table)

    earliest_goal_timestep = 0
    if len(constraint_table.keys()) != 0:
        earliest_goal_timestep = max(constraint_table.keys())

    if start_loc not in h_values.keys():
        return None
    h_value = h_values[start_loc]

    # how long it can wait in the root node
    root_wait_time = get_safe_intervals(start_loc, start_loc, 0, constraint_table)[0]
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'timestep': 0, 'safe_interval': root_wait_time, 'wait_time': 0}

    push_node(open_list, root)

    closed_list[(root['loc']), root_wait_time] = root
######################STOPPED HERE############################
    upper_bound = (len(max(my_map)) * len(my_map)) + 1

    while len(open_list) > 0:

        curr = pop_node(open_list)

        if curr['timestep'] > upper_bound:
            print("Hit upper bound:", upper_bound,
                  ", Shortest Path cannot be greater than the distance of the entire map! ")
            return None  # Failed to find solutions
        if curr['loc'] == goal_loc and curr['timestep'] >= earliest_goal_timestep:
            return get_path(curr)
        for dir in range(5):
            child_loc = move(curr['loc'], dir)
            # must add boundary condition for negative values
            if child_loc[0] < 0 or child_loc[1] < 0 or child_loc[0] >= len(my_map) or child_loc[1] >= len(my_map[0]):
                continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc,
                     'g_val': curr['g_val'] + 1,
                     'h_val': h_values[child_loc],
                     'parent': curr,
                     'timestep': curr['timestep'] + 1}  # parent+1

            if is_constrained(curr['loc'], child['loc'], child['timestep'], constraint_table):
                continue

            if (child['loc'], child['timestep']) in closed_list:
                existing_node = closed_list[(child['loc']), child['timestep']]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc']), child['timestep']] = child
                    push_node(open_list, child)
            else:
                closed_list[(child['loc']), child['timestep']] = child
                push_node(open_list, child)

    return None  # Failed to find solutions
