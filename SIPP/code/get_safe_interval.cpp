# calculate safe interval
def get_safe_interval(current_location, next_location, timestep, constraint_table):
    safe_interval = []
    constraint_set = []
    # infinite_constraint = []
    last_time = None

    for time in constraint_table:
        for constraint in constraints[time]:
            # detect which type of constraint it is and then treat the constraints differently according to the type
            if len(constraint['loc']) == 2: 
 		if current_location == constraint['loc'][0] and next_location == \
		constraint['loc'][1] and constraint['timestep'] not in constraint_set:
                    constraint_set.append(constraint['timestep']-1)
                    constraint_set.append(constraint['timestep'])
            else:
                if constraint['loc'][0] == next_location:
                    if time = -1:
			last_time = constraint['timestep'][0] 
			constraint_set.append(constraint['timestep'])                       
                    else:
                        # infinite_constraint.append(constraint['timestep'])
			if constraint['timestep'] not in constraint_set:
                            constraint_set.append(constraint['timestep'])
                        
    # constraint_set += infinite_constraint

    # no constraint 
    if len(constraint_set) == 0:
        return [(timestep, -1)]

    constraint_set.sort()

    for i in range(len(constraint_set)):
        # first meet point not no start loc, and time not past
        #if i == 0 and type(constraint_set[i-1]) == type(1) and constraint_set[i] != 0 and constraint_set[i] > timestep:
        #    safe_interval.append((timestep, constraint_set[i]-1))

        # not the first meet point and obstacle not stay in meet point
        if i != 0 and type(constraint_set[i-1]) == type(1) and constraint_set[i-1]+1 != constraint_set[i]:
            # time not past
            if timestep < constraint_set[i]:
                safe_interval.append((constraint_set[i-1]+1, constraint_set[i]-1))
            elif constraint_set[i-1]+1 <= timestep and timestep <= constraint_set[i]-1:
		safe_interval.append((timestep, constraint_set[i]-1))
        if i == len(constraint_set)-1 and constraint_set[i] != last_time:
            if timestep > constraint_set[i]+1:
                safe_interval.append((timestep, -1))
            else:
                safe_interval.append((constraint_set[i]+1, -1))

    safe_interval.sort(key=lambda x:x[0])

    return safe_interval

