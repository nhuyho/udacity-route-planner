from math import sqrt


def get_distance(x1, y1, x2, y2):
    distance = sqrt((x1 - x2)**2 + (y1 - y2)**2)
    distance = distance * 1000
    return int(distance)


def shortest_path(M, start, goal):

    if start == goal:
        return [start]

    # initialize h and calculate heuristic values for h dictionary
    h_dict = dict()
    for node, coordinates in M.intersections.items():
        h_dict[node] = get_distance(coordinates[0], coordinates[1], M.intersections[
                                    goal][0], M.intersections[goal][1])

    # initialize f dict
    f_dict = dict()
    # initialize distance from start
    distance_from_start = {start: 0}
    # initialize parent dict
    parent = dict()
    # previous vertex dict
    previous = dict()

    # initialize open and closed lists
    open_ = {start}
    closed = set()

    # make the start vertex current
    current = start

    # calculate heuristic dixtance of start vertex to destination (h)
    h = h_dict[current]

    # calculate f value for start vertex (f = g + h, where g = 0)
    f_dict[current] = 0 + h

    # while current vertex is not the destination
    while current != goal:

        # for each vertex adjacent to current
        for vertex in M.roads[current]:

            if vertex in closed:
                continue
            # calculate distance from start (g)
            g = distance_from_start[current] + get_distance(M.intersections[current][0], M.intersections[
                                                            current][1], M.intersections[vertex][0], M.intersections[vertex][1])
            # calculate heuristic distance to destination (h)
            h = h_dict[vertex]
            # calculate f value (f = g + h)
            f = g + h

            # if vertex not in open list or f value < existing f value
            if vertex not in open_ or f < f_dict[vertex]:
                parent[vertex] = current
                f_dict[vertex] = f
                distance_from_start[vertex] = g

                if vertex not in open_:
                    open_.add(vertex)

        # add current vertex to closed list
        closed.add(current)
        # remove current vertex from open list
        open_.remove(current)
        # remove vertex with lowest f value from open list and make it current
        del f_dict[current]

        # select the vertex with lowest f value and make it current
        lowest_vertex = min(f_dict.items(), key=lambda x: x[1])[0]
        current = lowest_vertex

    path = [goal]
    come_from = parent[goal]
    path.append(come_from)

    while come_from != start:
        come_from = parent[come_from]
        path.append(come_from)

    return path[::-1]
