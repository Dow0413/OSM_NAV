"""Dijkstra shortest-path implementation for weighted road topologies."""

import heapq


def shortest_path(graph, start, goal):
    """Return ``(cost, node_path)`` for the least-cost path in ``graph``.

    ``graph`` maps a node ID to ``(neighbour_id, non_negative_cost)`` pairs.
    A ``ValueError`` is raised when the two nodes are disconnected.
    """
    costs = {start: 0.0}
    previous = {}
    queue = [(0.0, start)]
    while queue:
        cost, current = heapq.heappop(queue)
        if cost != costs[current]:
            continue
        if current == goal:
            break
        for neighbor, edge_cost in graph.get(current, []):
            candidate = cost + edge_cost
            if candidate < costs.get(neighbor, float("inf")):
                costs[neighbor] = candidate
                previous[neighbor] = current
                heapq.heappush(queue, (candidate, neighbor))
    if goal not in costs:
        raise ValueError("The selected start and goal are not connected in the walkable robot-dog topology.")
    path = [goal]
    while path[-1] != start:
        path.append(previous[path[-1]])
    path.reverse()
    return costs[goal], path
