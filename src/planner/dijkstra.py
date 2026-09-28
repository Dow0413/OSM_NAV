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
        raise ValueError("The selected start and goal are not connected in the active topology.")
    path = [goal]
    while path[-1] != start:
        path.append(previous[path[-1]])
    path.reverse()
    return costs[goal], path


def shortest_path_with_turn_restrictions(transitions, start, goal, turn_allowed):
    """Dijkstra over ``(node, incoming_way)`` states.

    ``transitions[node]`` contains ``(next_node, cost, way_id)`` entries.
    Keeping the incoming way in the state is essential: a turn restriction is
    a property of ``from way -> via node -> to way``, not of the via node
    alone.  ``start`` and ``goal`` are virtual node IDs used by snapping.
    """
    costs = {start: 0.0}
    previous = {}
    queue = [(0.0, 0, start)]
    sequence = 0
    while queue:
        cost, _, current = heapq.heappop(queue)
        if cost != costs[current]:
            continue
        if current == goal:
            break
        if current == start:
            node_id, incoming_way = start, None
        else:
            node_id, incoming_way = current
        for next_node, edge_cost, outgoing_way in transitions.get(node_id, []):
            if not turn_allowed(incoming_way, node_id, outgoing_way):
                continue
            next_state = goal if next_node == goal else (next_node, outgoing_way)
            candidate = cost + edge_cost
            if candidate < costs.get(next_state, float("inf")):
                costs[next_state] = candidate
                previous[next_state] = current
                sequence += 1
                heapq.heappush(queue, (candidate, sequence, next_state))
    if goal not in costs:
        raise ValueError("The selected start and goal are not connected under the active topology and turn restrictions.")
    path = [goal]
    while path[-1] != start:
        path.append(previous[path[-1]])
    path.reverse()
    return costs[goal], path
