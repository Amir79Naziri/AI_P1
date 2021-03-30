import numpy as np
from ordered_set import OrderedSet


class Node:
    def __init__(self, state, parent):
        self.__state = state
        self.__parent = parent

    def get_state(self):
        return self.__state

    def get_parent(self):
        return self.__parent

    def __eq__(self, other):
        if isinstance(other, Node):
            return np.array_equal(self.__state, other.get_state())
        else:
            return False


def evaluate_neighbour(state, cor, target=False):
    up, down, left, right = None, None, None, None
    if cor[0] - 1 >= 0:
        up = state[cor[0] - 1, cor[1]]
        if ('x' in up) or ((not target) and 'b' in up):
            up = None

    if cor[0] + 1 < state.shape[0]:
        down = state[cor[0] + 1, cor[1]]
        if ('x' in down) or ((not target) and 'b' in down):
            down = None

    if cor[1] - 1 >= 0:
        left = state[cor[0], cor[1] - 1]
        if ('x' in left) or ((not target) and 'b' in left):
            left = None

    if cor[1] + 1 < state.shape[1]:
        right = state[cor[0], cor[1] + 1]
        if ('x' in right) or ((not target) and 'b' in right):
            right = None

    return up, down, left, right


def evaluate_direction(a, b):
    if a == 1:
        direction = 'D'
    elif a == -1:
        direction = 'U'
    else:
        if b == 1:
            direction = 'R'
        else:
            direction = 'L'

    return direction


def butter_destination_successor(state, robot_cor, butter_cor):
    result = []
    up, down, left, right = evaluate_neighbour(state, butter_cor)

    if up is None:
        down = None

    if down is None:
        up = None

    if left is None:
        right = None

    if right is None:
        left = None

    def new_state(a, b):
        n_state = state.copy()

        n_state[butter_cor[0], butter_cor[1]] = n_state[butter_cor[0], butter_cor[1]][:-1] + 'r'
        n_state[robot_cor[0], robot_cor[1]] = n_state[robot_cor[0], robot_cor[1]][:-1]
        if 'p' in n_state[butter_cor[0] + a, butter_cor[1] + b]:
            n_state[butter_cor[0] + a, butter_cor[1] + b] = n_state[butter_cor[0] + a, butter_cor[1] + b][:-1] + 'b'
        else:
            n_state[butter_cor[0] + a, butter_cor[1] + b] += 'b'

        dst_state = state.copy()
        dst_state[robot_cor[0], robot_cor[1]] = dst_state[robot_cor[0], robot_cor[1]][:-1]
        dst_state[butter_cor[0] - a, butter_cor[1] - b] += 'r'

        res, _, _, _, _ = ids(Node(state, None), [Node(dst_state, None)], robot_butter_successor, (state, robot_cor))

        return (n_state,), (butter_cor[0] + a, butter_cor[1] + b), (butter_cor[0], butter_cor[1]), \
               (evaluate_direction(a, b),), (res,)

    if up is not None:
        data = new_state(-1, 0)
        if data[4][0] is not None:
            result.append(data)

    if down is not None:
        data = new_state(1, 0)
        if data[4][0] is not None:
            result.append(data)

    if left is not None:
        data = new_state(0, -1)
        if data[4][0] is not None:
            result.append(data)

    if right is not None:
        data = new_state(0, 1)
        if data[4][0] is not None:
            result.append(data)

    return result


def robot_butter_successor(state, robot_cor, butter_cor=None):
    states = []
    up, down, left, right = evaluate_neighbour(state, robot_cor)

    def new_state(a, b):
        n_state = state.copy()
        n_state[robot_cor[0] + a, robot_cor[1] + b] += 'r'
        n_state[robot_cor[0], robot_cor[1]] = n_state[robot_cor[0], robot_cor[1]][:-1]
        return (n_state,), (None, None), (robot_cor[0] + a, robot_cor[1] + b), \
               (evaluate_direction(a, b),), (None,)

    if up is not None:
        states.append(new_state(-1, 0))

    if down is not None:
        states.append(new_state(1, 0))

    if left is not None:
        states.append(new_state(0, -1))

    if right is not None:
        states.append(new_state(0, 1))

    return states


def goal(src_node, dst_nodes):
    for dst_node in dst_nodes:
        if np.array_equal(src_node.get_state(), dst_node.get_state()):
            return True
    return False


def dls(node, dst_nodes, successor, successor_args, visited, limit, stack, cost=0):
    if goal(node, dst_nodes):
        return stack, node, cost, successor_args

    visited.add(repr(node.get_state())[6:-15])

    if limit <= 0:
        visited.pop()
        return None, None, None, None

    for data in successor(*successor_args):
        if data[4][0] is not None:
            stack.append(data[4][0])
            cost += len(data[4][0])
        stack.append(data[3][0])
        cost += 1
        cur_node = Node(data[0][0], node)
        cur_successor_args = data[0][0], data[2], data[1]
        if not (repr(cur_node.get_state())[6:-15] in visited):
            res, goal_node, new_cost, _ = dls(cur_node, dst_nodes, successor, cur_successor_args, visited, limit - 1, stack, cost)
            if res is not None:
                return res, goal_node, new_cost, successor_args
        del cur_node
        stack.pop()
        cost -= 1
        if data[4][0] is not None:
            stack.pop()
            cost -= len(data[4][0])
    visited.pop()
    return None, None, None, None


def ids(node, dst_nodes, successor, successor_args):
    for limit in range(1, node.get_state().shape[0] * node.get_state().shape[1]):
        res, goal_node, cost, final_successor_args = dls(node, dst_nodes, successor, successor_args, OrderedSet(), limit, [])
        if res is not None:
            return res, goal_node, cost, limit, final_successor_args
    return None, None, None, None, None


def goal_node_creator(initial_state, robot_cor, butter_cor, target_cors):
    def new_state(a, b):
        n_state = initial_state.copy()
        n_state[target_cor[0], target_cor[1]] = n_state[target_cor[0], target_cor[1]][:-1] + 'b'
        n_state[butter_cor[0], butter_cor[1]] = n_state[butter_cor[0], butter_cor[1]][:-1]
        n_state[target_cor[0] + a, target_cor[1] + b] += 'r'
        n_state[robot_cor[0], robot_cor[1]] = n_state[robot_cor[0], robot_cor[1]][:-1]
        return n_state

    goal_nodes = []

    for target_cor in target_cors:

        up, down, left, right = evaluate_neighbour(initial_state, target_cor, target=True)

        if up is not None:
            goal_nodes.append(Node(new_state(-1, 0), None))

        if down is not None:
            goal_nodes.append(Node(new_state(1, 0), None))

        if left is not None:
            goal_nodes.append(Node(new_state(0, -1), None))

        if right is not None:
            goal_nodes.append(Node(new_state(0, 1), None))

    return goal_nodes


def input_parser():
    row, col = input().split()

    data = []
    for i in range(int(row)):
        dummy = input().split()
        data.append(dummy)

    return np.array(data, dtype='object')


def permutation_of_butters(butter_cors, init_state, robot_cor, target_cors, step=0, result=None):
    if result is None:
        result = []
    if step == len(butter_cors):
        current_state = init_state
        total_path = []
        total_cost = 0
        counter = 0
        total_depth = 0
        for butter_cor in butter_cors:
            path, goal_node, cost, depth, final_successor_args = ids(Node(current_state, None), goal_node_creator(current_state, robot_cor, butter_cor, target_cors), butter_destination_successor, (current_state, robot_cor, butter_cor))
            if path is not None:
                current_state = goal_node.get_state()
                for i in range(current_state.shape[0]):
                    for j in range(current_state.shape[1]):
                        if 'r' in current_state[i, j]:
                            robot_cor = i, j
                total_path.extend(path)
                total_cost += cost
                counter += 1
                if depth > total_depth:
                    total_depth = depth

        result.append([total_path, total_cost, counter, total_depth])
    for i in range(step, len(butter_cors)):
        butter_cors_copy = butter_cors.copy()
        butter_cors_copy[i], butter_cors_copy[step] = butter_cors_copy[step], butter_cors_copy[i]
        permutation_of_butters(butter_cors_copy, init_state, robot_cor, target_cors, step + 1, result)


def main():
    init_state = input_parser()

    robot_cor = ()
    butter_cors = []
    target_cors = []

    for i in range(init_state.shape[0]):
        for j in range(init_state.shape[1]):
            if 'r' in init_state[i, j]:
                robot_cor = i, j
            elif 'b' in init_state[i, j]:
                butter_cor = i, j
                butter_cors.append(butter_cor)
            elif 'p' in init_state[i, j]:
                target_cor = i, j
                target_cors.append(target_cor)

    final_result = []
    permutation_of_butters(butter_cors, init_state, robot_cor, target_cors, result=final_result)
    print(final_result)

    # for i in robot_butter_successor(init_state, robot_cor, butter_cors[0]):
    #     print(i[0][0])

    # TODO fetch optimize result


if __name__ == '__main__':
    main()
