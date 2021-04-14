import numpy as np
import math
from ordered_set import OrderedSet


def manhattan_distance(src_cor, dst_cor):
    return math.fabs(src_cor[0] - dst_cor[0]) + math.fabs(src_cor[1] - dst_cor[1])


def butter_heuristic(butter_cor, dst_cors):
    li = []

    for dst_cor in dst_cors:
        li.append(manhattan_distance(butter_cor, dst_cor))

    return min(li)


def A_asterisk(init_node, init_butter_cor, init_robot_cor, dst_nodes, successor):
    visited = OrderedSet()
    fringe_list = dict()

    fringe_list[init_node] = (init_robot_cor, init_butter_cor)

    while len(fringe_list) > 0:
        minimum = np.inf
        minimum_key = None
        for key in fringe_list.keys():
            if minimum > key.get_cost():
                minimum = key.get_cost()
                minimum_key = key

        current_node = minimum_key
        current_robot_cor, current_butter_cor = fringe_list[minimum_key]
        del fringe_list[minimum_key]

        current_state = current_node.get_state()

        if goal(current_node, dst_nodes):
            return  # TODO : return final path

        for data in successor(current_state, current_robot_cor, current_butter_cor):
            path = ''
            if data[4][0] is not None:
                path += data[4][0]

            new_cost = evaluate_cost(data[0][0], data[1]) + current_node.get_cost()
            path += data[3][0]
            new_node = Node(data[0][0], current_node, new_cost, path)
            if not (repr(new_node.get_state())[6:-15] in visited):
                for node in fringe_list:
                    if node == new_node:
                        if node.get_cost() > new_node.get_cost():
                            del fringe_list[node]
                            fringe_list[new_node] = (data[1], data[2])
                else:
                    fringe_list[new_node] = (data[1], data[2])


class Node:
    def __init__(self, state, parent, cost=0, direction=None):
        self.__state = state
        self.__parent = parent
        self.__direction = direction
        self.__cost = cost

    def get_state(self):
        return self.__state

    def get_parent(self):
        return self.__parent

    def get_direction(self):
        return self.__direction

    def get_cost(self):
        return self.__cost

    def __eq__(self, other):
        if isinstance(other, Node):
            return np.array_equal(self.__state, other.get_state())
        else:
            return False

    def __hash__(self):
        return hash(repr(self.__state)[6:-15])


def determine_direction(a, b):
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


def evaluate_cost(state, cor):
    data = state[cor[0], cor[1]]

    for i in range(len(data), 0, -1):
        if data[0:i].isdigit():
            return int(data[0:i])


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
        res = None
        # TODO : configure return
        A_asterisk(Node(state, None), butter_cor, robot_cor, [Node(dst_state, None)], robot_butter_successor)

        return (n_state,), (butter_cor[0] + a, butter_cor[1] + b), (butter_cor[0], butter_cor[1]), \
               (determine_direction(a, b),), (res,)

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
               (determine_direction(a, b),), (None,)

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
            #TODO : configure result
            A_asterisk(Node(current_state, None), butter_cor, robot_cor,
                       goal_node_creator(current_state, robot_cor, butter_cor, target_cors),
                       butter_destination_successor)

            path, goal_node, cost, depth = None, None, None, None
            if path is not None:
                current_state = goal_node.get_state()
                for i in range(current_state.shape[0]):
                    for j in range(current_state.shape[1]):
                        if 'r' in current_state[i, j]:
                            robot_cor = i, j
                total_path.extend(path)
                total_cost += cost
                counter += 1
                total_depth = total_cost

        result.append([total_path, total_cost, counter, total_depth])
    for i in range(step, len(butter_cors)):
        butter_cors_copy = butter_cors.copy()
        butter_cors_copy[i], butter_cors_copy[step] = butter_cors_copy[step], butter_cors_copy[i]
        permutation_of_butters(butter_cors_copy, init_state, robot_cor, target_cors, step + 1, result)


def extract_result(final_result, num_of_butters):
    max_counter = max(i[2] for i in final_result)
    min_cost = np.Inf
    best_result = None

    for i in range(len(final_result)):
        if max_counter == final_result[i][2]:
            if final_result[i][1] < min_cost:
                min_cost = final_result[i][1]
                best_result = final_result[i]

    if max_counter == 0:
        print('can’t pass the butter')
    elif max_counter == num_of_butters:
        li = []
        for part in best_result[0]:
            li.extend(part)

        print(*li)
        print(best_result[1])
        print(best_result[3])
    else:
        print('can’t pass {} butter{}'.format(num_of_butters - max_counter,
                                              's' if num_of_butters - max_counter > 1 else ''))

        li = []
        for part in best_result[0]:
            li.extend(part)
        print(*li)
        print(best_result[1])
        print(best_result[3])


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
    extract_result(final_result, len(butter_cors))


if __name__ == '__main__':
    main()


