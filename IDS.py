import numpy as np


def evaluate_neighbour(state, cor):
    up, down, left, right = None, None, None, None
    if cor[0] - 1 >= 0:
        up = state[cor[0] - 1, cor[1]]
        if up == 'x' or ('b' in up):
            up = None

    if cor[0] + 1 < state.shape[0]:
        down = state[cor[0] + 1, cor[1]]
        if down == 'x' or ('b' in down):
            down = None

    if cor[1] - 1 >= 0:
        left = state[cor[0], cor[1] - 1]
        if left == 'x' or ('b' in left):
            left = None

    if cor[1] + 1 < state.shape[1]:
        right = state[cor[0], cor[1] + 1]
        if right == 'x' or ('b' in right):
            right = None

    return up, down, left, right


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

    # TODO : robot to dest

    def new_state(a, b):
        n_state = state.copy()
        n_state[butter_cor[0] + a, butter_cor[1] + b] += 'b'
        n_state[butter_cor[0], butter_cor[1]] = n_state[butter_cor[0], butter_cor[1]][:-1] + 'r'
        n_state[robot_cor[0], robot_cor[1]] = n_state[robot_cor[0], robot_cor[1]][:-1]
        return (n_state,), (butter_cor[0] + a, butter_cor[1] + b), (butter_cor[0], butter_cor[1])

    if up is not None:
        # check robot_dest IDS
        if True:
            result.append(new_state(-1, 0))

    if down is not None:
        # check robot_dest IDS
        if True:
            result.append(new_state(1, 0))

    if left is not None:
        # check robot_dest IDS
        if True:
            result.append(new_state(0, -1))

    if right is not None:
        # check robot_dest IDS
        if True:
            result.append(new_state(0, 1))

    return result


def robot_butter_successor(state, robot_cor, butter_cor=None):
    states = []
    up, down, left, right = evaluate_neighbour(state, robot_cor)

    def new_state(a, b):
        n_state = state.copy()
        n_state[robot_cor[0] + a, robot_cor[1] + b] += 'r'
        n_state[robot_cor[0], robot_cor[1]] = n_state[robot_cor[0], robot_cor[1]][:-1]
        return (n_state,), (None, None), (robot_cor[0] + a, robot_cor[1] + b)

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


def dls(node, dst_nodes, successor, successor_args, visited, limit):
    if goal(node, dst_nodes):
        pass
        # TODO path

    # visited[] = node
    if limit <= 0:
        return None

    for data in successor(successor_args):
        cur_node = Node(data[0][0], node)
        cur_successor_args = data[0][0], data[2], data[1]
        if cur_node not in visited:
            res = dls(cur_node, dst_nodes, successor, cur_successor_args, visited, limit - 1)
            if res is not None:
                return res
    return None


def ids(node, dst_node, successor, successor_args):
    visited = {}
    for limit in range(1, node.get_state().shape[0] * node.get_state().shape[1]):
        res = dls(node, dst_node, successor, successor_args, visited, limit)
        if res is not None:
            return res
    return None


class Node:
    def __init__(self, state, parent):
        self.__state = state
        self.__parent = parent
        self.__expanded = False

    def get_state(self):
        return self.__state

    def get_parent(self):
        return self.__parent

    def is_expanded(self):
        return self.__expanded

    def expand(self):
        self.__expanded = True

    def __eq__(self, other):
        if isinstance(other, Node):
            return np.array_equal(self, other)
        else:
            return False

    def __hash__(self):
        return hash(self.__state)


def input_parser():
    row, col = input().split()

    data = []
    for i in range(int(row)):
        dummy = input().split()
        data.append(dummy)

    return np.array(data, dtype='str')


def main():
    init_state = input_parser()
    robot_cor = ()
    butter_cor = ()
    print(init_state)
    print("\n-------")
    # for i in range(init_state.shape[0]):
    #     for j in range(init_state.shape[1]):
    #         if 'r' in init_state[i, j]:
    #             robot_cor = i, j
    #         elif 'b' in init_state[i, j]:
    #             butter_cor = i, j
    #
    # states = butter_destination_successor(init_state, butter_cor, robot_cor)
    # for state in states:
    #     print(state)
    #     print("\n------")


if __name__ == '__main__':
    main()
