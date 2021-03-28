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


def evaluate_direction(a, b):
    direction = ""
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
        n_state[butter_cor[0] + a, butter_cor[1] + b] += 'b'
        n_state[butter_cor[0], butter_cor[1]] = n_state[butter_cor[0], butter_cor[1]][:-1] + 'r'
        n_state[robot_cor[0], robot_cor[1]] = n_state[robot_cor[0], robot_cor[1]][:-1]

        dst_state = state.copy()
        dst_state[robot_cor[0], robot_cor[1]] = dst_state[robot_cor[0], robot_cor[1]][:-1]
        dst_state[butter_cor[0] - a, butter_cor[1] - b] += 'r'

        res = ids(Node(state, None), [Node(dst_state, None)], robot_butter_successor, (state, robot_cor))

        return (n_state,), (butter_cor[0] + a, butter_cor[1] + b), (butter_cor[0], butter_cor[1]), \
               (evaluate_direction(a, b),), (res, )

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


def dls(node, dst_nodes, successor, successor_args, visited, limit, stack):
    if goal(node, dst_nodes):
        return stack

    visited.add(repr(node.get_state())[6:-15])

    if limit <= 0:
        return None

    for data in successor(successor_args):

        stack.append(data[4][0])
        stack.append(data[3][0])
        cur_node = Node(data[0][0], node)
        cur_successor_args = data[0][0], data[2], data[1]
        if not (repr(cur_node.get_state())[6:-15] in visited):
            res = dls(cur_node, dst_nodes, successor, cur_successor_args, visited, limit - 1, stack)
            if res is not None:
                return res
        del cur_node
        stack.pop()
        stack.pop()
    return None


def ids(node, dst_nodes, successor, successor_args):
    visited = set()
    for limit in range(1, node.get_state().shape[0] * node.get_state().shape[1]):
        res = dls(node, dst_nodes, successor, successor_args, visited, limit, [])
        if res is not None:
            return res
    return None


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
    for i in range(init_state.shape[0]):
        for j in range(init_state.shape[1]):
            if 'r' in init_state[i, j]:
                robot_cor = i, j
            elif 'b' in init_state[i, j]:
                butter_cor = i, j

    print(ids())


if __name__ == '__main__':
    main()
