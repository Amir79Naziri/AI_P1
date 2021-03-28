import numpy as np


def butter_destination_successor(state, butter_cor, robot_cor):
    states = []
    up, down, left, right = None, None, None, None
    if butter_cor[0] - 1 >= 0:
        up = state[butter_cor[0] - 1, butter_cor[1]]
        if up == 'x' or ('b' in up):
            up = None

    if butter_cor[0] + 1 < state.shape[0]:
        down = state[butter_cor[0] + 1, butter_cor[1]]
        if down == 'x' or ('b' in down):
            down = None

    if butter_cor[1] - 1 >= 0:
        left = state[butter_cor[0], butter_cor[1] - 1]
        if left == 'x' or ('b' in left):
            left = None

    if butter_cor[1] + 1 < state.shape[1]:
        right = state[butter_cor[0], butter_cor[1] + 1]
        if right == 'x' or ('b' in right):
            right = None

    if up is None:
        down = None

    if down is None:
        up = None

    if left is None:
        right = None

    if right is None:
        left = None

    # TODO : robot to dest

    if up is not None:
        # check robot_dest IDS
        if True:
            upState = state.copy()
            upState[butter_cor[0] - 1, butter_cor[1]] += 'b'
            upState[butter_cor[0], butter_cor[1]] = upState[butter_cor[0], butter_cor[1]][:-1]
            states.append(upState)

    if down is not None:
        # check robot_dest IDS
        if True:
            downState = state.copy()
            downState[butter_cor[0] + 1, butter_cor[1]] += 'b'
            downState[butter_cor[0], butter_cor[1]] = downState[butter_cor[0], butter_cor[1]][:-1]
            states.append(downState)

    if left is not None:
        # check robot_dest IDS
        if True:
            leftState = state.copy()
            leftState[butter_cor[0], butter_cor[1] - 1] += 'b'
            leftState[butter_cor[0], butter_cor[1]] = leftState[butter_cor[0], butter_cor[1]][:-1]
            states.append(leftState)

    if right is not None:
        # check robot_dest IDS
        if True:
            rightState = state.copy()
            rightState[butter_cor[0], butter_cor[1] + 1] += 'b'
            rightState[butter_cor[0], butter_cor[1]] = rightState[butter_cor[0], butter_cor[1]][:-1]
            states.append(rightState)

    return states


class Node:
    def __init__(self, state, parent):
        self.__state = state
        self.__parent = parent
        self.__expanded = False

    def getState(self):
        return self.__state

    def getParent(self):
        return self.__parent

    def isExpanded(self):
        return self.__expanded

    def expand(self):
        self.__expanded = True


def input_parser():
    row, col = input().split()

    data = []
    for i in range(int(row)):
        dummy = input().split()
        data.append(dummy)

    return np.array(data, dtype='str')


def main():
    fringe = []
    visited = {}


if __name__ == '__main__':
    main()
