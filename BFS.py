import IDS


def evaluate_neighbour_2(state, cor):
    up, down, left, right = None, None, None, None
    if cor[0] - 2 >= 0:
        up = state[cor[0] - 1, cor[1]]
        double_up = state[cor[0] - 2, cor[1]]
        if (('x' in up) or ('b' in up) or ('p' in up)) and (('x' in double_up) or ('b' in double_up)):
            up = None

    if cor[0] + 2 < state.shape[0]:
        down = state[cor[0] + 1, cor[1]]
        double_down = state[cor[0] + 2, cor[1]]
        if (('x' in down) or ('b' in down) or ('p' in down)) and (('x' in double_down) or ('b' in double_down)):
            down = None

    if cor[1] - 2 >= 0:
        left = state[cor[0], cor[1] - 1]
        double_left = state[cor[0], cor[1] - 2]
        if (('x' in left) or ('b' in left) or ('p' in left)) and (('x' in double_left) or ('b' in double_left)):
            left = None

    if cor[1] + 2 < state.shape[1]:
        right = state[cor[0], cor[1] + 1]
        double_right = state[cor[0], cor[1] + 2]
        if (('x' in right) or ('b' in right) or ('p' in right)) and (('x' in double_right) or ('b' in double_right)):
            right = None

    return up, down, left, right


def butter_destination_predecessor(state, robot_cor, butter_cor, init_flag=False):
    result = []
    up, down, left, right = evaluate_neighbour_2(state, butter_cor)

    def new_state(a, b):
        n_state = state.copy()
        if init_flag:
            n_state[butter_cor[0], butter_cor[1]] = n_state[butter_cor[0], butter_cor[1]][:-1] + 'p'

        else:
            n_state[butter_cor[0], butter_cor[1]] = n_state[butter_cor[0], butter_cor[1]][:-1]
        n_state[butter_cor[0] + a, butter_cor[1] + b] += 'b'
        n_state[robot_cor[0], robot_cor[1]] = n_state[robot_cor[0], robot_cor[1]][:-1]
        n_state[butter_cor[0] + 2 * a, butter_cor[1] + 2 * b] += 'r'

        if not init_flag:
            dst_state = state.copy()
            dst_state[robot_cor[0], robot_cor[1]] = dst_state[robot_cor[0], robot_cor[1]][:-1]
            dst_state[butter_cor[0] + a, butter_cor[1] + b] += 'r'
            res, _, _, _ = IDS.ids(IDS.Node(state, None), [IDS.Node(dst_state, None)], IDS.robot_butter_successor,
                                   (state, robot_cor))
        else:
            res = list()

        return (n_state,), (butter_cor[0] + a, butter_cor[1] + b), (butter_cor[0], butter_cor[1]), \
               (IDS.evaluate_direction(a, b),), (res,)

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
