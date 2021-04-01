import IDS
from queue import Queue

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


def bfs(fringe, visited, successor, successor_args):
    current_node = fringe[next(iter(fringe))]
    del fringe[next(iter(fringe))]
    for data in successor(*successor_args):
        new_node = IDS.Node(data[0][0], current_node)
        if not (repr(new_node.get_state())[6:-15] in visited):
            fringe[new_node] = data[2], data[1]


def bidirectional_bfs(init_node, init_butter_cor, init_robot_cor, target_cors):
    dst_fringes_list = list()
    for target_cor in target_cors:
        final_nodes = IDS.goal_node_creator(init_node.get_state(), init_robot_cor, init_butter_cor, (target_cor, ))
        dst_fringes = list(dict() for _ in range(len(final_nodes)))
        for k in range(len(final_nodes)):
            for i in range(final_nodes[k].get_state().shape[0]):
                for j in range(final_nodes[k].get_state().shape[1]):
                    if 'r' in final_nodes[k].get_state()[i, j]:
                        dst_fringes[k][final_nodes[k]] = (i, j), target_cor
        dst_fringes_list.extend(dst_fringes)
    src_fringe = dict()


