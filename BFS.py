import IDS
from ordered_set import OrderedSet
import numpy as np

def evaluate_double_neighbour(state, cor):
    up, down, left, right = None, None, None, None
    if cor[0] - 2 >= 0:
        up = state[cor[0] - 1, cor[1]]
        double_up = state[cor[0] - 2, cor[1]]
        if (('x' in up) or ('b' in up) or ('p' in up)) or (('x' in double_up) or ('b' in double_up)):
            up = None

    if cor[0] + 2 < state.shape[0]:
        down = state[cor[0] + 1, cor[1]]
        double_down = state[cor[0] + 2, cor[1]]
        if (('x' in down) or ('b' in down) or ('p' in down)) or (('x' in double_down) or ('b' in double_down)):
            down = None

    if cor[1] - 2 >= 0:
        left = state[cor[0], cor[1] - 1]
        double_left = state[cor[0], cor[1] - 2]
        if (('x' in left) or ('b' in left) or ('p' in left)) or (('x' in double_left) or ('b' in double_left)):
            left = None

    if cor[1] + 2 < state.shape[1]:
        right = state[cor[0], cor[1] + 1]
        double_right = state[cor[0], cor[1] + 2]
        if (('x' in right) or ('b' in right) or ('p' in right)) or (('x' in double_right) or ('b' in double_right)):
            right = None

    return up, down, left, right


def determine_reverse_direction(a, b):
    if a == 1:
        direction = 'U'
    elif a == -1:
        direction = 'D'
    else:
        if b == 1:
            direction = 'L'
        else:
            direction = 'R'

    return direction


def reverse_directions_in_robot_path(path):
    for i in range(len(path)):
        if path[i] == 'D':
            path[i] = 'U'
            continue
        elif path[i] == 'U':
            path[i] = 'D'
            continue
        else:
            if path[i] == 'R':
                path[i] = 'L'
                continue
            else:
                path[i] = 'R'


def butter_destination_successor(state, robot_cor, butter_cor):
    result = []
    up, down, left, right = IDS.evaluate_neighbour(state, butter_cor)

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

        res = bidirectional_bfs_robot(IDS.Node(state, None), butter_cor, robot_cor,
                                      (butter_cor[0] - a, butter_cor[1] - b), IDS.Node(dst_state, None))

        return (n_state,), (butter_cor[0] + a, butter_cor[1] + b), (butter_cor[0], butter_cor[1]), \
               (IDS.determine_direction(a, b),), (res,)

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


def robot_movement_predecessor(state, robot_cor, butter_cor=None):
    states = []
    up, down, left, right = IDS.evaluate_neighbour(state, robot_cor)

    def new_state(a, b):
        n_state = state.copy()
        n_state[robot_cor[0] + a, robot_cor[1] + b] += 'r'
        n_state[robot_cor[0], robot_cor[1]] = n_state[robot_cor[0], robot_cor[1]][:-1]
        return (n_state,), (None, None), (robot_cor[0] + a, robot_cor[1] + b), \
               (determine_reverse_direction(a, b),), (None,)

    if up is not None:
        states.append(new_state(-1, 0))

    if down is not None:
        states.append(new_state(1, 0))

    if left is not None:
        states.append(new_state(0, -1))

    if right is not None:
        states.append(new_state(0, 1))

    return states


def butter_destination_predecessor(state, robot_cor, butter_cor, init_flag=False):
    result = []
    up, down, left, right = evaluate_double_neighbour(state, butter_cor)

    def new_state(a, b):
        n_state = state.copy()
        if init_flag:
            n_state[butter_cor[0], butter_cor[1]] = n_state[butter_cor[0], butter_cor[1]][:-1] + 'p'

        else:
            n_state[butter_cor[0], butter_cor[1]] = n_state[butter_cor[0], butter_cor[1]][:-1]
        n_state[robot_cor[0], robot_cor[1]] = n_state[robot_cor[0], robot_cor[1]][:-1]
        n_state[butter_cor[0] + a, butter_cor[1] + b] += 'b'
        n_state[butter_cor[0] + 2 * a, butter_cor[1] + 2 * b] += 'r'

        if not init_flag:
            dst_state = state.copy()
            dst_state[robot_cor[0], robot_cor[1]] = dst_state[robot_cor[0], robot_cor[1]][:-1]
            dst_state[butter_cor[0] + a, butter_cor[1] + b] += 'r'
            res = bidirectional_bfs_robot(IDS.Node(state, None), butter_cor, robot_cor,
                                          (butter_cor[0] + a, butter_cor[1] + b), IDS.Node(dst_state, None), True)
            res.reverse()
            reverse_directions_in_robot_path(res)
        else:
            res = list()

        return (n_state,), (butter_cor[0] + a, butter_cor[1] + b), (butter_cor[0] + 2 * a, butter_cor[1] + 2 * b), \
               (determine_reverse_direction(a, b),), (res,)

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
    current_node = next(iter(fringe))
    visited.add(repr(current_node.get_state())[6:-15])
    del fringe[next(iter(fringe))]

    for data in successor(*successor_args):
        direction = ''
        if successor.__name__ == butter_destination_successor.__name__:
            # TODO: direction should not be not string
            if data[4][0] is not None:
                direction += str(data[4][0])

            direction += data[3][0]
        elif successor.__name__ == butter_destination_predecessor.__name__:
            direction += data[3][0]
            if data[4][0] is not None:
                direction += str(data[4][0])
        else:
            direction += data[3][0]


        new_node = IDS.Node(data[0][0], current_node, direction=direction)
        if not (repr(new_node.get_state())[6:-15] in visited):
            fringe[new_node] = data[2], data[1]


def extract_path(src_node, dst_node, flag=False):
    path = list()
    while src_node.get_direction() is not None:
        path.append(src_node.get_direction())
        src_node = src_node.get_parent()

    path.reverse()
    print(path)

    while dst_node.get_direction() is not None:

        path.append(dst_node.get_direction())
        dst_node = dst_node.get_parent()

    return path


def bidirectional_bfs_butter(init_node, init_butter_cor, init_robot_cor, target_cors):
    dst_fringe_lists = list()
    dst_visited_lists = list()
    final_nodes_lists = list()

    for target_cor in target_cors:
        final_nodes = IDS.goal_node_creator(init_node.get_state(), init_robot_cor, init_butter_cor,
                                            (target_cor,))

        final_nodes_lists.append(final_nodes)
        dst_fringes = list(dict() for _ in range(len(final_nodes)))
        dst_visited_lists.append(list(OrderedSet() for _ in range(len(final_nodes))))
        for k in range(len(final_nodes)):
            for i in range(final_nodes[k].get_state().shape[0]):
                for j in range(final_nodes[k].get_state().shape[1]):
                    if 'r' in final_nodes[k].get_state()[i, j]:
                        dst_fringes[k][final_nodes[k]] = (i, j), target_cor
        dst_fringe_lists.append(dst_fringes)
    src_fringe_list = {init_node: (init_robot_cor, init_butter_cor)}
    src_visited_list = OrderedSet()
    init = True

    while len(src_fringe_list) != 0 and sum(map(len, dst_fringe_lists)) != 0:


        current_src_fringe_list_size = len(src_fringe_list)
        src_fringe_list_copy = src_fringe_list.copy()
        for _ in range(current_src_fringe_list_size):
            src_node = next(iter(src_fringe_list))
            bfs(src_fringe_list, src_visited_list, butter_destination_successor,
                (src_node.get_state(), src_fringe_list[src_node][0], src_fringe_list[src_node][1]))

        for src in src_fringe_list:
            for i in range(len(dst_fringe_lists)):
                for j in range(len(dst_fringe_lists[i])):
                    for dst in dst_fringe_lists[i][j]:
                        if np.array_equal(src.get_state(), dst.get_state()):
                            path = extract_path(src, dst, True)
                            return target_cors[i], final_nodes_lists[i][j], path, len(path)


        for i in range(len(dst_fringe_lists)):
            for j in range(len(dst_fringe_lists[i])):

                current_dst_fringe_list_size = len(dst_fringe_lists[i][j])
                for _ in range(current_dst_fringe_list_size):
                    dst_node = next(iter(dst_fringe_lists[i][j]))
                    bfs(dst_fringe_lists[i][j], dst_visited_lists[i][j], butter_destination_predecessor,
                        (dst_node.get_state(), dst_fringe_lists[i][j][dst_node][0], dst_fringe_lists[i][j][dst_node][1],
                         init))

                for dst in dst_fringe_lists[i][j]:
                    for src in src_fringe_list:
                        if np.array_equal(src.get_state(), dst.get_state()):
                            path = extract_path(src, dst, True)
                            return target_cors[i], final_nodes_lists[i][j], path, len(path)
                    for src in src_fringe_list_copy:
                        if np.array_equal(src.get_state(), dst.get_state()):
                            path = extract_path(src, dst, True)
                            return target_cors[i], final_nodes_lists[i][j], path, len(path)


        init = False
    return None, None, None, None


def bidirectional_bfs_robot(init_node, init_butter_cor, init_robot_cor, final_robot_cor, target_node, flag=False):
    dst_fringe_list = {target_node: (final_robot_cor, init_butter_cor)}
    dst_visited_list = OrderedSet()

    src_fringe_list = {init_node: (init_robot_cor, init_butter_cor)}
    src_visited_list = OrderedSet()

    while len(src_fringe_list) != 0 and len(dst_fringe_list) != 0:

        for src in src_fringe_list:
            for dst in dst_fringe_list:
                if np.array_equal(src.get_state(), dst.get_state()):
                    path = extract_path(src, dst)
                    return path

        src_fringe_list_copy = src_fringe_list.copy()
        current_src_fringe_list_size = len(src_fringe_list)
        for _ in range(current_src_fringe_list_size):
            src_node = next(iter(src_fringe_list))
            bfs(src_fringe_list, src_visited_list, IDS.robot_butter_successor,
                (src_node.get_state(), src_fringe_list[src_node][0], src_fringe_list[src_node][1]))

        for src in src_fringe_list:
            for dst in dst_fringe_list:
                if np.array_equal(src.get_state(), dst.get_state()):
                    path = extract_path(src, dst)
                    return path

        current_dst_fringe_list_size = len(dst_fringe_list)
        for _ in range(current_dst_fringe_list_size):
            dst_node = next(iter(dst_fringe_list))
            bfs(dst_fringe_list, dst_visited_list, robot_movement_predecessor,
                (dst_node.get_state(), dst_fringe_list[dst_node][0], dst_fringe_list[dst_node][1]))

        for src in src_fringe_list_copy:
            for dst in dst_fringe_list:
                if np.array_equal(src.get_state(), dst.get_state()):
                    path = extract_path(src, dst)
                    return path
    return None


def permutation_of_butters(butter_cors, init_state, robot_cor, target_cors, result, step=0):
    if step == len(butter_cors):
        current_state = init_state
        total_path = []
        total_cost = 0
        counter = 0
        total_depth = 0

        for butter_cor in butter_cors:
            chosen_target, goal_node, path, cost = bidirectional_bfs_butter(IDS.Node(current_state, None),
                                                                            butter_cor,
                                                                            robot_cor, target_cors)
            if path is not None:
                current_state = goal_node.get_state()
                target_cors.remove(chosen_target)
                for i in range(current_state.shape[0]):
                    for j in range(current_state.shape[1]):
                        if 'r' in current_state[i, j]:
                            robot_cor = i, j
                total_path.extend(path)
                total_cost += cost
                counter += 1
                if cost > total_depth:
                    total_depth = cost

        result.append([total_path, total_cost, counter, total_depth])
    for i in range(step, len(butter_cors)):
        butter_cors_copy = butter_cors.copy()
        butter_cors_copy[i], butter_cors_copy[step] = butter_cors_copy[step], butter_cors_copy[i]
        permutation_of_butters(butter_cors_copy, init_state, robot_cor, target_cors, result, step + 1)


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

        li2 = ''
        for part in li:
            if part == 'L' or part == 'U' or part == 'D' or part == 'R':
                li2 += part + ' '

        print(li2)
        print(best_result[1])
        print(best_result[3])
    else:
        print('can’t pass {} butter{}'.format(num_of_butters - max_counter,
                                              's' if num_of_butters - max_counter > 1 else ''))

        li = []
        for part in best_result[0]:
            li.extend(part)

        li2 = ''
        for part in li:
            if part == 'L' or part == 'U' or part == 'D' or part == 'R':
                li2 += part + ' '

        print(li2)
        print(best_result[1])
        print(best_result[3])


def main():
    init_state = IDS.input_parser()

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

    print(final_result)



if __name__ == '__main__':
    main()