import pygame
import numpy


class Robot:
    __robot_image = pygame.image.load('pic/robot-1.png')

    def __init__(self, x, y):
        self.x = x
        self.y = y

    def get_image(self):
        return self.__robot_image


robot = Robot(-1, -1)

butter_image = pygame.image.load('pic/butter-1.png')

obstacles = [pygame.image.load('pic/bread.png'), pygame.image.load('pic/coffee.png'),
             pygame.image.load('pic/english_breakfast.png'), pygame.image.load('pic/juice.png'),
             pygame.image.load('pic/milk.png')]


def update_state(state, move):
    global robot

    new_state = state.copy()
    new_state[robot.x, robot.y] = new_state[robot.x, robot.y][:-1]

    if move == 'U':
        if 'b' in state[robot.x, robot.y - 1]:

            new_state[robot.x, robot.y - 1] = new_state[robot.x, robot.y - 1][:-1]

            if 'p' in state[robot.x, robot.y - 2]:
                new_state[robot.x, robot.y - 2] = new_state[robot.x, robot.y - 2][:-1] + 'b'
            else:
                new_state[robot.x, robot.y - 2] += 'b'

        new_state[robot.x, robot.y - 1] += 'r'

    elif move == 'D':
        if 'b' in state[robot.x, robot.y + 1]:

            new_state[robot.x, robot.y + 1] = new_state[robot.x, robot.y + 1][:-1]

            if 'p' in state[robot.x, robot.y + 2]:
                new_state[robot.x, robot.y + 2] = new_state[robot.x, robot.y + 2][:-1] + 'b'
            else:
                new_state[robot.x, robot.y + 2] += 'b'

        new_state[robot.x, robot.y + 1] += 'r'

    elif move == 'L':
        if 'b' in state[robot.x - 1, robot.y]:

            new_state[robot.x - 1, robot.y] = new_state[robot.x - 1, robot.y][:-1]

            if 'p' in state[robot.x - 2, robot.y]:
                new_state[robot.x - 2, robot.y] = new_state[robot.x - 2, robot.y][:-1] + 'b'
            else:
                new_state[robot.x - 2, robot.y] += 'b'

        new_state[robot.x - 1, robot.y] += 'r'

    else:
        if 'b' in state[robot.x + 1, robot.y]:

            new_state[robot.x + 1, robot.y] = new_state[robot.x + 1, robot.y][:-1]

            if 'p' in state[robot.x + 2, robot.y]:
                new_state[robot.x + 2, robot.y] = new_state[robot.x + 2, robot.y][:-1] + 'b'
            else:
                new_state[robot.x + 2, robot.y] += 'b'

        new_state[robot.x + 1, robot.y] += 'r'

    return new_state


def redraw_window(window, state):
    window.fill((94, 61, 21))

    col_counter = 120
    row_counter = 120

    for i in range(window.get_size()[0] // 120 - 1):
        pygame.draw.line(window, (148, 101, 52), (0, col_counter), (window.get_size()[0], col_counter), width=1)
        col_counter += 120

    for j in range(window.get_size()[1] // 120 - 1):
        pygame.draw.line(window, (148, 101, 52), (row_counter, 0), (row_counter, window.get_size()[1]), width=1)
        row_counter += 120

    ## TODO : shape

    pygame.display.update()


def start(init_state, moves):
    global robot

    for i in range(init_state.shape[0]):
        for j in range(init_state.shape[1]):
            if 'r' in init_state[i, j]:
                robot.x = i
                robot.y = j
                break

    pygame.init()
    clock = pygame.time.Clock()

    window = pygame.display.set_mode((120 * init_state.shape[0], 120 * init_state.shape[1]))

    pygame.display.set_caption("What's my purpose ???")

    state = init_state
    while len(moves) > 0:
        clock.tick(27)

        state = update_state(state, moves.pop(0))

        redraw_window(window, state)

        for event1 in pygame.event.get():
            if event1.type == pygame.QUIT:
                pygame.quit()
                exit(0)

    pygame.quit()


if __name__ == '__main__':
    pass
