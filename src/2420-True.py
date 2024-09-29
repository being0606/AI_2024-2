# 전략 : A* 알고리즘을 사용하여 풀이
import heapq
from math import sqrt

start = '806524731'
goal = '123456780'

n = int(sqrt(len(goal)))  # 보드 크기

class Node:
    def __init__(self, state, path, cost):
        self.state = state
        self.path = path
        self.cost = cost  # f(n) = g(n) + h(n)

    def __lt__(self, other):
        return self.cost < other.cost

def print_puzzle(state):
    for i in range(n):
        print(state[n*i:n*(i+1)])
    print()

def manhattan_distance(state):
    distance = 0
    for idx, value in enumerate(state):
        if value != '0':
            target = goal.index(value)
            current_row, current_col = divmod(idx, n)
            target_row, target_col = divmod(target, n)
            distance += abs(current_row - target_row) + abs(current_col - target_col)
    return distance

def move(node):
    p = node.state
    i = p.index('0')  # 빈 칸 위치
    children = []
    moves = []
    directions = []
    
    if i % n > 0:
        moves.append(i - 1)
        directions.append('L')
    if i % n < n - 1:
        moves.append(i + 1)
        directions.append('R')
    if i >= n:
        moves.append(i - n)
        directions.append('U')
    if i < n * n - n:
        moves.append(i + n)
        directions.append('D')
    
    for move_pos, direction in zip(moves, directions):
        lst = list(p)
        lst[i], lst[move_pos] = lst[move_pos], lst[i]
        new_state = ''.join(lst)
        children.append((new_state, node.path + direction))
    
    return children

def a_star(start, goal):
    open_list = []
    heapq.heappush(open_list, Node(start, '', manhattan_distance(start)))
    closed_set = set()
    nodes_explored = 0

    while open_list:
        current_node = heapq.heappop(open_list)
        nodes_explored += 1

        if current_node.state == goal:
            return current_node.path, nodes_explored, True

        closed_set.add(current_node.state)

        for child_state, move_direction in move(current_node):
            if child_state not in closed_set:
                g = len(move_direction)
                h = manhattan_distance(child_state)
                f = g + h
                heapq.heappush(open_list, Node(child_state, move_direction, f))

    return '', nodes_explored, False

if __name__ == "__main__":
    print("초기 상태:")
    print_puzzle(start)
    
    path, nodes, found = a_star(start, goal)
    
    print(f"{nodes}개 노드를 살피고 찾았다!")
    print(f"경로: {path} ({len(path)} 단계)")
    
    filename = f"{nodes}-{found}.py"
    print(f"파일명: {filename}")