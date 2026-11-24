"""Busca em largura e exploracao em profundidade em um grid de custo unitario."""

from collections import deque


# A mesma ordem nas duas etapas resolve empates entre caminhos minimos.
MOVES = {
    'up': (-1, 0),
    'right': (0, 1),
    'down': (1, 0),
    'left': (0, -1),
}
SENSORS = {
    **MOVES,
    'up_left': (-1, -1),
    'up_right': (-1, 1),
    'down_left': (1, -1),
    'down_right': (1, 1),
}
WALKABLE = frozenset(('f', 'r', 't'))


def add(position, delta):
    return position[0] + delta[0], position[1] + delta[1]


def grid_cells(grid):
    if not grid or not grid[0] or any(len(row) != len(grid[0]) for row in grid):
        raise ValueError('O mapa deve ser uma matriz retangular nao vazia.')
    cells = {}
    for row_index, row in enumerate(grid):
        for col_index, value in enumerate(row):
            if value not in WALKABLE | {'b'}:
                raise ValueError(f'Celula invalida: {value!r}.')
            cells[row_index, col_index] = value
    return cells


def marker(cells, value):
    matches = [position for position, cell in cells.items() if cell == value]
    if len(matches) != 1:
        raise ValueError(f'Esperava exatamente uma celula {value!r}; encontrei {len(matches)}.')
    return matches[0]


def bfs(cells, start, target):
    """Retorna o caminho minimo, incluindo origem e alvo; desconhecido e bloqueado."""
    if cells.get(start) not in WALKABLE or cells.get(target) not in WALKABLE:
        raise ValueError('Origem ou alvo nao sao celulas livres conhecidas.')
    queue = deque([start])
    parents = {start: None}
    while queue:
        current = queue.popleft()
        if current == target:
            route = []
            while current is not None:
                route.append(current)
                current = parents[current]
            return list(reversed(route))
        for delta in MOVES.values():
            neighbor = add(current, delta)
            if neighbor not in parents and cells.get(neighbor) in WALKABLE:
                parents[neighbor] = current
                queue.append(neighbor)
    raise ValueError('Nao existe caminho conhecido entre a origem e o alvo.')


def directions(route):
    reverse_moves = {delta: direction for direction, delta in MOVES.items()}
    result = []
    for current, neighbor in zip(route, route[1:]):
        delta = neighbor[0] - current[0], neighbor[1] - current[1]
        if delta not in reverse_moves:
            raise ValueError('A rota contem um movimento diferente de um passo cardinal.')
        result.append(reverse_moves[delta])
    return result


class Explorer:
    """DFS fisica, com retorno pelos corredores e coordenadas relativas a origem.

    O alvo e observado, mas nunca atravessado durante a exploracao. Assim, o
    simulador nao entra no estado de vitoria antes de terminarmos o mapa.
    """

    def __init__(self):
        self.position = (0, 0)
        self.cells = {self.position: 'f'}
        self.visited = {self.position}
        self.stack = [self.position]
        self.target = None
        self.trace = [self.position]

    def observe(self, surroundings):
        for sensor, delta in SENSORS.items():
            value = surroundings[sensor]
            if value not in WALKABLE | {'b'}:
                raise ValueError(f'Sensor {sensor} retornou valor invalido: {value!r}.')
            value = 'f' if value == 'r' else value
            position = add(self.position, delta)
            known = self.cells.get(position)
            if known is not None and known != value:
                raise ValueError(
                    f'Sensor inconsistente em {position}: {known!r} virou {value!r}. '
                    'Confira se outro controlador esta movendo o robo. '
                    'Se necessario, aumente --intervalo.'
                )
            self.cells[position] = value
            if value == 't':
                if self.target is not None and self.target != position:
                    raise ValueError('Os sensores indicaram mais de um alvo.')
                self.target = position

    def next_direction(self):
        for direction, delta in MOVES.items():
            neighbor = add(self.position, delta)
            if neighbor not in self.visited and self.cells.get(neighbor) == 'f':
                return direction
        if len(self.stack) > 1:
            return directions([self.position, self.stack[-2]])[0]
        return None

    def confirm_move(self, direction):
        neighbor = add(self.position, MOVES[direction])
        if self.cells.get(neighbor) != 'f':
            raise ValueError('A exploracao tentou entrar em uma parede, no desconhecido ou no alvo.')
        if neighbor not in self.visited:
            self.visited.add(neighbor)
            self.stack.append(neighbor)
        elif len(self.stack) > 1 and neighbor == self.stack[-2]:
            self.stack.pop()
        else:
            raise ValueError('Retorno DFS inconsistente.')
        self.position = neighbor
        self.trace.append(neighbor)
