"""Testes sem ROS, usando a classe Robot original do simulador."""

from collections import deque
import copy
import csv
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

PACKAGE_DIR = Path(__file__).resolve().parents[1]
ROOT_DIR = PACKAGE_DIR.parent.parent
sys.path.insert(0, str(PACKAGE_DIR))
sys.path.insert(0, str(ROOT_DIR / 'culling_games/src/cg'))

from cg.Robot import Robot
from navegacao_autonoma.algorithms import MOVES, bfs, directions, grid_cells, marker
from navegacao_autonoma.solver import explore, load_json, part1, part2


class HeadlessMaze:
    def __init__(self, grid):
        self.grid = copy.deepcopy(grid)

    def get_occupancy_grid(self):
        return self.grid.copy()

    def get_cell(self, position):
        return self.grid[position[0]][position[1]]

    def set_cell(self, position, value):
        self.grid[position[0]][position[1]] = value


class OfflineRobot:
    """Mesmo movimento e sensores do Robot fornecido, sem janela ou servicos."""

    def __init__(self, grid, name='teste.csv'):
        self.initial_grid = copy.deepcopy(grid)
        self.name = name
        self.map_calls = 0
        self.forbid_map = False
        self.reset()

    def reset(self, map_name=''):
        if map_name and map_name != self.name:
            raise ValueError('Mapa nao encontrado.')
        self.maze = HeadlessMaze(self.initial_grid)
        self.robot = Robot(self.maze)
        self.start = self.robot.pos
        self.target = marker(grid_cells(self.initial_grid), 't')
        self.won = False
        return self.name

    def read_map(self):
        if self.forbid_map:
            raise AssertionError('A parte 2 consultou o mapa global!')
        self.map_calls += 1
        return copy.deepcopy(self.maze.grid)

    def sense(self):
        return self.robot.check_surroundings()

    def move(self, direction):
        if self.won:
            raise ValueError('O simulador ja terminou: reset necessario.')
        if not self.robot.move(direction):
            raise ValueError('Colisao com parede.')
        self.won = self.robot.pos == self.target
        return self.robot.pos


def grid(text):
    return [list(row) for row in text.split()]


def reference_distance(matrix):
    """Busca por camadas independente, usada apenas para verificar o custo."""
    cells = grid_cells(matrix)
    start, target = marker(cells, 'r'), marker(cells, 't')
    frontier = deque([(start, 0)])
    seen = {start}
    while frontier:
        (row, col), distance = frontier.popleft()
        if (row, col) == target:
            return distance
        for neighbor in ((row + 1, col), (row - 1, col), (row, col + 1), (row, col - 1)):
            if neighbor not in seen and cells.get(neighbor) in ('f', 'r', 't'):
                seen.add(neighbor)
                frontier.append((neighbor, distance + 1))
    return None


class SolutionTests(unittest.TestCase):
    def check_solution(self, matrix, name='teste.csv'):
        robot = OfflineRobot(matrix, name)
        with tempfile.TemporaryDirectory() as temp:
            output = Path(temp)
            first = part1(robot, output, name, log=lambda _: None)
            self.assertTrue(robot.won)
            self.assertEqual(first['passos'], reference_distance(matrix))
            self.assertEqual(robot.map_calls, 1)
            robot.forbid_map = True

            def checked_load(path):
                if Path(path).name == 'parte1.json':
                    self.assertTrue((output / 'mapa_descoberto.json').is_file())
                    self.assertEqual(robot.robot.pos, robot.start)
                    self.assertFalse(robot.won)
                return load_json(path)

            with patch('navegacao_autonoma.solver.load_json', side_effect=checked_load):
                second, comparison = part2(robot, output, name, log=lambda _: None)
            self.assertTrue(robot.won)
            self.assertEqual(robot.map_calls, 1)
            self.assertTrue(comparison['mapa_suficiente_para_reproduzir_rota'])
            self.assertTrue(comparison['rota_reproduzida_no_simulador'])
            self.assertEqual(first['passos'], second['passos'])
            self.assertEqual(first['comandos'], second['comandos'])
            self.assertEqual(second['comandos'], directions(second['rota']))
            relative_target = second['alvo']
            trace = load_json(output / 'exploracao.json')['trajeto']
            self.assertNotIn(relative_target, trace)
            self.assertEqual(trace[0], trace[-1])
            with (output / 'mapa_descoberto.csv').open(newline='') as stream:
                exported = list(csv.reader(stream))
            self.assertEqual(sum(row.count('r') for row in exported), 1)
            self.assertEqual(sum(row.count('t') for row in exported), 1)
            self.assertTrue(load_json(output / 'parte2.json')['execucao_concluida'])
            return first['passos'], second['movimentos_exploracao']

    def test_all_supplied_maps(self):
        paths = sorted((ROOT_DIR / 'culling_games/src/cg/maps').glob('*.csv'))
        self.assertTrue(paths)
        for path in paths:
            with self.subTest(map=path.name):
                with path.open(newline='') as stream:
                    matrix = list(csv.reader(stream))
                self.check_solution(matrix, path.name)

    def test_cycles_and_equal_cost_routes(self):
        self.check_solution(grid('''
            bbbbbbb
            bfffffb
            bfrbftb
            bfffffb
            bbbbbbb
        '''))

    def test_target_is_an_articulation(self):
        matrix = grid('''
            bbbbbbb
            brftffb
            bbbbbbb
        ''')
        robot = OfflineRobot(matrix)
        robot.forbid_map = True
        mapping = explore(robot, log=lambda _: None)
        self.assertEqual(mapping.visited, {(0, 0), (0, 1)})
        self.assertFalse(robot.won)
        self.check_solution(matrix)

    def test_adjacent_target(self):
        self.check_solution(grid('''
            bbbb
            brtb
            bbbb
        '''))

    def test_unreachable_target(self):
        matrix = grid('''
            bbbbb
            brbtb
            bbbbb
        ''')
        cells = grid_cells(matrix)
        with self.assertRaisesRegex(ValueError, 'Nao existe caminho'):
            bfs(cells, marker(cells, 'r'), marker(cells, 't'))
        with tempfile.TemporaryDirectory() as temp:
            with self.assertRaises(ValueError):
                part1(OfflineRobot(matrix), Path(temp), log=lambda _: None)
            self.assertFalse((Path(temp) / 'parte1.json').exists())

    def test_missing_reference(self):
        with tempfile.TemporaryDirectory() as temp:
            with self.assertRaisesRegex(ValueError, 'Execute a parte1'):
                part2(OfflineRobot(grid('bbbb brtb bbbb')), Path(temp), log=lambda _: None)

    def test_mismatched_reference_stops_replay(self):
        robot = OfflineRobot(grid('bbbbb brftb bbbbb'))
        with tempfile.TemporaryDirectory() as temp:
            output = Path(temp)
            first = part1(robot, output, log=lambda _: None)
            first['mapa'] = 'outro.csv'
            (output / 'parte1.json').write_text(json.dumps(first))
            with self.assertRaisesRegex(ValueError, 'comparacao falhou'):
                part2(robot, output, log=lambda _: None)
            self.assertFalse(robot.won)
            self.assertFalse((output / 'parte2.json').exists())
            self.assertFalse(load_json(output / 'comparacao.json')['rota_reproduzida_no_simulador'])

    def test_step_limit(self):
        robot = OfflineRobot(grid('bbbbbb brfftb bbbbbb'))
        with self.assertRaisesRegex(ValueError, 'Limite de passos'):
            explore(robot, max_steps=1, log=lambda _: None)

    def test_unknown_cells_are_not_traversable(self):
        with self.assertRaisesRegex(ValueError, 'Nao existe caminho'):
            bfs({(0, 0): 'r', (0, 2): 't'}, (0, 0), (0, 2))

    def test_bad_grid_and_markers(self):
        for matrix in ([], [['r'], ['f', 't']], [['r', 'x', 't']]):
            with self.subTest(matrix=matrix):
                with self.assertRaises(ValueError):
                    grid_cells(matrix)
        with self.assertRaises(ValueError):
            marker({(0, 0): 'r', (0, 1): 'r'}, 'r')

    def test_diagonal_move_is_rejected(self):
        with self.assertRaises(ValueError):
            directions([(0, 0), (1, 1)])

    def test_inconsistent_sensor_data_is_rejected(self):
        robot = OfflineRobot(grid('bbbbb brftb bbbbb'))
        from navegacao_autonoma.algorithms import Explorer
        explorer = Explorer()
        sensors = robot.sense()
        explorer.observe(sensors)
        sensors['right'] = 'b'
        with self.assertRaisesRegex(ValueError, 'Sensor inconsistente'):
            explorer.observe(sensors)


if __name__ == '__main__':
    unittest.main()
