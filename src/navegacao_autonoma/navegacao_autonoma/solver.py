"""Execucao das etapas e evidencias; nao depende do ROS ou do Pygame."""

import csv
import json
from pathlib import Path

from .algorithms import Explorer, MOVES, add, bfs, directions, grid_cells, marker


def save_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


def load_json(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def json_route(route):
    return [list(position) for position in route]


def walk(robot, route, initial_absolute=None):
    """Confere o deslocamento retornado pelo servico; nao usa target_pos."""
    previous_absolute = initial_absolute
    for direction in directions(route):
        actual = tuple(robot.move(direction))
        if previous_absolute is not None and actual != add(previous_absolute, MOVES[direction]):
            raise ValueError('A posicao do robo mudou inesperadamente durante a rota.')
        previous_absolute = actual


def part1(robot, output_dir, map_name='', log=print):
    output_dir = Path(output_dir)
    log('PARTE 1: reiniciando o labirinto e consultando /get_map.')
    loaded_map = robot.reset(map_name)
    grid = robot.read_map()
    cells = grid_cells(grid)
    start, target = marker(cells, 'r'), marker(cells, 't')
    route = bfs(cells, start, target)
    log(f'BFS encontrou uma rota minima de {len(route) - 1} passos. Executando...')
    walk(robot, route, initial_absolute=start)
    result = {
        'etapa': 1,
        'algoritmo': 'BFS',
        'mapa': loaded_map,
        'origem': list(start),
        'alvo': list(target),
        'grid': grid,
        'rota': json_route(route),
        'comandos': directions(route),
        'passos': len(route) - 1,
        'execucao_concluida': True,
    }
    save_json(output_dir / 'parte1.json', result)
    log(f'Parte 1 concluida. Evidencia: {output_dir / "parte1.json"}')
    return result


def explore(robot, max_steps=20000, log=print):
    """Recebe somente sensores e respostas de movimento. Nao consulta mapa global."""
    explorer = Explorer()
    last_absolute = None
    while True:
        explorer.observe(robot.sense())
        direction = explorer.next_direction()
        if direction is None:
            break
        if len(explorer.trace) - 1 >= max_steps:
            raise ValueError('Limite de passos de exploracao atingido.')
        actual = tuple(robot.move(direction))
        if last_absolute is not None and actual != add(last_absolute, MOVES[direction]):
            raise ValueError('Deslocamento inesperado: pare a teleoperacao e tente novamente.')
        explorer.confirm_move(direction)
        last_absolute = actual
        steps = len(explorer.trace) - 1
        if steps % 50 == 0:
            log(f'Exploracao: {steps} movimentos; {len(explorer.visited)} celulas visitadas.')
    if explorer.target is None:
        raise ValueError('A exploracao terminou sem encontrar um alvo acessivel.')
    return explorer


def save_discovered_map(output_dir, explorer, map_name):
    min_row = min(row for row, _ in explorer.cells)
    max_row = max(row for row, _ in explorer.cells)
    min_col = min(col for _, col in explorer.cells)
    max_col = max(col for _, col in explorer.cells)
    result = {
        'mapa': map_name,
        'coordenadas': 'relativas a origem do robo; linha cresce para baixo, coluna para direita',
        'origem': [0, 0],
        'alvo': list(explorer.target),
        'origem_do_csv': [min_row, min_col],
        'celulas': [
            {'linha': row, 'coluna': col, 'tipo': value}
            for (row, col), value in sorted(explorer.cells.items())
        ],
        'celulas_visitadas': len(explorer.visited),
        'movimentos_exploracao': len(explorer.trace) - 1,
        'consultas_get_map_na_exploracao': 0,
    }
    save_json(output_dir / 'mapa_descoberto.json', result)
    save_json(output_dir / 'exploracao.json', {'trajeto': json_route(explorer.trace)})
    with (output_dir / 'mapa_descoberto.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        for row in range(min_row, max_row + 1):
            writer.writerow([
                'r' if (row, col) == (0, 0) else explorer.cells.get((row, col), '?')
                for col in range(min_col, max_col + 1)
            ])


def compare(reference, discovered, cells, route):
    start = tuple(reference['origem'])
    reference_cells = grid_cells(reference['grid'])
    relative_reference_route = [
        (row - start[0], col - start[1]) for row, col in reference['rota']
    ]
    observations_match = all(
        ('f' if reference_cells.get(add(start, position)) == 'r'
         else reference_cells.get(add(start, position))) == value
        for position, value in cells.items()
    )
    checks = {
        'mesmo_labirinto': reference['mapa'] == discovered['mapa'],
        'mesmo_alvo': add(start, tuple(discovered['alvo'])) == tuple(reference['alvo']),
        'observacoes_compativeis_com_mapa_original': observations_match,
        'mesmo_numero_de_passos': reference['passos'] == len(route) - 1,
        'mesma_rota': relative_reference_route == route,
    }
    return {
        **checks,
        'mapa_suficiente_para_reproduzir_rota': all(checks.values()),
        'passos_parte1': reference['passos'],
        'passos_parte2': len(route) - 1,
        'ordem_de_desempate_bfs': list(MOVES),
    }


def part2(robot, output_dir, map_name='', max_steps=20000, log=print):
    output_dir = Path(output_dir)
    reference_path = output_dir / 'parte1.json'
    if not reference_path.is_file():
        raise ValueError('Execute a parte1 antes, usando a mesma pasta --saida e o mesmo labirinto.')
    # Uma tentativa anterior nao deve aparecer como prova da tentativa atual.
    for filename in ('parte2.json', 'comparacao.json'):
        (output_dir / filename).unlink(missing_ok=True)
    log('PARTE 2: reiniciando e explorando por sensores, sem consultar /get_map.')
    loaded_map = robot.reset(map_name)
    explorer = explore(robot, max_steps, log)
    save_discovered_map(output_dir, explorer, loaded_map)

    # Recarregamos o arquivo produzido pela exploracao. O planejamento nao usa
    # o mapa global nem a grade da parte 1.
    discovered = load_json(output_dir / 'mapa_descoberto.json')
    cells = {
        (cell['linha'], cell['coluna']): cell['tipo'] for cell in discovered['celulas']
    }
    route = bfs(cells, tuple(discovered['origem']), tuple(discovered['alvo']))

    # A referencia so e lida DEPOIS do mapeamento e do planejamento independente.
    comparison = compare(load_json(reference_path), discovered, cells, route)
    comparison['rota_reproduzida_no_simulador'] = False
    save_json(output_dir / 'comparacao.json', comparison)
    if not comparison['mapa_suficiente_para_reproduzir_rota']:
        raise ValueError('A comparacao falhou. Confira comparacao.json e use o mesmo labirinto.')

    log(f'Mapa salvo e validado. Reiniciando para reproduzir {len(route) - 1} passos.')
    replay_map = robot.reset(loaded_map)
    if replay_map != loaded_map:
        raise ValueError('O simulador carregou outro labirinto durante a reproducao.')
    walk(robot, route)
    result = {
        'etapa': 2,
        'algoritmos': {'exploracao': 'DFS', 'planejamento': 'BFS'},
        'mapa': loaded_map,
        'coordenadas': 'relativas a origem do robo',
        'origem': [0, 0],
        'alvo': discovered['alvo'],
        'rota': json_route(route),
        'comandos': directions(route),
        'passos': len(route) - 1,
        'movimentos_exploracao': discovered['movimentos_exploracao'],
        'execucao_concluida': True,
    }
    comparison['rota_reproduzida_no_simulador'] = True
    save_json(output_dir / 'parte2.json', result)
    save_json(output_dir / 'comparacao.json', comparison)
    log('Parte 2 concluida: mapa suficiente, mesma rota minima e reproducao executada.')
    return result, comparison
