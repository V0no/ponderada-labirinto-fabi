"""Cliente ROS 2 dos servicos e sensores do simulador fornecido."""

import argparse
from pathlib import Path
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from cg_interfaces.msg import RobotSensors
from cg_interfaces.srv import GetMap, MoveCmd, Reset

from .algorithms import MOVES, SENSORS
from .solver import part1, part2


class RosRobot(Node):
    def __init__(self, interval=0.08, timeout=10.0):
        super().__init__('navegacao_autonoma')
        self.interval = interval
        self.timeout = timeout
        self.move_client = self.create_client(MoveCmd, '/move_command')
        self.reset_client = self.create_client(Reset, '/reset')
        self.map_client = None
        self.sensor_count = 0
        self.sensor_time = 0.0
        self.sensor_ready_after = 0.0
        self.last_sensors = None
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.sensor_subscription = self.create_subscription(
            RobotSensors, '/culling_games/robot_sensors', self._on_sensors, qos,
        )

    def _on_sensors(self, message):
        self.last_sensors = {field: getattr(message, field) for field in SENSORS}
        self.sensor_time = time.monotonic()
        self.sensor_count += 1

    def _pause(self, duration):
        deadline = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(0.02, max(0.0, deadline - time.monotonic())))
        if not rclpy.ok():
            raise RuntimeError('A conexao ROS foi encerrada.')

    def _call(self, client, request):
        if not client.wait_for_service(timeout_sec=self.timeout):
            raise RuntimeError(f'Servico {client.srv_name} indisponivel. Abra o simulador primeiro.')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.timeout)
        if not future.done():
            future.cancel()
            raise RuntimeError(f'Tempo esgotado aguardando {client.srv_name}. Reinicie a etapa.')
        if future.exception() is not None:
            raise RuntimeError(f'Falha em {client.srv_name}: {future.exception()}')
        response = future.result()
        if response is None:
            raise RuntimeError(f'O servico {client.srv_name} nao retornou uma resposta.')
        return response

    def reset(self, map_name=''):
        request = Reset.Request()
        request.is_random = False
        request.map_name = map_name
        response = self._call(self.reset_client, request)
        if not response.success:
            raise ValueError(f'Nao foi possivel carregar o mapa {map_name!r}.')
        # O servico confirma a solicitacao antes do loop Pygame aplicar o reset.
        self.sensor_ready_after = time.monotonic() + max(0.25, self.interval)
        self._pause(max(0.25, self.interval))
        self.sense()
        return response.loaded_map_name

    def read_map(self):
        # Este cliente so e criado e chamado na parte 1.
        if self.map_client is None:
            self.map_client = self.create_client(GetMap, '/get_map')
        response = self._call(self.map_client, GetMap.Request())
        shape = list(response.occupancy_grid_shape)
        if len(shape) != 2 or not all(size > 0 for size in shape):
            raise ValueError('O simulador retornou dimensoes invalidas para o mapa.')
        rows, cols = shape
        flat = list(response.occupancy_grid_flattened)
        if len(flat) != rows * cols:
            raise ValueError('O numero de celulas nao corresponde as dimensoes do mapa.')
        return [flat[index * cols:(index + 1) * cols] for index in range(rows)]

    def move(self, direction):
        if direction not in MOVES:
            raise ValueError(f'Direcao invalida: {direction!r}.')
        request = MoveCmd.Request()
        request.direction = direction
        response = self._call(self.move_client, request)
        if not response.success:
            raise ValueError(
                f'O simulador recusou o movimento {direction}. '
                'Pare outros controladores e reinicie a etapa.'
            )
        # target_pos e intencionalmente ignorado: a descoberta do alvo usa sensores.
        self.sensor_ready_after = time.monotonic() + self.interval
        self._pause(self.interval)
        return tuple(response.robot_pos)

    def sense(self):
        deadline = time.monotonic() + self.timeout
        initial_count = self.sensor_count
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if (self.last_sensors is not None
                    and self.sensor_time >= self.sensor_ready_after
                    and self.sensor_count >= initial_count + 2):
                return dict(self.last_sensors)
        raise RuntimeError('Nao chegaram leituras recentes dos sensores. Confira o simulador.')


def main():
    parser = argparse.ArgumentParser(description='Solucao da ponderada de navegacao autonoma.')
    parser.add_argument('modo', choices=('parte1', 'parte2', 'demonstracao'))
    parser.add_argument('--mapa', default='', help='Ex.: 1.csv. Vazio conserva o labirinto atual.')
    parser.add_argument('--saida', type=Path, default=Path('resultados'))
    parser.add_argument('--intervalo', type=float, default=0.08, help='Pausa entre movimentos, em segundos.')
    parser.add_argument('--timeout', type=float, default=10.0, help='Limite de espera por servicos e sensores.')
    parser.add_argument('--max-passos', type=int, default=20000)
    args = parser.parse_args(remove_ros_args(args=sys.argv)[1:])
    if args.intervalo < 0.02 or args.timeout <= 0 or args.max_passos <= 0:
        parser.error('--intervalo deve ser >= 0.02; --timeout e --max-passos devem ser positivos.')
    if Path(args.mapa).name != args.mapa and args.mapa:
        parser.error('--mapa aceita somente o nome do arquivo, por exemplo 1.csv.')

    robot = None
    exit_code = 0
    rclpy.init(args=sys.argv)
    try:
        robot = RosRobot(args.intervalo, args.timeout)
        log = lambda message: robot.get_logger().info(message)
        if args.modo in ('parte1', 'demonstracao'):
            part1(robot, args.saida, args.mapa, log)
        if args.modo in ('parte2', 'demonstracao'):
            part2(robot, args.saida, args.mapa, args.max_passos, log)
    except KeyboardInterrupt:
        exit_code = 130
        print('Execucao interrompida. A proxima tentativa reinicia o labirinto.', file=sys.stderr)
    except (ValueError, RuntimeError, OSError) as error:
        exit_code = 1
        print(f'Erro: {error}', file=sys.stderr)
    finally:
        if robot is not None:
            robot.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)
