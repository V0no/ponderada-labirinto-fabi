# Origem do simulador

Este diretório contém o simulador fornecido para a atividade:

- Repositório: https://github.com/rmnicola/culling_games
- Commit de referência: `a6c182a74606bcd80d24fc3a20710b55decc863a`
- A licença MIT original está preservada em `LICENSE`.

A cópia faz parte dos arquivos deste projeto e pode ser obtida junto com a
solução, sem configurar um submódulo Git.

## Alteração local

Em `src/cg/cg/Game.py`, o loop Pygame foi limitado a 60 quadros por segundo
com `pygame.time.Clock().tick(60)`. Isso reduz a ocupação contínua da CPU e
permite que a thread dos serviços ROS processe solicitações com regularidade.
As regras de movimento, os sensores e as interfaces ROS foram preservados.
