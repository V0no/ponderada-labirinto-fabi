# Navegação autônoma em labirinto

Solução da atividade ponderada: planejamento de uma rota mínima com o mapa
completo e exploração autônoma para construir um mapa capaz de reproduzir a
mesma rota.

O simulador ROS 2/Pygame foi fornecido em
[rmnicola/culling_games](https://github.com/rmnicola/culling_games) e está em
`culling_games/`, com sua licença original preservada. O código da solução está
no pacote `src/navegacao_autonoma/`.
O commit de origem e a alteração no limite de quadros estão documentados em
[`culling_games/ORIGEM.md`](culling_games/ORIGEM.md).

## Executar

Ambiente usado: Ubuntu 24.04, ROS 2 Jazzy, Python 3.12, `colcon` e `pygame`.
Todos os comandos abaixo devem ser executados no terminal Ubuntu com ROS.
Use a raiz deste projeto como workspace; os caminhos com espaços devem estar
entre aspas.

Na raiz do projeto:

```zsh
colcon build
source install/setup.zsh
```

No primeiro terminal, abra o simulador:

```zsh
ros2 run cg maze
```

Em outro terminal, entre na mesma pasta, carregue o workspace e execute a
demonstração completa:

```zsh
source install/setup.zsh
ros2 run navegacao_autonoma resolver demonstracao --mapa 1.csv
```

O programa reinicia o labirinto antes de cada etapa e antes da reprodução final.
Ele seleciona o mapa informado pelo serviço `/reset`; portanto, a seleção
aleatória inicial da janela não interfere na comparação.
Encerre a teleoperação com Ctrl+C antes de executar a solução e deixe a janela
do simulador aberta durante as duas partes.

Em Bash, use `source install/setup.bash` no lugar de `setup.zsh`.

### Executar as partes separadamente

Com o simulador aberto:

```zsh
ros2 run navegacao_autonoma resolver parte1 --mapa 1.csv
ros2 run navegacao_autonoma resolver parte2 --mapa 1.csv
```

A parte 2 precisa do arquivo `parte1.json` produzido na mesma pasta de saída,
para comprovar o resultado depois da exploração. Execute as duas partes com o
mesmo mapa. A exploração não lê esse arquivo: a referência só é aberta depois
de construir o mapa e calcular a rota de forma independente.

### Opções

```zsh
ros2 run navegacao_autonoma resolver --help
ros2 run navegacao_autonoma resolver demonstracao --mapa test.csv --saida resultados/teste
ros2 run navegacao_autonoma resolver demonstracao --mapa 1.csv --intervalo 0.15
```

| Opção | Padrão | Função |
| --- | --- | --- |
| `--mapa` | labirinto atual | Nome de um CSV instalado pelo simulador |
| `--saida` | `resultados` | Pasta dos mapas, trajetos e comparações |
| `--intervalo` | `0.08` | Pausa entre movimentos, em segundos |
| `--timeout` | `10` | Limite de espera por serviço ou sensores |
| `--max-passos` | `20000` | Limite dos movimentos da exploração |

## Parte 1: navegação com mapa

1. Reiniciar o labirinto e consultar `/get_map`.
2. Reconstruir a matriz e localizar `r` (robô) e `t` (alvo).
3. Representar cada célula livre como um vértice e cada movimento cardinal
   permitido como uma aresta de custo 1.
4. Aplicar BFS, guardando o predecessor de cada vértice para reconstruir a rota.
5. Enviar os comandos por `/move_command` e conferir os deslocamentos retornados.
6. Salvar a referência da parte 1.

A BFS visita o grafo por distância crescente. Como todos os movimentos custam
um passo, a primeira rota encontrada para o alvo tem o menor número de passos.
O custo de tempo e memória da busca é `O(V + E)` e `O(V)`, respectivamente.
Em um grid com até quatro conexões por célula, `E` é proporcional a `V`.

## Parte 2: exploração e mapeamento

O ponto inicial do robô recebe a coordenada relativa `(0, 0)`. Cada movimento
bem-sucedido atualiza a linha ou coluna em uma unidade. As posições absolutas
retornadas pelo serviço servem para detectar deslocamentos inesperados;
`target_pos`, também retornado pelo simulador, é ignorado.

A DFS usa uma pilha para escolher células livres ainda não visitadas e retornar
fisicamente pelos corredores quando encontra um beco sem saída. O nó assina
`/culling_games/robot_sensors` com QoS best effort compatível com o publicador.
As oito células vizinhas ajudam a construir o mapa, mas os movimentos usam
somente cima, direita, baixo e esquerda. Células desconhecidas nunca são
consideradas transitáveis.

O simulador encerra a movimentação ao alcançar o alvo. Por isso, o robô observa
e registra o alvo, mas adia sua entrada até a reprodução final. A exploração
termina quando todas as células acessíveis sem atravessar o alvo foram visitadas
e o robô retornou à origem.

Essa região é suficiente para encontrar a rota ótima até o alvo: todo caminho
mínimo até ele termina em sua primeira entrada no alvo. Logo, todas as células
anteriores pertencem à componente da origem quando o alvo é removido do grafo.
A DFS explora essa componente inteira, inclusive eventuais atalhos e ciclos.
Não é necessário explorar uma região cujo acesso exija passar primeiro pelo alvo.

Depois da exploração, o programa:

1. Salva o mapa em JSON e CSV, além do trajeto de exploração.
2. Reabre o JSON e aplica BFS apenas às células descobertas.
3. Lê a referência da parte 1 e verifica o mapa, o alvo, as observações, o custo
   e a sequência da rota, ajustando a diferença entre coordenadas absolutas e
   relativas.
4. Reinicia o mesmo labirinto e executa a rota calculada com o mapa descoberto.

As duas buscas usam a ordem fixa `up`, `right`, `down`, `left`. Isso garante uma
escolha consistente quando há mais de um caminho mínimo. A comparação exige
tanto o mesmo número de passos quanto a mesma sequência de movimentos.

A exploração física percorre cada aresta da árvore DFS na ida e na volta:
para `V` células visitadas, são `2(V - 1)` movimentos. A análise dos vizinhos
tem custo `O(V + E)` e o mapa e a pilha usam memória `O(V)`.

## Evidências geradas

| Arquivo na pasta de saída | Conteúdo |
| --- | --- |
| `parte1.json` | Mapa completo de referência, rota, comandos e custo da BFS |
| `mapa_descoberto.json` | Células observadas, coordenadas relativas e estatísticas |
| `mapa_descoberto.csv` | Mapa em matriz; `?` indica célula desconhecida |
| `exploracao.json` | Trajeto físico da exploração, incluindo retornos |
| `parte2.json` | Rota planejada com o mapa descoberto e execução concluída |
| `comparacao.json` | Verificações de suficiência, igualdade e reprodução |

No CSV, `r` marca a origem, `t` o alvo, `f` as células livres e `b` as paredes.
O campo `origem_do_csv` do JSON informa a coordenada relativa da primeira célula
do CSV. O mapa pode conter `?` em regiões que não precisam ser exploradas.

Uma comprovação bem-sucedida registra, entre outros campos:

```json
{
  "mesmo_numero_de_passos": true,
  "mesma_rota": true,
  "mapa_suficiente_para_reproduzir_rota": true,
  "rota_reproduzida_no_simulador": true
}
```

Se a comparação falhar, a reprodução é interrompida e `comparacao.json` mantém
os campos que falharam. Colisões, sensores inconsistentes, limites de passos,
ausência de caminho e indisponibilidade dos serviços também interrompem a etapa
com uma mensagem de erro.

## Testes

A execução no simulador ROS com `1.csv` está registrada em `resultados/`:
102 passos na parte 1, 391 células visitadas em 780 movimentos de exploração
e reprodução da mesma rota de 102 passos na parte 2. Todas as verificações em
`resultados/comparacao.json` foram concluídas com sucesso.

Os algoritmos e a execução podem ser testados sem instalar ROS ou abrir uma
janela, usando a classe `Robot` original do simulador:

```bash
python3 -m unittest discover -s src/navegacao_autonoma/test -v
```

A suíte inclui os 11 mapas fornecidos, ciclos com rotas de mesmo custo, origem
fora do canto, alvo adjacente, alvo que bloqueia acesso a outra região,
alvo inacessível, ausência de referência, comparação entre mapas diferentes,
limite de exploração e dados de sensores inconsistentes. Os testes proíbem
consultas ao mapa completo durante a parte 2 e verificam que a referência só é
lida depois de construir o mapa e planejar a rota.

Esses testes não substituem a execução com os serviços ROS; o simulador deve
permanecer aberto ao usar o comando `resolver`.

## Limitações do simulador e do ambiente

- O mapa é estático e os movimentos têm custo unitário. A solução foi feita
  para os labirintos fornecidos, com bordas de paredes.
- Execute somente um controlador do robô por vez.
- As mensagens de sensores originais não incluem posição nem horário da
  leitura. O cliente usa fila de profundidade 1, aguarda a estabilização após
  os movimentos e recebe novas amostras antes de observar. Em um ambiente
  distribuído com atrasos arbitrários, seria necessário acrescentar posição
  ou timestamp ao protocolo para associar leituras e poses com garantia.
- O serviço `/reset` responde antes de o Pygame aplicar a mudança. O cliente
  aguarda pelo menos 0,25 segundo e novas leituras antes de continuar.
- Neste computador, a versão instalada do rosidl precisou de uma correção de
  leitura UTF-8 para compilar no caminho `Área de trabalho`. Se outro ambiente
  apresentar `list index out of range` com o caminho cortado em `rea de trabalho`,
  o ajuste aplicado no Ubuntu foi:

  ```bash
  sudo sed -i.bak 's/file(STRINGS "${idl_output}" idl_tuples)/file(STRINGS "${idl_output}" idl_tuples ENCODING UTF-8)/' /opt/ros/jazzy/share/rosidl_adapter/cmake/rosidl_adapt_interfaces.cmake
  ```

## Estrutura

```text
mapa-ponderada-fabi/
├── culling_games/                    # Simulador fornecido
├── src/navegacao_autonoma/
│   ├── navegacao_autonoma/
│   │   ├── algorithms.py             # BFS e estado da exploração DFS
│   │   ├── solver.py                 # Etapas, arquivos e comparação
│   │   └── node.py                   # Comunicação ROS 2 e comandos
│   ├── test/test_solution.py
│   ├── package.xml
│   └── setup.py
├── resultados/                       # Evidências produzidas ao executar
└── README.md
```

`build/`, `install/`, `log/` e caches Python são gerados localmente e ignorados
pelo Git. Os arquivos da entrega são o código em `src/`, a cópia do simulador
em `culling_games/`, as seis evidências em `resultados/`, este README e o
`.gitignore`. As configurações locais de ferramentas e credenciais também são
ignoradas pelo Git.
