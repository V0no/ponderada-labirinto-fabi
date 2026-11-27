# Navegação autônoma em labirinto

## Introdução à ponderada

Nesta atividade ponderada, apliquei algoritmos de grafos para resolver um
desafio de navegação autônoma em labirinto. O trabalho foi dividido em duas
partes: encontrar uma rota mínima até o alvo com acesso ao mapa completo e,
depois, explorar o ambiente para construir um mapa capaz de reproduzir essa
mesma rota.

O simulador ROS 2/Pygame foi fornecido em
[rmnicola/culling_games](https://github.com/rmnicola/culling_games) e está em
`culling_games/`, com sua licença original preservada. O código da solução está
no pacote `src/navegacao_autonoma/`.
## Como executar

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

Executei os comandos com `source install/setup.zsh` porque meu terminal usa
Zsh. Para quem usa Bash, o comando equivalente é `source install/setup.bash`.

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

### O que tinha que ser feito

Nesta parte, eu precisava desenvolver um algoritmo que encontrasse uma rota
otimizada entre a posição inicial do robô e o alvo, tendo acesso ao mapa
completo do labirinto.

### O que pensei para esta parte

Pensei no labirinto como um grafo: cada célula livre representa um vértice e
cada movimento permitido entre células vizinhas representa uma aresta. Como
o robô se movimenta somente nas quatro direções cardinais e cada movimento
custa um passo, escolhi a busca em largura (BFS) para encontrar a rota com o
menor número de movimentos.

A BFS explora os vértices por distância crescente a partir da origem. Nesse
caso, como todas as arestas têm o mesmo custo, a primeira rota encontrada até
o alvo é mínima. Também decidi guardar o predecessor de cada célula visitada
para reconstruir o caminho ao final da busca.

### O que foi feito

Implementei a seguinte sequência:

1. Reiniciei o labirinto e consultei o mapa completo pelo serviço `/get_map`.
2. Reconstruí a matriz e localizei `r` (robô) e `t` (alvo).
3. Considerei as células transitáveis e suas conexões nas quatro direções.
4. Apliquei BFS e reconstruí a rota usando os predecessores.
5. Enviei os movimentos pelo serviço `/move_command` e conferi os
   deslocamentos retornados.
6. Salvei o mapa, a rota, os comandos e o custo em `parte1.json`, para usar
   como referência na comparação com a parte 2.

Na implementação, usei a ordem fixa `up`, `right`, `down`, `left` para analisar
os vizinhos. Mantive essa mesma ordem na parte 2 para obter uma escolha
consistente quando existe mais de uma rota mínima.

A busca tem custo de tempo `O(V + E)` e de memória `O(V)`, em que `V` é o
número de vértices e `E` é o número de arestas. Como cada célula tem no máximo
quatro conexões, a quantidade de arestas é proporcional à de vértices.

## Parte 2: exploração e mapeamento

### O que tinha que ser feito

Nesta parte, eu precisava fazer o robô navegar pelo labirinto e construir o
mapa durante a exploração. Depois, precisava comprovar que o mapa criado era
suficiente para reproduzir a rota encontrada na parte 1.

### O que pensei para esta parte

Escolhi a busca em profundidade (DFS) para organizar a exploração. Minha ideia
foi visitar células livres ainda desconhecidas e retornar pelos corredores
quando não houvesse mais caminhos novos naquele ponto. Para evitar repetições
em ciclos, mantive o registro das células já visitadas.

Como a DFS não garante a rota mínima, usei esse algoritmo para mapear o
ambiente e deixei a busca da rota para uma BFS aplicada ao mapa descoberto.
Também defini a posição inicial como `(0, 0)` e acompanhei os movimentos em
coordenadas relativas, sem depender do mapa completo durante a exploração.

Um detalhe do simulador influenciou essa escolha: ao chegar ao alvo, o robô
não pode mais se movimentar. Por isso, decidi registrar o alvo ao observá-lo e
adiar sua entrada até a reprodução final. Assim, consegui explorar os
corredores e possíveis atalhos antes de calcular a rota.

Explorei toda a região acessível a partir da origem sem atravessar o alvo.
Essa região é suficiente porque uma rota mínima até o alvo termina na primeira
entrada nele; não preciso atravessá-lo para descobrir uma rota melhor até ele.

### O que foi feito

Implementei a leitura do tópico `/culling_games/robot_sensors` com QoS best
effort compatível com o publicador. Usei as observações das oito células
vizinhas para registrar paredes, espaços livres e o alvo, mantendo os
movimentos restritos a cima, direita, baixo e esquerda. Não considerei células
desconhecidas como transitáveis.

Usei uma pilha na DFS para controlar o avanço e os retornos físicos do robô.
A cada movimento bem-sucedido, atualizei a linha ou a coluna relativa em uma
unidade. Usei as posições absolutas retornadas pelo serviço apenas para
detectar deslocamentos inesperados e ignorei o campo `target_pos`, que também
é retornado pelo simulador.

Concluí a exploração quando todas as células acessíveis sem atravessar o alvo
haviam sido visitadas e o robô havia retornado à origem. Depois:

1. Salvei o mapa em JSON e CSV e registrei o trajeto da exploração.
2. Reabri o JSON e apliquei BFS somente às células descobertas.
3. Li a referência da parte 1 e comparei o mapa, o alvo, as observações, o
   custo e a sequência da rota, ajustando a diferença entre coordenadas
   absolutas e relativas.
4. Reiniciei o mesmo labirinto e executei a rota calculada com o mapa
   descoberto.

Mantive a leitura de `parte1.json` somente depois da construção do mapa e do
cálculo independente da rota. Na comparação, exigi tanto o mesmo número de
passos quanto a mesma sequência de movimentos da parte 1.

Na exploração, cada aresta da árvore DFS é percorrida na ida e na volta.
Assim, para `V` células visitadas, o robô realiza `2(V - 1)` movimentos. A
análise dos vizinhos tem custo `O(V + E)`, e o mapa e a pilha usam memória
`O(V)`.

## Resultados e testes

### Resultado no simulador

A execução no simulador ROS com `1.csv` está registrada em `resultados/`:

| Medida | Resultado |
| --- | --- |
| Rota da parte 1 | 102 passos |
| Células visitadas na exploração | 391 |
| Movimentos da exploração, incluindo retornos | 780 |
| Rota reproduzida na parte 2 | 102 passos, com a mesma sequência da parte 1 |

Todas as verificações em `resultados/comparacao.json` foram concluídas com
sucesso. Os 780 movimentos da exploração correspondem à ida e à volta pelas
390 arestas da árvore DFS: `2 × (391 - 1) = 780`.

### Evidências geradas

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

### Testes automatizados

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

## Limitações

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
