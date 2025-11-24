# Introdução

Este repositório contém a atividade completa de **navegação e mapeamento** desenvolvida para o ambiente ROS2.
O projeto inclui:

* O simulador do labirinto (pacote `culling_games`)
* Os solvers de navegação e mapeamento implementados em C++ (`maze_solvers`)
* Toda a estrutura necessária para compilar e executar o sistema no Windows utilizando ROS2 Humble

O foco principal do repositório é resolver duas etapas do desafio:

1. **Navegar em um labirinto conhecido**
2. **Explorar, mapear e depois navegar no labirinto desconhecido**

Ambas as etapas são resolvidas com algoritmos de busca, porém aplicados de maneiras diferentes.

# Algoritmos nos Scripts de Navegação

Ambos utilizam **BFS (Breadth-First Search)** como algoritmo principal, mas com aplicações e estratégias diferentes:

## Part 1 (Mapa Conhecido)
- **Algoritmo**: BFS tradicional
- **Função**: `bfs()` 
- **Propósito**: Encontrar o caminho mais curto em um mapa já conhecido
- **Características**: 
  - Recebe o mapa completo de uma vez através do serviço `/get_map`
  - Calcula a rota ótima antes de executar qualquer movimento
  - Executa a rota perfeita em uma única passagem

## Part 2 (Exploração + Navegação)

### Abordagem Híbrida: Greedy + BFS

A Part 2 utiliza uma **combinação estratégica** de algoritmos:

### 1. **Greedy para Exploração Local**
- Quando há vizinhos desconhecidos adjacentes, **escolhe sempre o primeiro da lista**
- Decisão local gulosa: "vou para o desconhecido mais imediato"
- A ordem de processamento (up → down → left → right) cria um viés direcional
- Estratégia oportunista: explora o que está ao alcance imediato

### 2. **BFS para Exploração Planejada** (`find_nearest_unknown()`)
- Quando não há células desconhecidas adjacentes
- Busca a célula desconhecida **mais próxima** em todo o mapa
- Guia o robô para explorar sistematicamente o labirinto
- Continua até mapear todo o espaço acessível

### 3. **BFS para Navegação** (`bfs_shortest_path()`)
- Calcula rotas no mapa parcialmente conhecido
- Usado durante a exploração e depois para ir ao goal
- Atualiza dinamicamente conforme descobre novas áreas

### 4. **BFS Final para Rota Otimizada**
- Após encontrar o goal, recalcula a rota perfeita no mapa completo
- Reseta o jogo e executa essa rota aprendida
- Compara o resultado com a rota da Part 1

