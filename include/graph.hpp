#pragma once

#include <climits>
#include <iomanip>
#include <iostream>
#include <queue>
#include <vector>

using namespace std;

// Структура для представления ребра графа
struct Edge {
  int dest;   // Вершина, в которую ведет ребро
  int weight; // Вес ребра (стоимость перехода)

  // Конструктор для удобного создания ребер
  Edge(int d, int w) : dest(d), weight(w) {}
};

// Класс для представления ориентированного взвешенного графа
class Graph {
  int V;                    // Количество вершин в графе
  vector<vector<Edge>> adj; // Список смежности (для каждой вершины храним
                            // вектор исходящих ребер)

public:
  Graph(int vertices) : V(vertices), adj(vertices) {}

  // Метод для добавления ребра в граф
  void addEdge(int src, int dest, int weight) {
    adj[src].emplace_back(dest, weight);
  }

  // Метод для вывода графа в удобочитаемом формате
  void printGraph() {
    cout << "\n=== DIRECTED WEIGHTED GRAPH ===\n";
    cout << "Edge list (format: source -> destination : weight):\n";
    cout << "----------------------------------------\n";
    // Перебираем все вершины
    for (int u = 0; u < V; ++u) {
      // Для каждой вершины перебираем все исходящие ребра
      for (const Edge &edge : adj[u]) {
        // Выводим информацию о ребре
        cout << "[" << u << " -> " << edge.dest << " : " << edge.weight
             << "]\n";
      }
    }
    cout << "----------------------------------------\n";
  }

  // Реализация алгоритма Дейкстры для поиска кратчайших путей
  void dijkstra(int start) {
    // Очередь с приоритетом для выбора следующей вершины с минимальным
    // расстоянием Храним пары (расстояние, вершина)
    priority_queue<pair<int, int>, vector<pair<int, int>>,
                   greater<pair<int, int>>>
        pq;

    // Вектор расстояний (изначально все расстояния бесконечны)
    vector<int> dist(V, INT_MAX);

    // Вектор для отметки посещенных вершин
    vector<bool> visited(V, false);

    // Начинаем с начальной вершины
    pq.push({0, start});
    dist[start] = 0; // Расстояние до себя равно 0

    cout << "\n=== DIJKSTRA'S ALGORITHM EXECUTION ===\n";

    int step = 1; // Счетчик шагов для вывода
    while (!pq.empty()) {
      // Извлекаем вершину с минимальным расстоянием
      int u = pq.top().second;
      pq.pop();

      // Если вершина уже обработана, пропускаем
      if (visited[u])
        continue;
      visited[u] = true; // Помечаем как посещенную

      // Выводим информацию о текущем шаге
      cout << "\nStep " << step++ << ": Current vertex = " << u << endl;
      cout << "Distances: [";
      // Выводим текущие расстояния до всех вершин
      for (int i = 0; i < V; ++i) {
        if (i != 0)
          cout << ", ";
        if (dist[i] == INT_MAX)
          cout << "INF";
        else
          cout << dist[i];
      }
      cout << "]" << endl;

      // Перебираем все соседние вершины (по исходящим ребрам)
      for (const Edge &edge : adj[u]) {
        int v = edge.dest;        // Куда ведет ребро
        int weight = edge.weight; // Вес ребра

        cout << "  Checking edge " << u << " -> " << v << " (weight: " << weight
             << ")";

        // Если нашли более короткий путь
        if (dist[v] > dist[u] + weight) {
          dist[v] = dist[u] + weight; // Обновляем расстояние
          pq.push({dist[v], v});      // Добавляем в очередь
          cout << " -> Updated distance to " << v << ": " << dist[v];
        }
        cout << endl;
      }
    }

    // Выводим итоговые результаты
    cout << "\n=== DIJKSTRA'S ALGORITHM RESULTS ===\n";
    cout << "Shortest distances from vertex " << start << ":\n";
    // Рисуем таблицу
    cout << "+------------+------------+\n";
    cout << "|  Vertex    | Distance   |\n";
    cout << "+------------+------------+\n";
    // Выводим расстояния для всех вершин, кроме начальной
    for (int i = 0; i < V; ++i) {
      if (i != start) {
        cout << "|     " << setw(3) << i << "     |    ";
        if (dist[i] == INT_MAX)
          cout << setw(5) << "INF";
        else
          cout << setw(5) << dist[i];
        cout << "    |\n";
      }
    }
    cout << "+------------+------------+\n";
  }
};
