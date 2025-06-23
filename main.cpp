#include <climits>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <queue>
#include <string>
#include <tuple>
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

// Функция для вывода инструкции по использованию программы
void printUsage() {
  cout << "Usage:\n";
  cout << "  dijkstra <input_file> [start_vertex]\n\n";
  cout << "Arguments:\n";
  cout << "  input_file    - Text file containing graph edges (format: source "
          "dest weight)\n";
  cout << "  start_vertex  - (Optional) Starting vertex for Dijkstra's "
          "algorithm (default: 0)\n";
  cout << "\nExample input file content:\n";
  cout << "0 1 10\n";
  cout << "0 2 5\n";
  cout << "1 2 2\n";
}

// Функция для чтения графа из файла
Graph readGraphFromFile(const string &filename, int &maxVertex) {
  // Открываем файл для чтения
  ifstream file(filename);
  if (!file.is_open()) {
    cerr << "Error: Could not open file " << filename << endl;
    exit(1);
  }

  // Временное хранилище для ребер
  vector<tuple<int, int, int>> edges;
  maxVertex = 0; // Для определения максимального номера вершины
  int src, dest, weight;

  // Читаем все ребра из файла
  while (file >> src >> dest >> weight) {
    edges.emplace_back(src, dest, weight);
    // Обновляем максимальный номер вершины
    if (src > maxVertex)
      maxVertex = src;
    if (dest > maxVertex)
      maxVertex = dest;
  }

  // Создаем граф с достаточным количеством вершин (вершины нумеруются с 0)
  Graph g(maxVertex + 1);

  // Добавляем все ребра в граф
  for (const auto &edge : edges) {
    g.addEdge(get<0>(edge), get<1>(edge), get<2>(edge));
  }

  return g;
}

int main(int argc, char *argv[]) {
  // Разбираем аргументы командной строки
  if (argc < 2) {
    printUsage();
    return 1;
  }

  // Имя файла с графом - первый аргумент
  string filename = argv[1];
  // Начальная вершина (по умолчанию 0)
  int startVertex = 0;

  // Если указан второй аргумент, используем его как начальную вершину
  if (argc >= 3) {
    startVertex = atoi(argv[2]);
  }

  // Читаем граф из файла
  int maxVertex;
  Graph g = readGraphFromFile(filename, maxVertex);

  // Проверяем корректность начальной вершины
  if (startVertex < 0 || startVertex > maxVertex) {
    cerr << "Error: Start vertex must be between 0 and " << maxVertex << endl;
    return 1;
  }

  // Запускаем алгоритм Дейкстры
  g.printGraph();
  g.dijkstra(startVertex);

  return 0;
}
