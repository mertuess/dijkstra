#include <fstream>
#include <iostream>
#include <string>
#include <tuple>
#include <vector>

#include "graph.hpp"

using namespace std;

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
