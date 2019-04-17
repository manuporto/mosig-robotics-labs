#ifndef MAPFILEPARSER_H
#define MAPFILEPARSER_H

#include "json.hpp"
#include <iostream>
#include <fstream>
#include <list>
#include <string>

using json = nlohmann::json;

// This class represents a directed graph using adjacency list representation
class MapFileParser {
private:
  std::string file;
  int number_of_vertices;
  std::list<std::list<int>> adj;
  std::list<std::pair<float, float>> points;

  void parseMapFile();

public:
  MapFileParser(std::string file);
  int getNumberOfVertices();
  std::list<std::list<int>> getAdjacencyMatrix();
  std::list<std::pair<float, float>> getPoints();
};

MapFileParser::MapFileParser(std::string file) : file(file) { parseMapFile(); }

void MapFileParser::parseMapFile() {
  std::ifstream mapFile(file);
  json j;
  mapFile >> j;
  std::cout << std::setw(4) << j << std::endl;
  mapFile.close();
}

#endif