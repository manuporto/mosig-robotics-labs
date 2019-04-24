#ifndef MAPFILEPARSER_H
#define MAPFILEPARSER_H

#include "json.hpp"
#include <iostream>
#include <fstream>
#include <vector>
#include <string>

using json = nlohmann::json;

// This class represents a directed graph using adjacency list representation
class MapFileParser {
private:
  std::string file;
  int number_of_vertices;
  std::vector<std::vector<int>> adj;
  std::vector<std::pair<float, float>> points;

  void parseMapFile();

public:
  MapFileParser(std::string file);
  int getNumberOfVertices();
  std::vector<std::vector<int>> getAdjacencyMatrix();
  std::vector<std::pair<float, float>> getPoints();
};

MapFileParser::MapFileParser(std::string file) : file(file) { parseMapFile(); }

void MapFileParser::parseMapFile() {
  std::ifstream mapFile(file);
  json j;
  mapFile >> j;
  number_of_vertices = j.at("vertices");
  for(std::pair<float, float> point : j.at("points")) {
    points.emplace_back(point.first, point.second);
  }

  for(std::vector<int> row : j.at("graph")) {
    adj.push_back(row);
  }

  mapFile.close();
}

int MapFileParser::getNumberOfVertices() {
  return number_of_vertices;
}

std::vector<std::vector<int>> MapFileParser::getAdjacencyMatrix() {
  return adj;
}

std::vector<std::pair<float, float>> MapFileParser::getPoints() {
  return points;
}

#endif
