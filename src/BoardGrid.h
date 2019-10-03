// BoarGrid.h
#ifndef PCBROUTER_BOARD_GRID_H
#define PCBROUTER_BOARD_GRID_H

#include <iostream>
#include <iomanip>
#include <queue>
#include <unordered_set>
#include <vector>
#include <unordered_map>
#include <array>
#include <queue>
#include <limits>
#include <algorithm>
#include <fstream>
#include <string>
#include "globalParam.h"
#include "point.h"

// custom Location priority queue class for search
template <typename T, typename priority_t>
struct LocationQueue
{
  typedef std::pair<priority_t, T> PQElement;
  std::priority_queue<PQElement, std::vector<PQElement>,
                      std::greater<PQElement>>
      elements;

  inline bool empty() const
  {
    return elements.empty();
  }

  inline void push(T item, priority_t priority)
  {
    elements.emplace(priority, item);
  }

  inline size_t size() const
  {
    return elements.size();
  }

  T front()
  {
    return elements.top().second; // best item
    // T best_item = elements.top().second;
    // return best_item;
  }

  inline void pop()
  {
    elements.pop();
  }
};

// Hash function for Location to support unordered_set
namespace std
{
template <>
struct hash<Location>
{
  size_t operator()(Location const &x) const
  {
    return (((53 + x.m_x) * 53 + x.m_y) * 53 + x.m_z);
  }
};
} // namespace std

namespace std
{
template <>
struct greater<std::pair<float, Location>>
{
  bool operator()(std::pair<float, Location> const &x, std::pair<float, Location>

                  const &y) const
  {
    return x.first > y.first;
  }
};
} // namespace std

class GridCell
{
public:
  //ctor
  GridCell() {}
  //dtor
  ~GridCell() {}

  friend class BoardGrid;

private:
  float baseCost = 0.0;
  float workingCost = 0.0;
  float viaCost = 0.0;
  int cameFromId = -1;
  bool isTargetedPin = false;
};

class GridPin
{
public:
  //ctor
  GridPin() {}
  //dtor
  ~GridPin() {}

  friend class BoardGrid;
  friend class MultipinRoute;

private:
  std::vector<Location> pinWithLayers;
};

class MultipinRoute
{
public:
  std::vector<Location> pins;
  //TODO: change below to flag in the grid
  std::unordered_set<Location> targetPins;
  //std::vector<std::unordered_map<Location, Location>> came_from;
  std::vector<Location> features;
  std::vector<Location> vias;
  std::vector<GridPin> gridPins;
  int netId;

  void addPin(std::vector<Location> &_pinWithLayers)
  {
    //TODO:: Optimize below for speeding up
    GridPin gridPin;
    gridPin.pinWithLayers = _pinWithLayers;
    gridPins.push_back(gridPin);
  }

  MultipinRoute()
  {
  }
  MultipinRoute(int netId)
  {
    this->netId = netId;
  }
  MultipinRoute(std::vector<Location> pins)
  {
    this->pins = pins;
    this->netId = 0;
  }
  MultipinRoute(std::vector<Location> pins, int netId)
  {
    this->pins = pins;
    this->netId = netId;
  }
};

class BoardGrid
{
  float *base_cost = nullptr;    //Initialize to nullptr
  float *working_cost = nullptr; //Initialize to nullptr
  float *via_cost = nullptr;     //Initialize to nullptr
  GridCell *grid = nullptr;      //Initialize to nullptr
  int size = 0;                  //Total number of cells

  void working_cost_fill(float value);
  float working_cost_at(const Location &l) const;
  void working_cost_set(float value, const Location &l);

  void setCameFromId(const Location &l, const int id);
  int getCameFromId(const Location &l) const;
  int getCameFromId(const int id) const;

  // void add_route_to_base_cost(const Route &route, int radius, float cost);
  void add_route_to_base_cost(const MultipinRoute &route, int radius, float cost, int via_size);
  // void remove_route_from_base_cost(const Route &route, int radius, float cost);
  void remove_route_from_base_cost(const MultipinRoute &route, int radius, float cost);

  void came_from_to_features(const std::unordered_map<Location, Location> &came_from, const Location &end, std::vector<Location> &features) const;
  std::vector<Location> came_from_to_features(const std::unordered_map<Location, Location> &came_from, const Location &end) const;
  void came_from_to_features(const Location &end, std::vector<Location> &features) const;

  void getNeighbors(const Location &l, int via_size, std::array<std::pair<float, Location>, 10> &ns) const;

  // std::unordered_map<Location, Location> dijkstras_with_came_from(const Location &start, const Location &end);
  std::unordered_map<Location, Location> dijkstras_with_came_from(const Location &start, int via_size);
  std::unordered_map<Location, Location> dijkstras_with_came_from(const std::vector<Location> &route, int via_size);
  void dijkstras_with_came_from(const std::vector<Location> &route, int via_size, std::unordered_map<Location, Location> &came_from);
  void dijkstrasWithGridCameFrom(const std::vector<Location> &route, int via_size);
  void aStarWithGridCameFrom(const std::vector<Location> &route, Location &finalEnd, int via_size);
  void breadth_first_search(const Location &start, const Location &end);
  std::unordered_map<Location, Location> breadth_first_search_with_came_from(const Location &start, const Location &end);

  int locationToId(const Location &l) const;
  void idToLocation(const int id, Location &l) const;

public:
  int w; // width
  int h; // height
  int l; // layers
  int current_trace_width;
  int current_half_trace_width;
  int current_clearance;
  int current_via_diameter;
  int current_half_via_diameter;
  Location current_targeted_pin;

  float cost_to_occupy(const Location &l) const;

  void initilization(int w, int h, int l);

  // constraints
  void set_current_rules(const int clr, const int trWid, int viaDia);

  // base cost
  void base_cost_fill(float value);
  float base_cost_at(const Location &l) const;
  void base_cost_set(float value, const Location &l);
  void base_cost_add(float value, const Location &l);
  bool validate_location(const Location &l) const;

  // void add_route(Route &route);
  void add_route(MultipinRoute &route);
  void addRoute(MultipinRoute &route);
  void addRouteWithGridPins(MultipinRoute &route);
  // void ripup_route(Route &route);
  void ripup_route(MultipinRoute &route);

  // via
  float sized_via_cost_at(const Location &l, int via_size) const;
  float via_cost_at(const Location &l) const;
  void add_via_cost(const Location &l, int layer);
  void remove_via_cost(const Location &l, int layer);
  void via_cost_set(float value, const Location &l);
  void via_cost_add(float value, const Location &l);

  // trace_width
  float sized_trace_cost_at(const Location &l, int traceRadius) const;

  // targetPin
  void setTargetedPins(const std::vector<Location> &pins);
  void clearTargetedPins(const std::vector<Location> &pins);
  void setIsTargetedPin(const Location &l);
  void clearIsTargetedPin(const Location &l);
  bool isTargetedPin(const Location &l);

  // cost
  float getEstimatedCost(const Location &l);

  void printGnuPlot();
  void printMatPlot();
  void pprint();
  void print_came_from(const std::unordered_map<Location, Location> &came_from, const Location &end);
  void print_route(const std::unordered_map<Location, Location> &came_from, const Location &end);
  void print_features(std::vector<Location> features);

  //ctor
  BoardGrid() {}

  //dtor
  ~BoardGrid()
  {
    delete[] this->base_cost;
    this->base_cost = NULL;
    delete[] this->working_cost;
    this->working_cost = NULL;
    delete[] this->via_cost;
    this->via_cost = NULL;
  }
};

#endif