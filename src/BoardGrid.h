// BoarGrid.h
#ifndef BOARD_GRID_H
#define BOARD_GRID_H

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


#define DIAGONAL_COST 1.41421356237
// #define DIAGONAL_COST 1.4
#define LAYER_CHANGE_COST 10.0



// custom Location priority queue class for search
template<typename T, typename priority_t>
struct LocationQueue {
  typedef std::pair<priority_t, T> PQElement;
  std::priority_queue<PQElement, std::vector<PQElement>,
                 std::greater<PQElement> > elements;

  inline bool empty() const {
     return elements.empty();
  }

  inline void push(T item, priority_t priority) {
    elements.emplace(priority, item);
  }

  inline size_t size() const {
    return elements.size();
  }

  T front() {
  	return elements.top().second; // best item
    // T best_item = elements.top().second;
    // return best_item;
  }

  inline void pop() {
  	elements.pop();
  }
};


// Location class for board position, just three ints
class Location {
public:
	int x;
	int y;
	int z;
	bool operator==(const Location &other) const;
	Location(int x, int y, int z) {
		this->x = x;
		this->y = y;
		this->z = z;
	}
	Location() {
		this->x = 0;
		this->y = 0;
		this->z = 0;
	}
};

std::ostream &operator<<(std::ostream &os, Location const &l);

// Hash function for Location to support unordered_set
namespace std
{
    template <>
    struct hash<Location>
    {
        size_t operator()(Location const & x) const
        {
            return (((53 + x.x) * 53 + x.y) * 53 + x.z);
        }
    };
}

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
}



class Route {
public:
    Location start;
    Location end;
    std::unordered_map<Location, Location> came_from;
    std::vector<Location> features;
    int netId;
    Route() {

    }
    Route(Location start, Location end) {
        this->start = start;
        this->end = end;
        this->netId = 0;
    }
    Route(Location start, Location end, int netId) {
        this->start = start;
        this->end = end;
        this->netId = netId;
    }
};

class MultipinRoute {
public:
  std::vector<Location> pins;
  std::vector<std::unordered_map<Location, Location>> came_from;
  std::vector<Location> features;
  int netId;
  MultipinRoute() {

  }
  MultipinRoute(std::vector<Location> pins) {
    this->pins = pins;
    this->netId = 0;
  }
  MultipinRoute(std::vector<Location> pins, int netId) {
    this->pins = pins;
    this->netId = netId;
  }
};


class BoardGrid {
  float* base_cost;
  float* working_cost;
  int size; // total number of cells

	void working_cost_fill(float value);
  float working_cost_at(const Location &l) const;
  void working_cost_set(float value, const Location &l);


  void add_route_to_base_cost (const Route &route, int radius, float cost);
  void add_route_to_base_cost (const MultipinRoute &route, int radius, float cost);
  void remove_route_from_base_cost (const Route &route, int radius, float cost);

  void came_from_to_features (const std::unordered_map<Location, Location> &came_from, const Location &end, std::vector<Location> &features) const;
  std::vector<Location> came_from_to_features (const std::unordered_map<Location, Location> &came_from, const Location &end) const;
  
  std::array<std::pair<float, Location>, 10> neighbors (const Location &l) const;

  // std::unordered_map<Location, Location> dijkstras_with_came_from(const Location &start, const Location &end);
  std::unordered_map<Location, Location> dijkstras_with_came_from(const Location &start);
  std::unordered_map<Location, Location> dijkstras_with_came_from(const std::vector<Location> &route);
  void breadth_first_search(const Location &start, const Location &end);
  std::unordered_map<Location, Location> breadth_first_search_with_came_from(const Location &start, const Location &end);

public:
  int w; // width
  int h; // height
  int l; // layers
	void base_cost_fill(float value);
  float base_cost_at(const Location &l) const;
  void base_cost_set(float value, const Location &l);
  bool validate_location(const Location &l) const;
  
  void add_route(Route &route);
  void add_route(MultipinRoute &route);
  void ripup_route(Route &route);

  void pprint();
	void print_came_from(const std::unordered_map<Location, Location> &came_from, const Location &end);
	void print_route(const std::unordered_map<Location, Location> &came_from, const Location &end);
  void print_features (std::vector<Location> features);
  BoardGrid(int w, int h, int l) {
  	this->w = w;
  	this->h = h;
  	this->l = l;
  	this->size = w*h*l;
	  this->base_cost = new float[this->size];
	  this->working_cost = new float[this->size];
  }
  ~BoardGrid() {
  	delete [] this->base_cost;
	  this->base_cost = NULL; 
  	delete [] this->working_cost;
  	this->working_cost = NULL;
  }
    
};

#endif