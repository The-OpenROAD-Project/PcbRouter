#ifndef PCBROUTER_BOARD_GRID_H
#define PCBROUTER_BOARD_GRID_H

#include <algorithm>
#include <array>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <list>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include "GridCell.h"
#include "GridNetclass.h"
#include "GridPin.h"
#include "GridPath.h"
#include "MultipinRoute.h"
#include "IncrementalSearchGrids.h"
#include "Location.h"
#include "globalParam.h"
#include "point.h"

class BoardGrid {
   public:
    int w;  // width
    int h;  // height
    int l;  // layers

    //ctor
    BoardGrid() {}

    //dtor
    ~BoardGrid() {
        delete[] this->grid;
        this->grid = nullptr;
    }
    void initilization(int w, int h, int l);

    // constraints
    void setCurrentGridNetclassId(const int id) { currentGridNetclassId = id; }
    void addGridNetclass(const GridNetclass &);
    GridNetclass &getGridNetclass(const int gridNetclassId);
    // Routing APIs
    [[deprecated]] void addRoute(MultipinRoute &route);
    void addRouteWithGridPins(MultipinRoute &route);
    void ripup_route(MultipinRoute &route);
    // working cost
    void working_cost_fill(float value);
    float working_cost_at(const Location &l) const;
    void working_cost_set(float value, const Location &l);
    // cached trace cost
    void cached_trace_cost_fill(float value);
    float cached_trace_cost_at(const Location &l) const;
    void cached_trace_cost_set(float value, const Location &l);
    // cached via cost
    void cached_via_cost_fill(float value);
    float cached_via_cost_at(const Location &l) const;
    void cached_via_cost_set(float value, const Location &l);
    // base cost
    void base_cost_fill(float value);
    float base_cost_at(const Location &l) const;
    void base_cost_set(float value, const Location &l);
    void base_cost_add(float value, const Location &l);
    // via
    [[deprecated]] bool sizedViaExpandableAndCost(const Location &l, const int viaRadius, float &cost) const;
    bool sizedViaExpandableAndCost(const Location &l, const std::vector<Point_2D<int>> &viaRelativeSearchGrids, float &cost) const;
    bool sizedViaExpandableAndIncrementalCost(const Location &curLoc, const std::vector<Point_2D<int>> &viaRelativeSearchGrids, const Location &prevLoc, const float &prevCost, const IncrementalSearchGrids &searchGrids, float &cost) const;
    float via_cost_at(const Location &l) const;
    void add_via_cost(const Location &l, const int layer, const float cost, const int viaRadius);
    void add_via_cost(const Location &l, const int layer, const float cost, const std::vector<Point_2D<int>> &);
    void via_cost_set(const float value, const Location &l);
    void via_cost_add(const float value, const Location &l);
    // void via_cost_fill(float value);
    // targetPin
    void setTargetedPins(const std::vector<Location> &pins);
    void clearTargetedPins(const std::vector<Location> &pins);
    void setTargetedPin(const Location &l);
    void clearTargetedPin(const Location &l);
    bool isTargetedPin(const Location &l);
    // via Forbidden
    void setViaForbiddenArea(const std::vector<Location> &locations);
    void clearViaForbiddenArea(const std::vector<Location> &locations);
    void setViaForbidden(const Location &l);
    void clearViaForbidden(const Location &l);
    bool isViaForbidden(const Location &l) const;
    // Helpers
    inline bool validate_location(const Location &l) const {
        if (l.m_x >= this->w || l.m_x < 0 || l.m_y >= this->h || l.m_y < 0 || l.m_z >= this->l || l.m_z < 0) {
            return false;
        }
        return true;
    }
    void printGnuPlot();
    void printMatPlot(const std::string fileNameTag = "");
    void pprint();
    void print_came_from(const std::unordered_map<Location, Location> &came_from, const Location &end);
    void print_route(const std::unordered_map<Location, Location> &came_from, const Location &end);
    void print_features(std::vector<Location> features);

    void showViaCachePerformance() {
        std::cout << "# Via Cost Cached Miss: " << this->viaCachedMissed << std::endl;
        std::cout << "# Via Cost Cached Hit: " << this->viaCachedHit << std::endl;
        std::cout << "# Via Cost Cached Hit ratio: " << (double)this->viaCachedHit / (this->viaCachedHit + this->viaCachedMissed) << std::endl;
    }

   private:
    GridCell *grid = nullptr;  //Initialize to nullptr
    int size = 0;              //Total number of cells

    long long viaCachedMissed = 0;
    long long viaCachedHit = 0;

    int currentGridNetclassId;
    Location current_targeted_pin;
    //TODO:: Experiment on this...
    std::vector<Location> currentTargetedPinWithLayers;

    // Netclass mapping from DB netclasses, indices are aligned
    std::vector<GridNetclass> mGridNetclasses;

    // trace_width
    float sized_trace_cost_at(const Location &l, const int traceRadius) const;
    float sized_trace_cost_at(const Location &l, const std::vector<Point_2D<int>> &traRelativeSearchGrids) const;
    // came from id
    void setCameFromId(const Location &l, const int id);
    int getCameFromId(const Location &l) const;
    int getCameFromId(const int id) const;
    void clearAllCameFromId();
    // 2D cost estimation
    float getEstimatedCost(const Location &l);
    float getEstimatedCostWithBendingCost(const Location &current, const Location &next);
    // 3D cost esitmation
    float getEstimatedCostWithLayers(const Location &current);
    float getEstimatedCostWithLayersAndBendingCost(const Location &current, const Location &next);

    //TODO: refactor on the trace/via size and their cost......
    void add_route_to_base_cost(const MultipinRoute &route);
    void add_route_to_base_cost(const MultipinRoute &route, const int traceRadius, const float traceCost, const int viaRadius, const float viaCost);
    void addGridPathToBaseCost(const GridPath &route, const int gridNetclassId, const int traceRadius, const int diagonalTraceRadius, const float traceCost, const int viaRadius, const float viaCost);
    void remove_route_from_base_cost(const MultipinRoute &route);

    void came_from_to_features(const std::unordered_map<Location, Location> &came_from, const Location &end, std::vector<Location> &features) const;
    std::vector<Location> came_from_to_features(const std::unordered_map<Location, Location> &came_from, const Location &end) const;
    void came_from_to_features(const Location &end, std::vector<Location> &features) const;

    void getNeighbors(const Location &l, std::vector<std::pair<float, Location>> &ns);

    std::unordered_map<Location, Location> dijkstras_with_came_from(const Location &start, int via_size);
    std::unordered_map<Location, Location> dijkstras_with_came_from(const std::vector<Location> &route, int via_size);
    void dijkstras_with_came_from(const std::vector<Location> &route, int via_size, std::unordered_map<Location, Location> &came_from);
    void dijkstrasWithGridCameFrom(const std::vector<Location> &route, int via_size);
    void aStarWithGridCameFrom(const std::vector<Location> &route, Location &finalEnd, float &finalCost);
    void initializeFrontiers(const std::vector<Location> &route, LocationQueue<Location, float> &frontier);
    void initializeLocationToFrontier(const Location &start, LocationQueue<Location, float> &frontier);
    void breadth_first_search(const Location &start, const Location &end);
    std::unordered_map<Location, Location> breadth_first_search_with_came_from(const Location &start, const Location &end);

    int locationToId(const Location &l) const;
    void idToLocation(const int id, Location &l) const;
};

#endif