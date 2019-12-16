// BoarGrid.h
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
#include "globalParam.h"
#include "point.h"

// custom Location priority queue class for search
template <typename T, typename priority_t>
struct LocationQueue {
    typedef std::pair<priority_t, T> PQElement;
    std::priority_queue<PQElement, std::vector<PQElement>,
                        std::greater<PQElement>>
        elements;

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
        return elements.top().second;  // best item
    }

    priority_t frontKey() {
        return elements.top().first;  // best item's key values
    }

    inline void pop() {
        elements.pop();
    }
};

// Hash function for Location to support unordered_set
namespace std {
template <>
struct hash<Location> {
    size_t operator()(Location const &x) const {
        return (((53 + x.m_x) * 53 + x.m_y) * 53 + x.m_z);
    }
};
}  // namespace std

namespace std {
template <>
struct greater<std::pair<float, Location>> {
    bool operator()(std::pair<float, Location> const &x, std::pair<float, Location>

                    const &y) const {
        return x.first > y.first;
    }
};
}  // namespace std

class IncrementalSearchGrids {
   public:
    IncrementalSearchGrids() {}
    ~IncrementalSearchGrids() {}

    std::vector<Point_2D<int>> &getLeftAddGrids() { return mAdditionGridsL; }
    std::vector<Point_2D<int>> &getLeftDedGrids() { return mDeductionGridsL; }
    std::vector<Point_2D<int>> &getRightAddGrids() { return mAdditionGridsR; }
    std::vector<Point_2D<int>> &getRightDedGrids() { return mDeductionGridsR; }
    std::vector<Point_2D<int>> &getForwardAddGrids() { return mAdditionGridsF; }
    std::vector<Point_2D<int>> &getForwardDedGrids() { return mDeductionGridsF; }
    std::vector<Point_2D<int>> &getBackwardAddGrids() { return mAdditionGridsB; }
    std::vector<Point_2D<int>> &getBackwardDedGrids() { return mDeductionGridsB; }

    std::vector<Point_2D<int>> &getLBAddGrids() { return mAdditionGridsLB; }
    std::vector<Point_2D<int>> &getLBDedGrids() { return mDeductionGridsLB; }
    std::vector<Point_2D<int>> &getRBAddGrids() { return mAdditionGridsRB; }
    std::vector<Point_2D<int>> &getRBDedGrids() { return mDeductionGridsRB; }
    std::vector<Point_2D<int>> &getLFAddGrids() { return mAdditionGridsLF; }
    std::vector<Point_2D<int>> &getLFDedGrids() { return mDeductionGridsLF; }
    std::vector<Point_2D<int>> &getRFAddGrids() { return mAdditionGridsRF; }
    std::vector<Point_2D<int>> &getRFDedGrids() { return mDeductionGridsRF; }

    friend class GridNetclass;
    friend class GridBasedRouter;

   private:
    // For incremental costs
    std::vector<Point_2D<int>> mAdditionGridsR;
    std::vector<Point_2D<int>> mDeductionGridsR;
    std::vector<Point_2D<int>> mAdditionGridsL;
    std::vector<Point_2D<int>> mDeductionGridsL;
    std::vector<Point_2D<int>> mAdditionGridsF;
    std::vector<Point_2D<int>> mDeductionGridsF;
    std::vector<Point_2D<int>> mAdditionGridsB;
    std::vector<Point_2D<int>> mDeductionGridsB;

    std::vector<Point_2D<int>> mAdditionGridsRB;
    std::vector<Point_2D<int>> mDeductionGridsRB;
    std::vector<Point_2D<int>> mAdditionGridsRF;
    std::vector<Point_2D<int>> mDeductionGridsRF;
    std::vector<Point_2D<int>> mAdditionGridsLF;
    std::vector<Point_2D<int>> mDeductionGridsLF;
    std::vector<Point_2D<int>> mAdditionGridsLB;
    std::vector<Point_2D<int>> mDeductionGridsLB;
};

class GridNetclass {
   public:
    //ctor
    GridNetclass(const int id = -1,
                 const int clearance = 0,
                 const int trace_width = 0,
                 const int via_dia = 0,
                 const int via_drill = 0,
                 const int uvia_dia = 0,
                 const int uvia_drill = 0)
        : m_id(id), m_clearance(clearance), m_trace_width(trace_width), m_via_dia(via_dia), m_via_drill(via_drill), m_uvia_dia(uvia_dia), m_uvia_drill(uvia_drill) {}
    //dtor
    ~GridNetclass() {}

    int getId() { return m_id; }
    int getClearance() { return m_clearance; }
    int getDiagonalClearance() { return m_clearance_diagonal; }
    int getTraceWidth() { return m_trace_width; }
    int getDiagonalTraceWidth() { return m_trace_width_diagonal; }
    int getHalfTraceWidth() { return m_half_trace_width; }
    int getHalfDiagonalTraceWidth() { return m_half_trace_width_diagonal; }
    int getViaDia() { return m_via_dia; }
    int getHalfViaDia() { return m_half_via_dia; }
    int getViaDrill() { return m_via_drill; }
    int getMicroViaDia() { return m_uvia_dia; }
    int getMicroViaDrill() { return m_uvia_drill; }
    int getViaExpansion() { return m_via_expansion; }
    static int getObstacleExpansion() { return m_obstacle_expansion; }
    int getTraceExpansion() { return m_trace_expansion; }
    int getDiagonalTraceExpansion() { return m_trace_expansion_diagonal; }
    // Setup Derived
    void setHalfTraceWidth(const int halfTraWid) { m_half_trace_width = halfTraWid; }
    void setHalfViaDia(const int halfViaDia) { m_half_via_dia = halfViaDia; }
    // Derived diagonal cases values
    void setDiagonalTraceWidth(const int diagonalTraWid) { m_trace_width_diagonal = diagonalTraWid; }
    void setHalfDiagonalTraceWidth(const int halfDiagonalTraWid) { m_half_trace_width_diagonal = halfDiagonalTraWid; }
    void setDiagonalClearance(const int diagonalClr) { m_clearance_diagonal = diagonalClr; }
    void setViaExpansion(const int viaExp) { m_via_expansion = viaExp; }
    void setTraceExpansion(const int traExp) { m_trace_expansion = traExp; }
    static void setObstacleExpansion(const int obsExp) { m_obstacle_expansion = obsExp; }
    void setDiagonalTraceExpansion(const int traExp) { m_trace_expansion_diagonal = traExp; }
    // Via shape
    void addViaShapeGridPoint(const Point_2D<int> &pt) { mViaShapeToGrids.push_back(pt); }
    void setViaShapeGrids(const std::vector<Point_2D<int>> &grids) { mViaShapeToGrids = grids; }
    const std::vector<Point_2D<int>> &getViaShapeToGrids() const { return mViaShapeToGrids; }
    // Trace searching space
    void setTraceSearchingSpaceToGrids(const std::vector<Point_2D<int>> &grids) { mTraceSearchingSpaceToGrids = grids; }
    const std::vector<Point_2D<int>> &getTraceSearchingSpaceToGrids() const { return mTraceSearchingSpaceToGrids; }
    // Via searching space
    void setViaSearchingSpaceToGrids(const std::vector<Point_2D<int>> &grids) { mViaSearchingSpaceToGrids = grids; }
    const std::vector<Point_2D<int>> &getViaSearchingSpaceToGrids() const { return mViaSearchingSpaceToGrids; }
    // Incremental searching grids
    IncrementalSearchGrids &getTraceIncrementalSearchGrids() { return mTraceIncrementalSearchGrids; }
    IncrementalSearchGrids &getViaIncrementalSearchGrids() { return mViaIncrementalSearchGrids; }
    // Setup the incremental search grids
    void setupViaIncrementalSearchGrids();
    void setupTraceIncrementalSearchGrids();
    void setupIncrementalSearchGrids(const std::vector<Point_2D<int>> &searchGrids, IncrementalSearchGrids &incrementalSearchGrids);

   private:
    void getAddDedSearchGrids(const std::vector<Point_2D<int>> &searchGrids, const std::vector<Point_2D<int>> &shiftedSearchGrids, std::vector<Point_2D<int>> &add, std::vector<Point_2D<int>> &ded);

   private:
    int m_id = -1;
    int m_clearance = 0;
    int m_trace_width = 0;
    int m_via_dia = 0;
    int m_via_drill = 0;
    int m_uvia_dia = 0;
    int m_uvia_drill = 0;

    // Derived
    int m_half_trace_width = 0;
    int m_trace_width_diagonal = 0;
    int m_half_trace_width_diagonal = 0;
    int m_half_via_dia = 0;
    int m_clearance_diagonal = 0;

    int m_trace_expansion = 0;
    int m_trace_expansion_diagonal = 0;
    int m_via_expansion = 0;

    // Refactor below static memeber....
    static int m_obstacle_expansion;

    // Via shape, relative to the via center grid
    std::vector<Point_2D<int>> mViaShapeToGrids;
    // Trace searching space when caluclating grid cost, relative to trace center grid
    std::vector<Point_2D<int>> mTraceSearchingSpaceToGrids;
    // Via searching space when caluclating grid cost, relative to via center grid
    std::vector<Point_2D<int>> mViaSearchingSpaceToGrids;

    IncrementalSearchGrids mTraceIncrementalSearchGrids;
    IncrementalSearchGrids mViaIncrementalSearchGrids;
};

class GridCell {
   public:
    //ctor
    GridCell() {}
    //dtor
    ~GridCell() {}

    friend class BoardGrid;

   private:
    float baseCost = 0.0;  //Record Routed Nets's traces
    //float viaCost = 0.0;   //Record Routed Nets's vias

    float workingCost = 0.0;  //Walked Cost

    //For incremental cost calculation
    float cachedTraceCost = -1.0;
    float cachedViaCost = -1.0;

    int cameFromId = -1;
    //TODO:: Use integer's bit to represent the flag;// Or Enum to represent everything
    bool targetedPin = false;
    bool viaForbidden = false;
};

class GridPin {
   public:
    //ctor
    GridPin() {}
    //dtor
    ~GridPin() {}

    friend class BoardGrid;
    friend class MultipinRoute;
    friend class GridBasedRouter;

    const Point_2D<int> &getPinLL() const { return pinLL; }
    const Point_2D<int> &getPinUR() const { return pinUR; }
    void setPinLL(Point_2D<int> &point) { pinLL = point; }
    void setPinUR(Point_2D<int> &point) { pinUR = point; }
    const std::vector<Location> &getPinWithLayers() const { return pinWithLayers; }
    void addPinShapeGridPoint(const Point_2D<int> &pt) { pinShapeToGrids.push_back(pt); }
    const std::vector<Point_2D<int>> &getPinShapeToGrids() const { return pinShapeToGrids; }

   private:
    // TODO:: Change to layer index only
    std::vector<Location> pinWithLayers;

    // Pin Shape
    std::vector<Point_2D<int>> pinShapeToGrids;
    Point_2D<int> pinLL;
    Point_2D<int> pinUR;
};

class GridPath {
   public:
    //ctor
    GridPath() {}
    //dtor
    ~GridPath() {}

    const std::list<Location> &getSegments() const { return mSegments; }
    void removeRedundantPoints();

    friend class BoardGrid;
    friend class MultipinRoute;

   private:
    std::list<Location> mSegments;  //TODO:: contains vias for now...
};

class MultipinRoute {
   public:
    int netId = -1;
    int gridNetclassId = -1;
    float currentRouteCost = 0.0;
    std::vector<Location> pins;
    std::vector<Location> features;
    std::vector<GridPin> mGridPins;

    //derived from features, doesn't guarantee updated
    std::vector<GridPath> mGridPaths;
    //TODO
    // std::vector<Location> vias;

    void featuresToGridPaths();
    int getGridNetclassId() const { return gridNetclassId; }
    const std::vector<GridPath> &getGridPaths() const { return mGridPaths; }

    GridPath &getNewGridPath() {
        mGridPaths.push_back(GridPath{});
        return mGridPaths.back();
    }
    GridPin &getNewGridPin() {
        mGridPins.push_back(GridPin{});
        return mGridPins.back();
    }

    MultipinRoute() {
    }
    MultipinRoute(const int netId) {
        this->netId = netId;
    }
    MultipinRoute(const int netId, const int gridNetclassId) {
        this->netId = netId;
        this->gridNetclassId = gridNetclassId;
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
    void add_route(MultipinRoute &route);
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
    bool sizedViaExpandableAndCost(const Location &l, const int viaRadius, float &cost) const;
    bool sizedViaExpandableAndCost(const Location &l, const std::vector<Point_2D<int>> &viaRelativeSearchGrids, float &cost) const;
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