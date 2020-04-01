#ifndef PCBROUTER_MULTI_PIN_ROUTE_H
#define PCBROUTER_MULTI_PIN_ROUTE_H

#include <vector>

#include "GridPath.h"
#include "GridPin.h"
#include "globalParam.h"
#include "point.h"

class MultipinRoute {
   public:
    MultipinRoute() {
    }
    MultipinRoute(const int netId) {
        this->netId = netId;
    }
    MultipinRoute(const int netId, const int gridNetclassId) {
        this->netId = netId;
        this->gridNetclassId = gridNetclassId;
    }

    void featuresToGridPaths();
    void gridPathLocationsToSegments();

    int getGridNetclassId() const { return gridNetclassId; }

    const std::vector<GridPath> &getGridPaths() const { return mGridPaths; }
    void clearGridPaths() { mGridPaths.clear(); }

    GridPath &getNewGridPath() {
        mGridPaths.push_back(GridPath{});
        return mGridPaths.back();
    }
    GridPin &getNewGridPin() {
        mGridPins.push_back(GridPin{});
        return mGridPins.back();
    }

    double getRoutedWirelength();
    int getRoutedNumVias();
    int getRoutedNumBends();
    double getCurTrackObstacleCost() const { return curTrackObstacleCost; }
    double getCurViaObstacleCost() const { return curViaObstacleCost; }
    double getCurNegTrackObstacleCost() const { return -curTrackObstacleCost; }
    double getCurNegViaObstacleCost() const { return -curViaObstacleCost; }

    void setCurTrackObstacleCost(const double &toc) { curTrackObstacleCost = toc; }
    void setCurViaObstacleCost(const double &voc) { curViaObstacleCost = voc; }
    void addCurTrackObstacleCost(const double &stoc) { curTrackObstacleCost += stoc; }
    void addCurViaObstacleCost(const double &svoc) { curViaObstacleCost += svoc; }

    friend class BoardGrid;
    friend class MultipinRoute;
    friend class GridBasedRouter;

   private:
    int netId = -1;
    int gridNetclassId = -1;
    float currentRouteCost = 0.0;
    std::vector<Location> features;  // Make this obsolete, please
    std::vector<GridPin> mGridPins;

    // Cost along with this net
    double curTrackObstacleCost = 0.0;
    double curViaObstacleCost = 0.0;

    //derived from features, doesn't guarantee updated
    // Please replace features with mGridPaths
    std::vector<GridPath> mGridPaths;
    //TODO
    // std::vector<Location> vias;
};

#endif