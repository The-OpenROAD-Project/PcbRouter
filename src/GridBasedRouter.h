// GridBasedRouter.h
#ifndef PCBROUTER_GRID_BASED_ROUTER_H
#define PCBROUTER_GRID_BASED_ROUTER_H

#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "BoardGrid.h"
#include "globalParam.h"
#include "kicadPcbDataBase.h"
#include "util.h"

class GridBasedRouter {
   public:
    // ctor
    GridBasedRouter(kicadPcbDataBase &db) : mDb(db) {}
    // dtor
    ~GridBasedRouter(){}

        [[deprecated]] void test_router();
    void testRouterWithPinAndKeepoutAvoidance();
    void testRouterWithAvoidanceAndVariousPadType();
    // bool outputResults2KiCadFile(std::vector<Route> &nets); // If needed
    bool outputResults2KiCadFile(std::vector<MultipinRoute> &nets);

   private:
    bool writeNets(std::vector<MultipinRoute> &multipinNets, std::ofstream &ofs);

    // Setup
    void setupBoardAndMappingStructure();

    // Pin Layers on Grid
    bool getGridLayers(const Pin &, std::vector<int> &layers);
    bool getGridLayers(const padstack &, const instance &, std::vector<int> &layers);
    // GridNetclass
    GridNetclass &getGridNetclass(const int gridNetclassId);

    // Utility
    int dbLengthToGridLength(const double dbLength) {
        return (int)ceil(dbLength * inputScale);
    }

    bool dbPointToGridPoint(const point_2d &dbPt, point_2d &gridPt);
    bool dbPointToGridPointCeil(const Point_2D<double> &dbPt, Point_2D<int> &gridPt);
    bool dbPointToGridPointFloor(const Point_2D<double> &dbPt, Point_2D<int> &gridPt);
    bool gridPointToDbPoint(const point_2d &gridPt, point_2d &dbPt);
    void addPinAvoidingCostToGrid(const Pin &, const float, const bool, const bool, const bool);
    void addPinAvoidingCostToGrid(const padstack &, const instance &, const float, const bool, const bool, const bool);

   private:
    BoardGrid mBg;
    kicadPcbDataBase &mDb;

    // Layer mapping
    std::vector<std::string> mGridLayerToName;
    std::unordered_map<std::string, int> mLayerNameToGrid;
    std::unordered_map<int, int> mDbLayerIdToGridLayer;

    // Netclass mapping
    std::vector<GridNetclass> mGridNetclasses;

    // TODO
    // Temporary value
    const float pinCost = 1000.0;
    // Put below stuff to globalParam:: ??
    double mMinX = std::numeric_limits<double>::max();
    double mMaxX = std::numeric_limits<double>::min();
    double mMinY = std::numeric_limits<double>::max();
    double mMaxY = std::numeric_limits<double>::min();
    // Take const to below?
    const unsigned int inputScale = 10;
    const unsigned int enlargeBoundary = 10;
    const float grid_factor = 0.1;  // For outputing
};

#endif