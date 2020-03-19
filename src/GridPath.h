#ifndef PCBROUTER_GRID_PATH_H
#define PCBROUTER_GRID_PATH_H

#include <algorithm>
#include <list>
#include <vector>
#include "globalParam.h"
#include "point.h"

class GridPath {
   public:
    //ctor
    GridPath() {}
    //dtor
    ~GridPath() {}

    const std::list<Location> &getSegments() const { return mSegments; }
    void removeRedundantPoints();
    double getRoutedWirelength();
    int getRoutedNumVias();
    int getRoutedNumBends();

    friend class BoardGrid;
    friend class MultipinRoute;

   private:
    std::list<Location> mSegments;  //TODO:: contains vias for now...
};

#endif