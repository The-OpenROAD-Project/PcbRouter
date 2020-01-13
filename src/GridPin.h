#ifndef PCBROUTER_GRID_PIN_H
#define PCBROUTER_GRID_PIN_H

#include <algorithm>
#include <vector>
#include "globalParam.h"
#include "point.h"

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

#endif