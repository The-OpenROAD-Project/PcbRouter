#ifndef PCBROUTER_GRID_PIN_H
#define PCBROUTER_GRID_PIN_H

#include <algorithm>
#include <vector>

#include "globalParam.h"
#include "point.h"

class GridPin {
   public:
    enum class PinShape {
        RECT,
        ROUNDRECT,
        CIRCLE,
        OVAL,
        TRAPEZOID
    };

    //ctor
    GridPin() {}
    //dtor
    ~GridPin() {}

    friend class BoardGrid;
    friend class MultipinRoute;
    friend class GridBasedRouter;

    void setPinLL(Point_2D<int> &point) { pinLL = point; }
    void setPinUR(Point_2D<int> &point) { pinUR = point; }
    void setPinShape(const PinShape shape) { mPinShape = shape; }
    void addPinWithLayer(const Location pt) { pinWithLayers.push_back(pt); }
    void addPinShapeGridPoint(const Point_2D<int> &pt) { pinShapeToGrids.push_back(pt); }

    const Point_2D<int> &getPinLL() const { return pinLL; }
    const Point_2D<int> &getPinUR() const { return pinUR; }
    const std::vector<Location> &getPinWithLayers() const { return pinWithLayers; }
    const std::vector<Point_2D<int>> &getPinShapeToGrids() const { return pinShapeToGrids; }

   private:
    // TODO:: Change to layer index only
    std::vector<Location> pinWithLayers;

    // Pin Shape
    std::vector<Point_2D<int>> pinShapeToGrids;
    Point_2D<int> pinLL = Point_2D<int>(0, 0);
    Point_2D<int> pinUR = Point_2D<int>(0, 0);
    PinShape mPinShape = PinShape::RECT;
};

#endif