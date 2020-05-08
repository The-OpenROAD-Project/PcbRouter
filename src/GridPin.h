#ifndef PCBROUTER_GRID_PIN_H
#define PCBROUTER_GRID_PIN_H

#include <algorithm>
#include <vector>

#include "PcbRouterBoost.h"
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
    void setExpandedPinLL(Point_2D<int> &&point) { mExpandedPinLL = point; }
    void setExpandedPinUR(Point_2D<int> &&point) { mExpandedPinUR = point; }
    void setContractedPinLL(Point_2D<int> &point) { mContractedPinLL = point; }
    void setContractedPinUR(Point_2D<int> &point) { mContractedPinUR = point; }
    void setPinShape(const PinShape shape) { mPinShape = shape; }
    void setExpandedPinPolygon(const polygon_double_t poly) { mExpandedPinPolygon = poly; }
    void setPinPolygon(const polygon_double_t poly) { mPinPolygon = poly; }
    void addPinWithLayer(const Location pt) { pinWithLayers.push_back(pt); }
    void setPinCenter(const Point_2D<int> pt) { mPinCenter = pt; }
    void addPinShapeGridPoint(const Point_2D<int> &pt) { pinShapeToGrids.push_back(pt); }

    const Point_2D<int> &getPinLL() const { return pinLL; }
    const Point_2D<int> &getPinUR() const { return pinUR; }
    const Point_2D<int> &getExpandedPinLL() const { return mExpandedPinLL; }
    const Point_2D<int> &getExpandedPinUR() const { return mExpandedPinUR; }
    const Point_2D<int> &getContractedPinLL() const { return mContractedPinLL; }
    const Point_2D<int> &getContractedPinUR() const { return mContractedPinUR; }
    const std::vector<Location> &getPinWithLayers() const { return pinWithLayers; }
    const std::vector<Point_2D<int>> &getPinShapeToGrids() const { return pinShapeToGrids; }
    const polygon_double_t &getExpandedPinPolygon() const { return mExpandedPinPolygon; }
    const polygon_double_t &getPinPolygon() const { return mPinPolygon; }
    const PinShape getPinShape() const { return mPinShape; }
    const Point_2D<int> &getPinCenter() const { return mPinCenter; }
    bool isPinLayer(const int layerId) const {
        for (const auto &loc : this->pinWithLayers) {
            if (loc.z() == layerId) return true;
        }
        return false;
    }

   private:
    // TODO:: Change to layer index only
    std::vector<Location> pinWithLayers;
    Point_2D<int> mPinCenter;

    // Pin Shape
    std::vector<Point_2D<int>> pinShapeToGrids;
    Point_2D<int> pinLL = Point_2D<int>(0, 0);
    Point_2D<int> pinUR = Point_2D<int>(0, 0);
    PinShape mPinShape = PinShape::RECT;

    // Boost Polygon, absolute grid's coordinates
    polygon_double_t mPinPolygon;
    polygon_double_t mExpandedPinPolygon;

    // align the rounded bounding box of the expanded pin polygon
    Point_2D<int> mExpandedPinLL = Point_2D<int>(0, 0);
    Point_2D<int> mExpandedPinUR = Point_2D<int>(0, 0);
    // Contracted bounding box of the pin
    Point_2D<int> mContractedPinLL = Point_2D<int>(0, 0);
    Point_2D<int> mContractedPinUR = Point_2D<int>(0, 0);
};

#endif