#ifndef PCBROUTER_POST_PROCESSING_H
#define PCBROUTER_POST_PROCESSING_H

#include <unordered_set>
#include <vector>

#include "GridPath.h"
#include "GridPin.h"
#include "MultipinRoute.h"
#include "PcbRouterBoost.h"
#include "globalParam.h"
#include "point.h"

class PostProcessing {
   public:
    PostProcessing() {
    }

    void removeAcuteAngleBetweenGridPinsAndPaths(const vector<GridPin> &gridPins, vector<GridPath> &gridPaths, const double gridWireWidth);

   private:
    bool isAcuteAngleBetweenPadAndSegment(const GridPin &gPin, const Location &inPt, const Location &outPt, const linestring_double_t &bgLs);
    bool isIntersectionPointOnTheBoxCorner(const point_double_t &point, const polygon_double_t &poly, const double wireWidth);
    void findIntersectionPointAndGetIntraPadSegments(const GridPin &gPin, const linestring_double_t &bgSeg, const Location &inPt, const Location &outPt, list<Location> &ret);

   private:
    double mGridWireWidth = 0.0;
};

#endif