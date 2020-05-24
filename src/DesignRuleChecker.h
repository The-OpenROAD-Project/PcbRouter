#ifndef PCBROUTER_DESIGN_RULE_CHECKER_H
#define PCBROUTER_DESIGN_RULE_CHECKER_H

#include <vector>

#include "PcbRouterBoost.h"
#include "globalParam.h"
#include "kicadPcbDataBase.h"
#include "point.h"

#define INPUT_PRECISION 1000  //For DeepPCB cases

class DesignRuleChecker {
   public:
    DesignRuleChecker(kicadPcbDataBase &db) : mDb(db) {}
    ~DesignRuleChecker() {}

    int checkAcuteAngleViolationBetweenTracesAndPads();
    int checkTJunctionViolation();
    void setAcuteAngleTol(double _tol) { mAcuteAngleTol = _tol; }
    void setInputPrecision(int _ip) { mInputPrecision = _ip; }

   private:
    bool isPadstackAndSegmentHaveSameLayer(instance &inst, padstack &pad, Segment &seg);
    bool isIntersectionPointOnTheBoxCorner(point_double_t &point, polygon_double_t &poly, double wireWidth, double tol = 0.0);
    bool isSegmentTouchAPoint(linestring_double_t &ls, point_double_t &point, double wireWidth);
    bool isOrthogonalSegment(points_2d &points);
    bool isDiagonalSegment(points_2d &points);
    bool isOrthogonalSegment(Point_2D<long long> &pt1, Point_2D<long long> &pt2);
    bool isDiagonalSegment(Point_2D<long long> &pt1, Point_2D<long long> &pt2);

   private:
    kicadPcbDataBase &mDb;

    double mAcuteAngleTol = 0.1;
    int mInputPrecision = 1000;  //1000 for DeepPCB cases
};

#endif