#ifndef PCBROUTER_DESIGN_RULE_CHECKER_H
#define PCBROUTER_DESIGN_RULE_CHECKER_H

#include <string>
#include <vector>

#include "PcbRouterBoost.h"
#include "globalParam.h"
#include "kicadPcbDataBase.h"
#include "point.h"

#define INPUT_PRECISION 1000  //For DeepPCB cases

struct TJunctionPatch {
    polygon_double_t poly;
    std::string layer;
    int netId;
};

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
    bool areConnectedSegments(linestring_double_t &ls1, linestring_double_t &ls2);
    bool isSegmentTouchAPoint(linestring_double_t &ls, point_double_t &point, double wireWidth);
    bool isOrthogonalSegment(points_2d &points);
    bool isDiagonalSegment(points_2d &points);
    bool isOrthogonalSegment(Point_2D<long long> &pt1, Point_2D<long long> &pt2);
    bool isDiagonalSegment(Point_2D<long long> &pt1, Point_2D<long long> &pt2);
    void addTJunctionPatch(const point_double_t &point, const linestring_double_t &ls1, const linestring_double_t &ls2, const double size, polygon_double_t &patchPoly);
    void printTJunctionPatchToKiCadZone(std::vector<TJunctionPatch> &patches);

   private:
    kicadPcbDataBase &mDb;

    double mAcuteAngleTol = 0.1;
    int mInputPrecision = 1000;  //1000 for DeepPCB cases
    double mEpsilon = 0.001;
    double mTJunctionEpsilon = 0.001;
};

#endif