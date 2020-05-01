#ifndef PCBROUTER_GRID_DIFF_PAIR_NET_H
#define PCBROUTER_GRID_DIFF_PAIR_NET_H

#include "GridPin.h"
#include "MultipinRoute.h"

class GridDiffPairNet : public MultipinRoute {
   public:
    GridDiffPairNet(const int netId, const int gridDPNetclassId, const size_t numGridLayer,
                    MultipinRoute &net1, MultipinRoute &net2)
        : MultipinRoute(netId, gridDPNetclassId, numGridLayer),
          mNet1(net1),
          mNet2(net2) {}

    ~GridDiffPairNet() {}

    int getGridDiffPairNetclassId() const { return getGridNetclassId(); }
    MultipinRoute &getGridNet1() const { return mNet1; }
    MultipinRoute &getGridNet2() const { return mNet2; }

    void setupDiffPairGridPins(const int startLayerId, const int endLayerId);
    void separateGridPathsIntoTwo(const int traceClr, const int traceDiagClr, const int traceDiagOffset);
    void postProcessingGridPaths();

   private:
    void separateGridPath(const GridPath &path, const int traceClr, const int traceDiagClr, const int traceDiagOffset);
    void locBetweenHorizontalAndDiagonal(const Location &horizon, const Location &middle, const Location &diag,
                                         const int traceClr, const int traceDiagOffset, Location &middleL, Location &middleR);
    void locBetweenVerticalAndDiagonal(const Location &vertical, const Location &middle, const Location &diag,
                                       const int traceClr, const int traceDiagOffset, Location &middleL, Location &middleR);
    void locBetweenDiagonalAndDiagonal(const Location &diag1, const Location &middle, const Location &diag2,
                                       const int traceClr, const int traceDiagOffset, Location &middleL, Location &middleR);
    void locBetweenOrthogonalAndOrthogonal(const Location &orth1, const Location &middle, const Location &orth2,
                                           const int traceClr, const int traceDiagOffset, Location &middleL, Location &middleR);
    void endLocByStartEndLocations(const Location &start, const Location &end, const Location &startL, const Location &startR,
                                   const int traceClr, const int traceDiagOffset, Location &endL, Location &endR);
    void locsForStartEndLocationsWithoutMiddlePoints(const Location &start, const Location &end,
                                                     const int traceClr, const int traceDiagClr,
                                                     std::list<Location> &locsL, std::list<Location> &locsR);

   private:
    // int mGridDiffPairNetclassId = -1;
    MultipinRoute &mNet1;
    MultipinRoute &mNet2;

    std::vector<pair<int, int>> mGridPinPairsId;  //(GridPinId1, GridPinId2)
};

#endif