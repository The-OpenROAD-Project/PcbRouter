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

   private:
    // int mGridDiffPairNetclassId = -1;
    MultipinRoute &mNet1;
    MultipinRoute &mNet2;

    std::vector<pair<int, int>> mGridPinPairsId;  //(GridPinId1, GridPinId2)
};

#endif