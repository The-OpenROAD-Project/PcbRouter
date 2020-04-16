#ifndef PCBROUTER_GRID_DIFF_PAIR_NET_H
#define PCBROUTER_GRID_DIFF_PAIR_NET_H

#include "MultipinRoute.h"

class GridDiffPairNet : public MultipinRoute {
   public:
    GridDiffPairNet(const int netId, const int gridNetclassId, const size_t numGridLayer,
                    const int _gridDPNcId, MultipinRoute &net1, MultipinRoute &net2)
        : MultipinRoute(netId, gridNetclassId, numGridLayer),
          mGridDiffPairNetclassId(_gridDPNcId),
          mNet1(net1),
          mNet2(net2) {}

    ~GridDiffPairNet() {}

    int getGridDiffPairNetclassId() const { return mGridDiffPairNetclassId; }
    MultipinRoute &getGridNet1() const { return mNet1;}
    MultipinRoute &getGridNet2() const { return mNet2;}

   private:
    int mGridDiffPairNetclassId = -1;
    MultipinRoute &mNet1;
    MultipinRoute &mNet2;
};

#endif