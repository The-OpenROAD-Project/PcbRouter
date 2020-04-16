#ifndef PCBROUTER_GRID_DIFF_PAIR_NET_H
#define PCBROUTER_GRID_DIFF_PAIR_NET_H

#include "MultipinRoute.h"

class GridDiffPairNet : public MultipinRoute {
   public:
    GridDiffPairNet() {
    }
    GridDiffPairNet(const int netId, const int gridNetclassId, const size_t numGridLayer)
        : MultipinRoute(netId, gridNetclassId, numGridLayer) {}
    ~GridDiffPairNet() {}

   private:
};

#endif