#ifndef PCBROUTER_POST_PROCESSING_H
#define PCBROUTER_POST_PROCESSING_H

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

    void removeAcuteAngleBetweenGridPinsAndPaths(const vector<GridPin> gridPins, vector<GridPath> gridPaths);

   private:
};

#endif