#ifndef PCBROUTER_GRID_CELL_H
#define PCBROUTER_GRID_CELL_H

#include "globalParam.h"

enum GridCellType {
    VACANT,
    PAD,
    TRACE,
    VIA,
    KEEP_OUT,
    VIA_FORBIDDEN,
    TARGET_PIN  //Temporary flag, Should be a bool in GridCell? change to PAD_TARGET_PIN?
};

// struct NeighborCell {
// };

class GridCell {
   public:
    //ctor
    GridCell() {}
    //dtor
    ~GridCell() {}

    friend class BoardGrid;

   private:
    float baseCost = 0.0;     //Record Routed Nets's traces
    float workingCost = 0.0;  //Walked Cost

    // // Working cost breakdown
    // float overlappingCost = 0.0;  // cost of overlapping
    // float wirelengthCost = 0.0;   // walked distance
    // float historyCost = 0.0;      // overlapping/overflow cost from previous iteration
    int bendingCost = 0.0;  // # Bending
    // int viaCost = 0.0;            // # Vias

    // For incremental cost calculation
    float cachedTraceCost = -1.0;
    float cachedViaCost = -1.0;

    int cameFromId = -1;
    GridCellType cellType = VACANT;
    int numTraces = 0;
    //bool targetedPin = false;
    //bool viaForbidden = false;
};

#endif