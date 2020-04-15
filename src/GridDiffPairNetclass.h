#ifndef PCBROUTER_GRID_DIFF_PAIR_NETCLASS_H
#define PCBROUTER_GRID_DIFF_PAIR_NETCLASS_H

#include "GridNetclass.h"

class GridDiffPairNetclass : public GridNetclass {
   public:
    //ctor
    GridDiffPairNetclass(const int id = -1,
                         const int clearance = 0,
                         const int trace_width = 0,
                         const int via_dia = 0,
                         const int via_drill = 0,
                         const int uvia_dia = 0,
                         const int uvia_drill = 0,
                         const int first_base_gnc = -1,
                         const int second_base_gnc = -1)
        : GridNetclass(id, clearance, trace_width, via_dia, via_drill, uvia_dia, uvia_drill),
          mFirstBaseGridNetclassId(first_base_gnc),
          mSecondBaseGridNetclassId(second_base_gnc) {}
    //dtor
    ~GridDiffPairNetclass() {}

    int getFirstBaseGridNetclassId() { return mFirstBaseGridNetclassId; }
    int getSecondBaseGridNetclassId() { return mSecondBaseGridNetclassId; }

   private:
    int mFirstBaseGridNetclassId = -1;
    int mSecondBaseGridNetclassId = -1;
};

#endif