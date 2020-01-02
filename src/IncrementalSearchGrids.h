#ifndef PCBROUTER_INCREMENTAL_SEARCH_GRIDS_H
#define PCBROUTER_INCREMENTAL_SEARCH_GRIDS_H

#include <vector>
#include "globalParam.h"
#include "point.h"

class IncrementalSearchGrids {
   public:
    IncrementalSearchGrids() {}
    ~IncrementalSearchGrids() {}

    const std::vector<Point_2D<int>> &getLeftAddGrids() const { return mAdditionGridsL; }
    const std::vector<Point_2D<int>> &getLeftDedGrids() const { return mDeductionGridsL; }
    const std::vector<Point_2D<int>> &getRightAddGrids() const { return mAdditionGridsR; }
    const std::vector<Point_2D<int>> &getRightDedGrids() const { return mDeductionGridsR; }
    const std::vector<Point_2D<int>> &getForwardAddGrids() const { return mAdditionGridsF; }
    const std::vector<Point_2D<int>> &getForwardDedGrids() const { return mDeductionGridsF; }
    const std::vector<Point_2D<int>> &getBackwardAddGrids() const { return mAdditionGridsB; }
    const std::vector<Point_2D<int>> &getBackwardDedGrids() const { return mDeductionGridsB; }

    const std::vector<Point_2D<int>> &getLBAddGrids() const { return mAdditionGridsLB; }
    const std::vector<Point_2D<int>> &getLBDedGrids() const { return mDeductionGridsLB; }
    const std::vector<Point_2D<int>> &getRBAddGrids() const { return mAdditionGridsRB; }
    const std::vector<Point_2D<int>> &getRBDedGrids() const { return mDeductionGridsRB; }
    const std::vector<Point_2D<int>> &getLFAddGrids() const { return mAdditionGridsLF; }
    const std::vector<Point_2D<int>> &getLFDedGrids() const { return mDeductionGridsLF; }
    const std::vector<Point_2D<int>> &getRFAddGrids() const { return mAdditionGridsRF; }
    const std::vector<Point_2D<int>> &getRFDedGrids() const { return mDeductionGridsRF; }

    std::vector<Point_2D<int>> &setLeftAddGrids() { return mAdditionGridsL; }
    std::vector<Point_2D<int>> &setLeftDedGrids() { return mDeductionGridsL; }
    std::vector<Point_2D<int>> &setRightAddGrids() { return mAdditionGridsR; }
    std::vector<Point_2D<int>> &setRightDedGrids() { return mDeductionGridsR; }
    std::vector<Point_2D<int>> &setForwardAddGrids() { return mAdditionGridsF; }
    std::vector<Point_2D<int>> &setForwardDedGrids() { return mDeductionGridsF; }
    std::vector<Point_2D<int>> &setBackwardAddGrids() { return mAdditionGridsB; }
    std::vector<Point_2D<int>> &setBackwardDedGrids() { return mDeductionGridsB; }

    std::vector<Point_2D<int>> &setLBAddGrids() { return mAdditionGridsLB; }
    std::vector<Point_2D<int>> &setLBDedGrids() { return mDeductionGridsLB; }
    std::vector<Point_2D<int>> &setRBAddGrids() { return mAdditionGridsRB; }
    std::vector<Point_2D<int>> &setRBDedGrids() { return mDeductionGridsRB; }
    std::vector<Point_2D<int>> &setLFAddGrids() { return mAdditionGridsLF; }
    std::vector<Point_2D<int>> &setLFDedGrids() { return mDeductionGridsLF; }
    std::vector<Point_2D<int>> &setRFAddGrids() { return mAdditionGridsRF; }
    std::vector<Point_2D<int>> &setRFDedGrids() { return mDeductionGridsRF; }

   private:
    // For incremental costs
    std::vector<Point_2D<int>> mAdditionGridsR;
    std::vector<Point_2D<int>> mDeductionGridsR;
    std::vector<Point_2D<int>> mAdditionGridsL;
    std::vector<Point_2D<int>> mDeductionGridsL;
    std::vector<Point_2D<int>> mAdditionGridsF;
    std::vector<Point_2D<int>> mDeductionGridsF;
    std::vector<Point_2D<int>> mAdditionGridsB;
    std::vector<Point_2D<int>> mDeductionGridsB;

    std::vector<Point_2D<int>> mAdditionGridsRB;
    std::vector<Point_2D<int>> mDeductionGridsRB;
    std::vector<Point_2D<int>> mAdditionGridsRF;
    std::vector<Point_2D<int>> mDeductionGridsRF;
    std::vector<Point_2D<int>> mAdditionGridsLF;
    std::vector<Point_2D<int>> mDeductionGridsLF;
    std::vector<Point_2D<int>> mAdditionGridsLB;
    std::vector<Point_2D<int>> mDeductionGridsLB;
};

#endif