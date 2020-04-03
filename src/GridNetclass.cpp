#include "GridNetclass.h"

int GridNetclass::m_obstacle_expansion = 0;

void GridNetclass::setupViaIncrementalSearchGrids() {
    this->setupIncrementalSearchGrids(this->mViaSearchingSpaceToGrids, this->mViaIncrementalSearchGrids);
}
void GridNetclass::setupTraceIncrementalSearchGrids() {
    this->setupIncrementalSearchGrids(this->mTraceSearchingSpaceToGrids, this->mTraceIncrementalSearchGrids);
}

void GridNetclass::setupIncrementalSearchGrids(const std::vector<Point_2D<int>> &searchGrids, IncrementalSearchGrids &incrementalSearchGrids) {
    // Right
    auto searchGridsR = searchGrids;
    // Shift the grids
    for (auto &&pt : searchGridsR) {
        pt.m_x += 1;
    }
    getAddDedSearchGrids(searchGrids, searchGridsR, incrementalSearchGrids.setRightAddGrids(), incrementalSearchGrids.setRightDedGrids());

    // Left
    auto searchGridsL = searchGrids;
    // Shift the grids
    for (auto &&pt : searchGridsL) {
        pt.m_x -= 1;
    }
    getAddDedSearchGrids(searchGrids, searchGridsL, incrementalSearchGrids.setLeftAddGrids(), incrementalSearchGrids.setLeftDedGrids());
    // // Debugging
    // std::cout << "Left additional relative trace searching grids points: " << std::endl;
    // for (const auto &pt : addL) {
    //     std::cout << pt << std::endl;
    // }
    // std::cout << "Left deduction relative trace searching grids points: " << std::endl;
    // for (const auto &pt : dedL) {
    //     std::cout << pt << std::endl;
    // }

    // Forward
    auto searchGridsF = searchGrids;
    // Shift the grids
    for (auto &&pt : searchGridsF) {
        pt.m_y += 1;
    }
    getAddDedSearchGrids(searchGrids, searchGridsF, incrementalSearchGrids.setForwardAddGrids(), incrementalSearchGrids.setForwardDedGrids());

    // Backward
    auto searchGridsB = searchGrids;
    // Shift the grids
    for (auto &&pt : searchGridsB) {
        pt.m_y -= 1;
    }
    getAddDedSearchGrids(searchGrids, searchGridsB, incrementalSearchGrids.setBackwardAddGrids(), incrementalSearchGrids.setBackwardDedGrids());

    // LB
    auto searchGridsLB = searchGrids;
    // Shift the grids
    for (auto &&pt : searchGridsLB) {
        pt.m_x -= 1;
        pt.m_y -= 1;
    }
    getAddDedSearchGrids(searchGrids, searchGridsLB, incrementalSearchGrids.setLBAddGrids(), incrementalSearchGrids.setLBDedGrids());

    // LF
    auto searchGridsLF = searchGrids;
    // Shift the grids
    for (auto &&pt : searchGridsLF) {
        pt.m_x -= 1;
        pt.m_y += 1;
    }
    getAddDedSearchGrids(searchGrids, searchGridsLF, incrementalSearchGrids.setLFAddGrids(), incrementalSearchGrids.setLFDedGrids());

    // RB
    auto searchGridsRB = searchGrids;
    // Shift the grids
    for (auto &&pt : searchGridsRB) {
        pt.m_x += 1;
        pt.m_y -= 1;
    }
    getAddDedSearchGrids(searchGrids, searchGridsRB, incrementalSearchGrids.setRBAddGrids(), incrementalSearchGrids.setRBDedGrids());

    // RF
    auto searchGridsRF = searchGrids;
    // Shift the grids
    for (auto &&pt : searchGridsRF) {
        pt.m_x += 1;
        pt.m_y += 1;
    }
    getAddDedSearchGrids(searchGrids, searchGridsRF, incrementalSearchGrids.setRFAddGrids(), incrementalSearchGrids.setRFDedGrids());
}

void GridNetclass::getAddDedSearchGrids(const std::vector<Point_2D<int>> &searchGrids, const std::vector<Point_2D<int>> &shiftedSearchGrids, std::vector<Point_2D<int>> &add, std::vector<Point_2D<int>> &ded) {
    // Get add
    for (const auto &pt : shiftedSearchGrids) {
        auto it = std::find(searchGrids.begin(), searchGrids.end(), pt);
        if (it == searchGrids.end()) {
            add.push_back(pt);
        }
    }
    // Get ded
    for (const auto &pt : searchGrids) {
        auto it = std::find(shiftedSearchGrids.begin(), shiftedSearchGrids.end(), pt);
        if (it == shiftedSearchGrids.end()) {
            ded.push_back(pt);
        }
    }
}
