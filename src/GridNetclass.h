#ifndef PCBROUTER_GRID_NETCLASS_H
#define PCBROUTER_GRID_NETCLASS_H

#include <algorithm>
#include <vector>
#include "IncrementalSearchGrids.h"
#include "globalParam.h"
#include "point.h"

class GridNetclass {
   public:
    //ctor
    GridNetclass(const int id = -1,
                 const int clearance = 0,
                 const int trace_width = 0,
                 const int via_dia = 0,
                 const int via_drill = 0,
                 const int uvia_dia = 0,
                 const int uvia_drill = 0)
        : m_id(id), m_clearance(clearance), m_trace_width(trace_width), m_via_dia(via_dia), m_via_drill(via_drill), m_uvia_dia(uvia_dia), m_uvia_drill(uvia_drill) {}
    //dtor
    ~GridNetclass() {}

    int getId() { return m_id; }
    int getClearance() { return m_clearance; }
    int getDiagonalClearance() { return m_clearance_diagonal; }
    int getTraceWidth() { return m_trace_width; }
    int getDiagonalTraceWidth() { return m_trace_width_diagonal; }
    int getHalfTraceWidth() { return m_half_trace_width; }
    int getHalfDiagonalTraceWidth() { return m_half_trace_width_diagonal; }
    int getViaDia() { return m_via_dia; }
    int getHalfViaDia() { return m_half_via_dia; }
    int getHalfMicroViaDia() { return m_half_uvia_dia; }
    int getViaDrill() { return m_via_drill; }
    int getMicroViaDia() { return m_uvia_dia; }
    int getMicroViaDrill() { return m_uvia_drill; }
    int getViaExpansion() { return m_via_expansion; }
    static int getObstacleExpansion() { return m_obstacle_expansion; }
    int getTraceExpansion() { return m_trace_expansion; }
    int getDiagonalTraceExpansion() { return m_trace_expansion_diagonal; }
    // Setup Derived
    void setHalfTraceWidth(const int halfTraWid) { m_half_trace_width = halfTraWid; }
    void setHalfViaDia(const int halfViaDia) { m_half_via_dia = halfViaDia; }
    void setHalfMicroViaDia(const int halfMicroViaDia) { m_half_uvia_dia = halfMicroViaDia; }
    // Derived diagonal cases values
    void setDiagonalTraceWidth(const int diagonalTraWid) { m_trace_width_diagonal = diagonalTraWid; }
    void setHalfDiagonalTraceWidth(const int halfDiagonalTraWid) { m_half_trace_width_diagonal = halfDiagonalTraWid; }
    void setDiagonalClearance(const int diagonalClr) { m_clearance_diagonal = diagonalClr; }
    void setViaExpansion(const int viaExp) { m_via_expansion = viaExp; }
    void setTraceExpansion(const int traExp) { m_trace_expansion = traExp; }
    static void setObstacleExpansion(const int obsExp) { m_obstacle_expansion = obsExp; }
    void setDiagonalTraceExpansion(const int traExp) { m_trace_expansion_diagonal = traExp; }
    // Via shape
    void addViaShapeGridPoint(const Point_2D<int> &pt) { mViaShapeToGrids.push_back(pt); }
    void setViaShapeGrids(const std::vector<Point_2D<int>> &grids) { mViaShapeToGrids = grids; }
    const std::vector<Point_2D<int>> &getViaShapeToGrids() const { return mViaShapeToGrids; }
    // Trace-end shape
    void setTraceEndShapeGrids(const std::vector<Point_2D<int>> &grids) { mTraceEndShapeToGrids = grids; }
    const std::vector<Point_2D<int>> &getTraceEndShapeToGrids() const { return mTraceEndShapeToGrids; }
    // Trace searching space
    void setTraceSearchingSpaceToGrids(const std::vector<Point_2D<int>> &grids) { mTraceSearchingSpaceToGrids = grids; }
    const std::vector<Point_2D<int>> &getTraceSearchingSpaceToGrids() const { return mTraceSearchingSpaceToGrids; }
    // Via searching space
    void setViaSearchingSpaceToGrids(const std::vector<Point_2D<int>> &grids) { mViaSearchingSpaceToGrids = grids; }
    const std::vector<Point_2D<int>> &getViaSearchingSpaceToGrids() const { return mViaSearchingSpaceToGrids; }
    // Incremental searching grids
    IncrementalSearchGrids &getTraceIncrementalSearchGrids() { return mTraceIncrementalSearchGrids; }
    IncrementalSearchGrids &getViaIncrementalSearchGrids() { return mViaIncrementalSearchGrids; }
    // Setup the incremental search grids
    void setupViaIncrementalSearchGrids();
    void setupTraceIncrementalSearchGrids();
    void setupIncrementalSearchGrids(const std::vector<Point_2D<int>> &searchGrids, IncrementalSearchGrids &incrementalSearchGrids);

   private:
    void getAddDedSearchGrids(const std::vector<Point_2D<int>> &searchGrids, const std::vector<Point_2D<int>> &shiftedSearchGrids, std::vector<Point_2D<int>> &add, std::vector<Point_2D<int>> &ded);

   private:
    int m_id = -1;
    int m_clearance = 0;
    int m_trace_width = 0;
    int m_via_dia = 0;
    int m_via_drill = 0;
    int m_uvia_dia = 0;
    int m_uvia_drill = 0;

    // Derived
    int m_half_trace_width = 0;
    int m_trace_width_diagonal = 0;
    int m_half_trace_width_diagonal = 0;
    int m_half_via_dia = 0;
    int m_half_uvia_dia = 0;
    int m_clearance_diagonal = 0;

    int m_trace_expansion = 0;
    int m_trace_expansion_diagonal = 0;
    int m_via_expansion = 0;

    // Refactor below static memeber....
    static int m_obstacle_expansion;

    // Via shape, relative to the via center grid
    std::vector<Point_2D<int>> mViaShapeToGrids;
    // Trace-end shape, relative to the trace center grid
    std::vector<Point_2D<int>> mTraceEndShapeToGrids;
    // Trace searching space when caluclating grid cost, relative to trace center grid
    std::vector<Point_2D<int>> mTraceSearchingSpaceToGrids;
    // Via searching space when caluclating grid cost, relative to via center grid
    std::vector<Point_2D<int>> mViaSearchingSpaceToGrids;

    IncrementalSearchGrids mTraceIncrementalSearchGrids;
    IncrementalSearchGrids mViaIncrementalSearchGrids;
};

#endif