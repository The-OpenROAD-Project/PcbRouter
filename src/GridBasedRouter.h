// GridBasedRouter.h
#ifndef PCBROUTER_GRID_BASED_ROUTER_H
#define PCBROUTER_GRID_BASED_ROUTER_H

#include <fstream>
#include <vector>
#include <string>
#include <cstdio>
#include <iostream>
#include "BoardGrid.h"
#include "kicadPcbDataBase.h"
#include "globalParam.h"
#include "util.h"

class GridBasedRouter
{
public:
  //ctor
  GridBasedRouter(kicadPcbDataBase &db) : mDb(db) {}
  //dtor
  ~GridBasedRouter() {}

  void test_router();
  bool outputResults2KiCadFile(std::vector<Route> &nets); // If needed
  bool outputResults2KiCadFile(std::vector<MultipinRoute> &nets);

private:
  bool writeNets(std::vector<MultipinRoute> &multipinNets, std::ofstream &ofs);

private:
  BoardGrid mBg;
  kicadPcbDataBase &mDb;

  // TODO
  // Put below stuff to globalParam:: ??
  double min_x = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::min();
  double min_y = std::numeric_limits<double>::max();
  double max_y = std::numeric_limits<double>::min();
  // Take const to below?
  const unsigned int inputScale = 10;
  const unsigned int enlargeBoundary = 10;
  const float grid_factor = 0.1; //For outputing
};

#endif