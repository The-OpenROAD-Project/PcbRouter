// GridBasedRouter.h
#ifndef GRID_BASED_ROUTER_H
#define GRID_BASED_ROUTER_H

#include "BoardGrid.h"
#include "kicadPcbDataBase.h"

class GridBasedRouter
{
public:
  //ctor
  GridBasedRouter() {}
  //dtor
  ~GridBasedRouter() {}

  void test_router(kicadPcbDataBase &db);

private:
  BoardGrid bg;
};

#endif