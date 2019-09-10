#ifndef GLOBALPARAM_H
#define GLOBALPARAM_H

#include <string>
#include <math.h>
#include <time.h>
#include <stdlib.h>
#include "util.h"

using namespace std;

#define PI 3.14159265358979323846264338

// TODO
// See router.h -> class net
struct NetClass
{
  double ncClearance;
  double ncTraceWidth;
  double ncViaDiameter;
  double ncMicroViaDiameter;
};

// TODO
// See clearance matrix in the EAGLE/KiCad format
class GlobalParam
{
public:
  static int gLayerNum;
  static double gWireWidth;
  static double gWireSpace;
  static double gWirePadSpace;

  static double gHalfWireWidth;
  static double gHWidthAndSpace;
  static double gHWidthAndWPSpace;
  static double gWire2Wire;
  static double gDisBetweenWire;

  static double gEpsilon;
  static bool g90DegreeMode;

  const static double gSqrt2;
  const static double gTan22_5;

  static int gSeed;

  static util::TimeUsage runTime;

  static void setDesignRule();
  static void setLayerNum(int l) { gLayerNum = l; }

  static void showParam();
  static void showCurrentUsage(const string comment);
  static void showFinalUsage(const string comment);
  static void setUsageStart();
};

#endif
