#ifndef PCBROUTER_GLOBALPARAM_H
#define PCBROUTER_GLOBALPARAM_H

#include <math.h>
#include <stdlib.h>
#include <time.h>

#include <string>

#include "util.h"

using namespace std;

#define PI 3.14159265358979323846264338

#define BOUND_CHECKS

namespace pr {

using prIntCost = int;
using prFltCost = double;

}  // namespace pr

class GlobalParam {
   public:
    static int gLayerNum;
    static double gEpsilon;
    static bool g90DegreeMode;

    //BoardGrid
    static double gDiagonalCost;
    static double gWirelengthCost;
    static double gLayerChangeCost;

    //Obstacle cost
    static double gViaInsertionCost;
    static double gTraceBasicCost;
    static double gPinObstacleCost;

    //Step size of obstacle cost
    static double gStepViaObsCost;
    static double gStepTraObsCost;

    //Other costs
    static double gViaTouchBoundaryCost;
    static double gTraceTouchBoundaryCost;
    static double gViaForbiddenCost;

    //Grid Setup
    static unsigned int inputScale;
    static unsigned int enlargeBoundary;
    static float gridFactor;  // For outputing

    //Routing Options
    static bool gViaUnderPad;
    static bool gUseMircoVia;
    static unsigned int gNumRipUpReRouteIteration;

    //Outputfile
    static int gOutputPrecision;
    static string gOutputFolder;
    static bool gOutputDebuggingKiCadFile;
    static bool gOutputDebuggingGridValuesPyFile;
    static bool gOutputStackedMicroVias;

    //Log
    static string gLogFolder;

    const static double gSqrt2;
    const static double gTan22_5;

    static int gSeed;

    static util::TimeUsage runTime;

    static void setFolders();
    static void setLayerNum(int l) { gLayerNum = l; }

    static void showCurrentUsage(const string comment);
    static void showFinalUsage(const string comment);
    static void setUsageStart();
};

#endif
