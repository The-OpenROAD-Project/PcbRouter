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

// TODO
// See router.h -> class net
struct NetClass {
    double ncClearance;
    double ncTraceWidth;
    double ncViaDiameter;
    double ncMicroViaDiameter;
};

// TODO
// See clearance matrix in the EAGLE/KiCad format
class GlobalParam {
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

    //BoardGrid
    static double gDiagonalCost;
    static double gLayerChangeCost;
    static double gViaInsertionCost;
    static double gTraceBasicCost;
    static double gPinObstacleCost;

    //Grid Setup
    static unsigned int inputScale;
    static unsigned int enlargeBoundary;
    static float gridFactor;  // For outputing

    //Routing Options
    static bool gViaUnderPad;

    //Outputfile
    static int gOutputPrecision;
    static string gOutputFolder;

    //Log
    static string gLogFolder;

    const static double gSqrt2;
    const static double gTan22_5;

    static int gSeed;

    static util::TimeUsage runTime;

    static void setFolders();
    static void setDesignRule();
    static void setLayerNum(int l) { gLayerNum = l; }

    static void showParam();
    static void showCurrentUsage(const string comment);
    static void showFinalUsage(const string comment);
    static void setUsageStart();
};

#endif
