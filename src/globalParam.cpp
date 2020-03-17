#include "globalParam.h"

int GlobalParam::gLayerNum = 3;
double GlobalParam::gEpsilon = 0.00000000000001;
bool GlobalParam::g90DegreeMode = true;
// BoardGrid
double GlobalParam::gDiagonalCost = 1.41421356237;  //Cost for path searching
double GlobalParam::gWirelengthCost = 1.0;          //Cost for path searching
double GlobalParam::gLayerChangeCost = 10.0;        //Cost for path searching
// Obstacle cost
double GlobalParam::gViaInsertionCost = 100.0;  //Cost added to the via grid
double GlobalParam::gTraceBasicCost = 50.0;     //10?     //Cost added to the base grid by traces
double GlobalParam::gPinObstacleCost = 1000.0;  //2000?   //Cost for obstacle pins added to the via (or/and) base grid
// Increment of obstacle cost
double GlobalParam::gStepViaObsCost = 0.0;  //10.0;  //Cost added to the via grid
double GlobalParam::gStepTraObsCost = 0.0;  //2.5;
// Other cost
double GlobalParam::gViaTouchBoundaryCost = 1000.0;
double GlobalParam::gTraceTouchBoundaryCost = 100000.0;
double GlobalParam::gViaForbiddenCost = 2000.0;
// Grid Setup
unsigned int GlobalParam::inputScale = 10;
unsigned int GlobalParam::enlargeBoundary = 0;  //from 10 -> 50
float GlobalParam::gridFactor = 0.1;            // 1/inputScale
// Routing Options
bool GlobalParam::gViaUnderPad = false;
bool GlobalParam::gUseMircoVia = true;
unsigned int GlobalParam::gNumRipUpReRouteIteration = 5;
// Outputfile
int GlobalParam::gOutputPrecision = 5;
string GlobalParam::gOutputFolder = "output";
bool GlobalParam::gOutputDebuggingKiCadFile = true;
bool GlobalParam::gOutputDebuggingGridValuesPyFile = true;
bool GlobalParam::gOutputStackedMicroVias = true;
// logfile
string GlobalParam::gLogFolder = "log";

int GlobalParam::gSeed = 1470295829;  //time(NULL);
const double GlobalParam::gSqrt2 = sqrt(2);
const double GlobalParam::gTan22_5 = tan(22.5 * PI / 180.0);

util::TimeUsage GlobalParam::runTime = util::TimeUsage();

void GlobalParam::setFolders() {
    if (!util::createDirectory(gOutputFolder)) {
        gOutputFolder = "";
    }
    if (!util::createDirectory(gLogFolder)) {
        gLogFolder = "";
    }
}

void GlobalParam::showCurrentUsage(const string comment) {
    runTime.showUsage(comment, util::TimeUsage::PARTIAL);
}

void GlobalParam::showFinalUsage(const string comment) {
    runTime.showUsage(comment, util::TimeUsage::FULL);
}

void GlobalParam::setUsageStart() {
    runTime.start(util::TimeUsage::PARTIAL);
}
