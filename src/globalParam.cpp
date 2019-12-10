#include "globalParam.h"

int GlobalParam::gLayerNum = 3;
double GlobalParam::gWireWidth = 0;
double GlobalParam::gWireSpace = 0;
double GlobalParam::gWirePadSpace = 0;
double GlobalParam::gHalfWireWidth = 0;
double GlobalParam::gHWidthAndSpace = 0;
double GlobalParam::gHWidthAndWPSpace = 0;
double GlobalParam::gWire2Wire = 0;
double GlobalParam::gDisBetweenWire = 0;
double GlobalParam::gEpsilon = 0.00000000000001;
bool GlobalParam::g90DegreeMode = true;
// BoardGrid
double GlobalParam::gDiagonalCost = 1.41421356237;  //Cost for path searching
double GlobalParam::gLayerChangeCost = 10.0;        //Cost for path searching
double GlobalParam::gViaInsertionCost = 100.0;      //Cost added to the via grid
double GlobalParam::gTraceBasicCost = 10.0;         //10?    //Cost added to the base grid by traces
double GlobalParam::gPinObstacleCost = 1000.0;      //2000?   //Cost for obstacle pins added to the via (or/and) base grid
// Grid Setup
unsigned int GlobalParam::inputScale = 10;
unsigned int GlobalParam::enlargeBoundary = 10;  //from 10 -> 50
float GlobalParam::gridFactor = 0.1;             // 1/inputScale
// Routing Options
bool GlobalParam::gViaUnderPad = false;
unsigned int GlobalParam::gNumRipUpReRouteIteration = 5;
// Outputfile
int GlobalParam::gOutputPrecision = 5;
string GlobalParam::gOutputFolder = "output";
bool GlobalParam::gOutputDebuggingKiCadFile = true;
// logfile
string GlobalParam::gLogFolder = "log";

int GlobalParam::gSeed = 1470295829;  //time(NULL);
const double GlobalParam::gSqrt2 = sqrt(2);
const double GlobalParam::gTan22_5 = tan(22.5 * PI / 180.0);

util::TimeUsage GlobalParam::runTime = util::TimeUsage();

void GlobalParam::showParam() {
    cout << "=============PARAM============"
         << "\nDesign Related:"
         << "\ngLayerNum: " << gLayerNum
         << "\ngWireWidth: " << gWireWidth
         << "\ngWireSpace: " << gWireSpace
         << "\ngWirePadSpace: " << gWirePadSpace
         << "\n\nDesign Related (derived by info. above):"
         << "\ngHalfWireWidth: " << gHalfWireWidth
         << "\ngHWidthAndSpace: " << gHWidthAndSpace
         << "\ngHWidthAndWPSpace: " << gHWidthAndWPSpace
         << "\ngWire2Wire: " << gWire2Wire
         << "\ngDisBetweenWire: " << gDisBetweenWire
         << "\n\nAlgorithm Related:"
         << "\ngEpsilon: " << gEpsilon
         << "\ngSeed: " << gSeed << endl;
}

void GlobalParam::setDesignRule() {
    gHalfWireWidth = gWireWidth / 2.0;
    gHWidthAndSpace = gHalfWireWidth + gWireSpace;
    gHWidthAndWPSpace = gHalfWireWidth + gWirePadSpace;
    gWire2Wire = gWireWidth + gWireSpace;
    gDisBetweenWire = gWireWidth + gWireSpace;

    srand(GlobalParam::gSeed);
}

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
