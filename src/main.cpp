
#include "GridBasedRouter.h"
#include "kicadPcbDataBase.h"
#include "util.h"

int main(int argc, char *argv[]) {
    if (argc < 2) {
        std::cout << "Please provide input testcase filename." << std::endl;
        return 0;
    }

    // testShapeToCoords();
    // return 0;

    util::showSysInfoComdLine(argc, argv);
    GlobalParam::setFolders();
    GlobalParam::setUsageStart();

    std::string designName = argv[1];
    std::cout << "Parsing design: " << designName << std::endl;
    kicadPcbDataBase db(designName);

    db.printLayer();
    db.printComp();
    db.printInst();
    db.printNetclass();
    db.printNet();
    db.printFile();
    db.printPcbRouterInfo();

    GlobalParam::showCurrentUsage("Parser");
    GlobalParam::setUsageStart();

    std::cout << "Starting router..." << std::endl;
    if (argc >= 3) {
        GlobalParam::inputScale = abs(atoi(argv[2]));
        GlobalParam::gridFactor = 1.0 / (float)GlobalParam::inputScale;
    }
    if (argc >= 4) {
        GlobalParam::numRipUpReRouteIteration = abs(atoi(argv[3]));
    }
    if (argc >= 5) {
        GlobalParam::enlargeBoundary = abs(atoi(argv[4]));
    }
    srand(GlobalParam::gSeed);
    GridBasedRouter router(db);
    // router.testRouterWithPinShape();
    router.testRouterWithRipUpAndReroute();

    GlobalParam::showCurrentUsage("GridBasedRouter");
    GlobalParam::showFinalUsage("End of Program");

    return 0;
}
