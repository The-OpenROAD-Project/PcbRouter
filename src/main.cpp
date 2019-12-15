
#include <limits>
#include "GridBasedRouter.h"
#include "kicadPcbDataBase.h"
#include "util.h"

int main(int argc, char *argv[]) {
    if (argc < 2) {
        std::cout << "Please provide input testcase filename." << std::endl;
        return 0;
    }

    std::cout << "float\t"
              << std::numeric_limits<float>::lowest() << '\t'
              << std::numeric_limits<float>::min() << '\t'
              << std::numeric_limits<float>::min() - 100000 << '\t'
              << std::numeric_limits<float>::min() + 100000 << '\t'
              << std::numeric_limits<float>::max() << '\t'
              << std::numeric_limits<float>::max() - 100000 << '\t'
              << std::numeric_limits<float>::max() + 100000 << '\t'
              << std::numeric_limits<float>::infinity() << '\t'
              << std::numeric_limits<float>::infinity() + 100000 << '\t'
              << '\n';

    float cost =0;
    for(int i=0; i<10000; ++i){
        cost += 10000000000000000000000000000000000000.0;
        std::cout << cost <<std::endl;
    }

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
    db.printDesignStatistics();

    // Report current WL & # vias
    db.printRoutedSegmentsWLAndNumVias();

    // Remove all the routed nets
    db.removeRoutedSegmentsAndVias();

    GlobalParam::showCurrentUsage("Parser");
    GlobalParam::setUsageStart();

    std::cout << "Starting router..." << std::endl;
    srand(GlobalParam::gSeed);
    GridBasedRouter router(db);

    if (argc >= 3) {
        router.set_input_scale(atoi(argv[2]));
    }
    if (argc >= 4) {
        router.set_num_ripup_reroute_iteration(atoi(argv[3]));
    }
    if (argc >= 5) {
        router.set_enlarge_boundary(atoi(argv[4]));
    }
    if (argc >= 6) {
        router.set_layer_change_cost(atof(argv[5]));
    }
    // router.testRouterWithPinShape();
    router.route_all_net_with_ripup_and_reroute();

    db.printRoutedSegmentsWLAndNumVias();

    GlobalParam::showCurrentUsage("GridBasedRouter");
    GlobalParam::showFinalUsage("End of Program");

    return 0;
}
