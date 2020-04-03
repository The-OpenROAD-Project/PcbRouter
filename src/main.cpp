
#include "GridBasedRouter.h"
#include "frTime.h"
#include "kicadPcbDataBase.h"
#include "util.h"

int main(int argc, char *argv[]) {
    if (argc < 2) {
        std::cout << "Please provide input testcase filename." << std::endl;
        return 0;
    }

    util::showSysInfoComdLine(argc, argv);
    GlobalParam::setFolders();
    GlobalParam::setUsageStart();
    fr::frTime timeObj;

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
        router.set_grid_scale(atoi(argv[2]));
    }
    if (argc >= 4) {
        router.set_num_iterations(atoi(argv[3]));
    }
    if (argc >= 5) {
        router.set_enlarge_boundary(atoi(argv[4]));
    }
    if (argc >= 6) {
        router.set_layer_change_weight(atof(argv[5]));
    }
    if (argc >= 7) {
        router.set_track_obstacle_weight(atof(argv[6]));
    }
    if (argc >= 8) {
        router.set_track_obstacle_step_size(atof(argv[7]));
    }
    if (argc >= 9) {
        router.set_via_obstacle_step_size(atof(argv[8]));
    }
    // router.testRouterWithPinShape();
    router.initialization();
    // GND (20) to route on Bottom Layer
    // router.set_net_all_layers_pref_weights(20, 10);
    // router.set_net_layer_pref_weight(20, "Bottom", 0);

    router.route();

    // db.printRoutedSegmentsWLAndNumVias();

    std::cout << "routed WL: " << router.get_routed_wirelength()
              << ", routed # vias: " << router.get_routed_num_vias()
              << ", routed # bends: " << router.get_routed_num_bends() << std::endl;

    GlobalParam::showCurrentUsage("GridBasedRouter");
    GlobalParam::showFinalUsage("End of Program");

    timeObj.print();

    return 0;
}
