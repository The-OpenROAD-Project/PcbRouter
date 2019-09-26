
#include "kicadPcbDataBase.h"
#include "GridBasedRouter.h"
#include "util.h"

int main(int argc, char *argv[])
{
  if (argc < 2)
  {
    std::cout << "Please provide input testcase filename." << std::endl;
    return 0;
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

  GlobalParam::showCurrentUsage("Parser");
  GlobalParam::setUsageStart();

  std::cout << "Starting router..." << std::endl;
  GridBasedRouter router(db);
  //router.test_router();
  router.testRouterWithPinAndKeepoutAvoidance();

  GlobalParam::showCurrentUsage("GridBasedRouter");
  GlobalParam::showFinalUsage("End of Program");

  return 0;
}
