
#include "kicadPcbDataBase.h"
#include "GridBasedRouter.h"
#include "util.h"

int main(int argc, char *argv[])
{
  if (argc < 2)
  {
    std::cout << "Please provide input testcase filename.";
    return 0;
  }

  util::showSysInfoComdLine(argc, argv);
  GlobalParam::setFolders();

  std::string designName = argv[1];
  std::cout << "Parsing design: " << designName << std::endl;
  kicadPcbDataBase db(designName);

  db.printComp();
  db.printInst();
  db.printNet();
  db.printPcbRouterInfo();

  std::cout << "Starting router..." << std::endl;
  GridBasedRouter router(db);
  router.test_router();

  return 0;
}
