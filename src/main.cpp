#include "kicadPcbDataBase.h"
#include "GridBasedRouter.h"

int main(int argc, char *argv[])
{
  if(argc < 2) {
    std::cout << "Please provide input testcase filename.";
    return 0;
  }

  std::string designName = argv[1];
  kicadPcbDataBase db(designName);

  db.printComp();
  db.printInst();
  db.printNet();
  db.printPcbRouterInfo();

  //test_multipin();
  //test_router(db);
  GridBasedRouter router;
  router.test_router(db);

  return 0;
}