#include <iostream>

#include <execinfo.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>

#include <thread>

#include "network/networkserver.h"
#include "tradr/ocuserver.h"
#include "uav/uavserver.h"

#include "tgui/terminalgui.h"

using namespace std;


void handler(int sig)
{

  void *array[10];
  size_t size;

  // get void*'s for all entries on the stack
  size = backtrace(array, 10);

  // print out all the frames to stderr
  fprintf(stderr, "Error: signal %d:\n", sig);
  backtrace_symbols_fd(array, size, STDERR_FILENO);

  exit(1);
}


void runRos(io_service* ios)
{
    ros::spin();
    ios->stop();
}

int main(int argc, char** argv)
{
    signal(SIGSEGV, handler);
    signal(SIGABRT, handler);

    TerminalGUI tgui;

    ros::init(argc, argv, "uav_bridge");

    io_service ioService;
    io_service::work ioWork(ioService);

    NetworkServer networkServer(&ioService);
    OCUServer ocuServer;
    UavServer uavServer(&ocuServer, &networkServer, &tgui);

    std::thread rosThread(runRos, &ioService);

    uavServer.start();

    ioService.run();

    cout << "\n" << endl;

    return 0;
}




