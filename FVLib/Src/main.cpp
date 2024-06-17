#include <iostream>
#include <string>
using namespace std;

#include "lib/cxxopts.h"
using namespace cxxopts;

#include "Modeler.h"
#include "FV.h"

/*
,
    {
      "name": "C++ Debug",
      "type": "cppdbg",
      "request": "launch",
      "program": "${command:cmake.launchTargetPath}",
      "cwd": "${workspaceFolder}/Tests/",
      "stopAtEntry": false,
      "args" : [ "-i", "../Tests/test3.json", "-o", "../Tests/output3.json" ],
      "customLaunchSetupCommands": [
        {
          "text": "target-run",
          "description": "run target",
          "ignoreFailures": false
        }
      ],
      "launchCompleteCommand": "exec-run",
      "linux": {
        "MIMode": "gdb",
        "miDebuggerPath": "/usr/bin/gdb"
      }
    }
    */
int main(int argc, char* argv[])
{
  Options options("UMVmodelling", "Program for modelling UFV motion and conflict avoidance");
  options.add_options()
    ("h, help", "Show the help instructions")
    ("i, input", "Name of the input file", value<string>())
    ("o, output", "Name of the output file", value<string>());

  ParseResult result = options.parse(argc, argv);

  //  Shows the help if needed
  if (result.count("h") > 0 || result.arguments().empty()) {
    cout << options.help({ "" });
    exit(0);
  }

  string in_file, out_file;

  if (result.count("input") > 0) {
    in_file = result["input"].as<string>();
  } else {
    cout << "Input file name is needed!" << endl;
    cout << options.help({ "" });
    exit(1);
  }

  if (result.count("output") > 0) {
    out_file = result["output"].as<string>();
  } else {
    cout << "Output file name is needed!" << endl;
    cout << options.help({ "" });
    exit(2);
  }

  Modeler modeler = Modeler(in_file, out_file);

  modeler.startModeling();

  return 0;
}
