#include <iostream>
#include <string>
using namespace std;

#include "lib/cxxopts.h"
using namespace cxxopts;

#include "Modeler.h"
#include "FV.h"

int main(int argc, char* argv[])
{
  system("pwd");
  
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

  // Getting input file name
  if (result.count("input") > 0) {
    in_file = result["input"].as<string>();
  } else {
    cout << "Input file name is needed!" << endl;
    cout << options.help({ "" });
    exit(1);
  }

  // Getting output file name
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
