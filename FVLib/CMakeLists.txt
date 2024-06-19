cmake_minimum_required(VERSION 3.25)

set(CMAKE_CXX_STANDARD 20)

project(UMVmodelling)

include_directories(Src Src/lib)

add_executable(UMVmodelling 

#  Include files
  Src/CircularSegment.h
  Src/Copter.h
  Src/DataParser.h
  Src/DecisionMaker.h
  Src/FlightPlan.h
  Src/FV.h
  Src/GlobalSituation.h
  Src/MassPoint.h
  Src/Modeler.h
  Src/NavigationPlan.h
  Src/Obstacle.h
  Src/OutputData.h
  Src/PathPoint.h
  Src/Plane.h
  Src/RectilinearSegment.h
  Src/Vector3.h
  Src/global.h

  Src/lib/cxxopts.h
  Src/lib/json.h

#  Source files
  Src/CircularSegment.cpp
  Src/Copter.cpp
  Src/DataParser.cpp
  Src/DecisionMaker.cpp
  Src/FlightPlan.cpp
  Src/FV.cpp
  Src/GlobalSituation.cpp
  Src/MassPoint.cpp
  Src/Modeler.cpp
  Src/NavigationPlan.cpp
  Src/OutputData.cpp
  Src/PathPoint.cpp
  Src/Plane.cpp
  Src/RectilinearSegment.cpp
  Src/Vector3.cpp
  Src/global.cpp
  Src/main.cpp
)


