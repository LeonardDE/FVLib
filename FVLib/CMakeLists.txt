cmake_minimum_required(VERSION 3.25)

set(CMAKE_CXX_STANDARD 20)

project(UMVmodelling)

include_directories(Src Src/lib)

add_executable(UMVmodelling 

#  Include files
  Src/Copter.h
  Src/DataParser.h
  Src/FV.h
  Src/GlobalSituation.h
  Src/MassPoint.h
  Src/Modeler.h
  Src/NavigationSegment.h
  Src/Obstacle.h
  Src/OutputData.h
  Src/Path.h
  Src/PathPoint.h
  Src/Plane.h
  Src/Point.h
  Src/TurnData.h
  Src/Vector3.h
  Src/global.h

  Src/lib/cxxopts.h
  Src/lib/json.h

#  Source files
  Src/Copter.cpp
  Src/DataParser.cpp
  Src/FV.cpp
  Src/GlobalSituation.cpp
  Src/MassPoint.cpp
  Src/Modeler.cpp
  Src/NavigationSegment.cpp
  Src/OutputData.cpp
  Src/Path.cpp
  Src/PathPoint.cpp
  Src/Plane.cpp
  Src/Point.cpp
  Src/TurnData.cpp
  Src/Vector3.cpp
  Src/global.cpp
  Src/main.cpp
)


