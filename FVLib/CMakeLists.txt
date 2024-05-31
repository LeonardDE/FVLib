cmake_minimum_required(VERSION 3.25)

set(CMAKE_CXX_STANDARD 20)

project(FVLib)

include_directories(Src Src/lib)

add_executable(FVLib 
#  Include files
  Src/Copter.h
  Src/DataParser.h
  Src/FV.h
  Src/GlobalSituation.h
  Src/MaterialPoint.h
  Src/Modeler.h
  Src/Obstacle.h
  Src/OutputData.h
  Src/ParamReader.h
  Src/Path.h
  Src/PathPoint.h
  Src/Point.h
  Src/TurnData.h
  Src/Vector3.h
  Src/global.h
  Src/json.hpp    
  Src/lib/plane.h

#  Source files
  Src/Copter.cpp
  Src/DataParser.cpp
  Src/FV.cpp
  Src/GlobalSituation.cpp
  Src/MaterialPoint.cpp
  Src/Modeler.cpp
  Src/OutputData.cpp
  Src/ParamReader.cpp
  Src/Path.cpp
  Src/PathPoint.cpp
  Src/Point.cpp
  Src/TurnData.cpp
  Src/Vector3.cpp
  Src/global.cpp
  Src/main.cpp
)


