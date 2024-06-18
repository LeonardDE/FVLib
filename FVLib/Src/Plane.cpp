#include "Plane.h"

// Structure to convert a string to the FVType enum
map<string, FVType> FVNameToType = {
  { "MassPoint"       ,  MASSPOINT         },
  { "Copter"          ,  COPTER            },
  { "HelicopterAcc"   ,  HELICOPTER_ACC    },
  { "HelicopterSignal",  HELICOPTER_SIGNAL },
  { "AircraftAcc"     ,  AIRCRAFT_ACC      },
  { "AircraftSignal"  ,  AIRCRAFT_SIGNAL   }
};

// Structure to convert a FVType enum to the string
map<FVType, string> FVTypeToName = {
  { MASSPOINT        ,  "MassPoint"        },
  { COPTER           ,  "COPTER"           },
  { HELICOPTER_ACC   ,  "HelicopterAcc"    },
  { HELICOPTER_SIGNAL,  "HelicopterSignal" },
  { AIRCRAFT_ACC     ,  "AircraftAcc"      },
  { AIRCRAFT_SIGNAL  ,  "AircraftSignal"   }
}; 
