#include "global.h"
#include "Plane.h"
#include "Modeler.h"
#include "DataParser.h"
#include "OutputData.h"
#include "FV.h"

Modeler::Modeler(const string &in_file, const string &out_file)
{
  this->in_file = in_file;
  this->out_file = out_file;

  DataParser parser(in_file);

  // Собираем данные о ЛА
  parser.ParseFVToList(globalSituation);

  // Собираем данные для вычислений
  parser.ParseSolveDataFromJSON(timeStep, solve_part_count);

  // Ищем максимальное и минимальное значение времени
  start_time = INFINITY;
  end_time = -INFINITY;
  for (FV *&fv : globalSituation.FVs)
  {
    const FlightPlan &plan = fv->getBasePath();
    for (int i = plan.size() - 1; i >= 0; i--)
    {
      start_time = min(start_time, plan[i].arrivalTime);
      end_time = max(end_time, plan[i].arrivalTime);
    }
  }
}

void Modeler::startModeling()
{
  double timer = start_time;
  double h = timeStep / solve_part_count;

  // Creation of the output data structure
  for (auto &fv : globalSituation.FVs)
  {
    OutputFV out_fv;
    FVOutputState plane = fv->getOutputState();
    out_fv.parameters.type = plane.type;
    out_fv.parameters.name = plane.name;
    out_fv.parameters.navType = NavMethodTypeToName[fv->getNavigationType()];

    out_fv.parameters.plan.clear();
    const FlightPlan &plan = fv->getBasePath();
    for (int i = 0; i < plan.size(); i++)
    {
      out_fv.parameters.plan.push_back(plan[i]);
    }
    globalSituation.jsonData.FVs.push_back(out_fv);
  }

  // The global loop over time
  while (timer <= end_time)
  {

    // Moving the FVs
    for (int i = 0; i < globalSituation.FVs.size(); i++)
    {
      FV *fv = globalSituation.FVs[i];
      if (fv->getCurrentPath().doesInstantBelong(timer))
      {
        fv->next(h, timer);
        
        OutputFV &out_fv = globalSituation.jsonData.FVs[i];
        out_fv.dynamic_data.push_back(fv->getOutputPosition());
      }
    }

    timer += timeStep;
  }

  globalSituation.jsonData.dt = timeStep;
  writeJsonData(out_file, globalSituation.jsonData);
}
