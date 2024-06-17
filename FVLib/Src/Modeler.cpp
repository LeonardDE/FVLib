#include "global.h"

#include "Modeler.h"
#include "DataParser.h"
#include "OutputData.h"
#include "FV.h"

Modeler::Modeler(const string& in_file, const string& out_file) {
  this->in_file = in_file;
  this->out_file = out_file;

  // Собираем данные о ЛА
  ParseFVToList(this->in_file, globalSituation);
  
  // Собираем данные для вычислений
  ParseSolveDataFromJSON(this->in_file, timeStep, integral_h, solve_part_count);

  // Ищем максимальное и минимальное значение времени
  for (auto& fv : globalSituation.FVs) {
    for (auto& point : fv->getDynamicPath().getPath()) {
      start_time = start_time > point.arrivalTime ? point.arrivalTime : start_time;
      end_time = end_time < point.arrivalTime ? point.arrivalTime : end_time;
    }
  }
}

void Modeler::startModeling() {
  double timer = start_time;
  double h = timeStep / solve_part_count;

  OutputJsonData data;
  for (auto& fv : globalSituation.FVs) {
    OutputFV out_fv;
    Plane plane = fv->getPlane();
    out_fv.parameters.type = plane.type;
    out_fv.parameters.name = plane.name;
    out_fv.parameters.plan = fv->getBasePath();
    data.FVs.push_back(out_fv);
  }

  while (timer <= end_time) {

    for (auto& fv : globalSituation.FVs) {
      fv->next(h, timer);
    }

    for (int i = 0; i < data.FVs.size(); i++) {
      FV* fv = globalSituation.FVs[i];
      OutputFV& out_fv = data.FVs[i];
      Plane plane = fv->getPlane();

      if (check::LE(timer, fv->getBasePath()[fv->getBasePath().size() - 1].arrivalTime) &&
        check::LE(fv->getBasePath()[0].arrivalTime, timer)) {
        DynamicData dynamic_data;
        dynamic_data.t = timer;
        dynamic_data.x = plane.x;
        dynamic_data.y = plane.y;
        dynamic_data.z = plane.z;
        out_fv.dynamic_data.push_back(dynamic_data);
      }


      if (check::LE(globalSituation.aetherInfo.states[plane.name].planeTranslationTime, timer + timeStep) &&
        check::LT(timer, globalSituation.aetherInfo.states[plane.name].planeTranslationTime)) {
        BroadcastData broadcastdata;
        broadcastdata.t = globalSituation.aetherInfo.states[plane.name].planeTranslationTime;
        broadcastdata.type = "position";
        broadcastdata.x = globalSituation.aetherInfo.states[plane.name].plane.x;

        broadcastdata.y = globalSituation.aetherInfo.states[plane.name].plane.y;

        broadcastdata.z = globalSituation.aetherInfo.states[plane.name].plane.z;
        out_fv.broadcasts.push_back(broadcastdata);
      }

      if (check::LE(globalSituation.aetherInfo.states[plane.name].translationTime, timer + timeStep) &&
        check::LT(timer, globalSituation.aetherInfo.states[plane.name].translationTime)) {
        BroadcastData broadcastdata;
        broadcastdata.t = globalSituation.aetherInfo.states[plane.name].translationTime;
        broadcastdata.type = "plan";
        broadcastdata.plan = globalSituation.aetherInfo.states[plane.name].shortPlan;
        out_fv.broadcasts.push_back(broadcastdata);
      }

    }
    timer += timeStep;
  }
  data.dt = timeStep;
  writeJsonData(out_file, data);
}
