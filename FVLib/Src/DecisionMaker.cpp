#include "global.h"
#include "FlightPlan.h"
#include "DecisionMaker.h"

void DecisionMaker::mergeShortPlans(const FlightPlan& arr1, const FlightPlan& arr2,
  FlightPlan& res1, FlightPlan& res2) {
  res1.clear();
  res2.clear();

  size_t i = 0;
  size_t j = 0;

  double maxStart = max(arr1.getTStart(), arr2.getTStart());
  double minEnd = min(arr1.getTFinal(), arr2.getTFinal());

  while (i < arr1.size() && j < arr2.size()) {
    if (check::LT(arr1[i].arrivalTime, arr2[j].arrivalTime)) {
      if (check::GE(arr1[i].arrivalTime, maxStart) && check::LE(arr1[i].arrivalTime, minEnd)) {
        PathPoint p;
        p.arrivalTime = arr1[i].arrivalTime;
        p.position = arr2.getStateAt(p.arrivalTime, j-1).position;
        res2.addPoint(p);
        res1.addPoint(arr1[i]);
      }
      i++;
    }
    else if (check::GT(arr1[i].arrivalTime, arr2[j].arrivalTime)) {
      if (arr2[j].arrivalTime >= maxStart && arr2[j].arrivalTime <= minEnd) {
        PathPoint p;
        p.arrivalTime = arr1[j].arrivalTime;
        p.position = arr1.getStateAt(p.arrivalTime, i-1).position;
        res1.addPoint(p);
        res2.addPoint(arr2[j]);
      }
      j++;
    }
    else {
      res1.addPoint(arr1[i]);
      res2.addPoint(arr2[j]);
      i++;
      j++;
    }
  }
}

/*

void FV::checkConflict() {
  vector<FV*>& fvs = globalSituation->FVs;
  map<string, BroadcastBatch>& states = globalSituation->aetherInfo.states;
  for (FV* & fv : fvs) {
    string fv_name = fv->getOutputState().name;
    if (fv_name == this->name) continue;

    FVOutputState plane = getOutputState();
    Vector3 pos = Vector3(plane.x, plane.y, plane.z);

    Vector3 hisPos = Vector3(states[fv_name].plane.x,
      states[fv_name].plane.y,
      states[fv_name].plane.z);
    Vector3 relPosition = hisPos - pos;

    if (relPosition.norm() > radiusFilter) continue;

    Vector3 vel = Vector3(plane.speedX, plane.speedY, plane.speedZ);

    Vector3 hisVel = Vector3(states[fv_name].plane.speedX,
      states[fv_name].plane.speedY,
      states[fv_name].plane.speedZ);

    if (scalarProduct(relPosition, hisVel - vel) < 0) continue;

    // Сюда нужна проверка с норированием сетки

    // Нормируем сетки
    vector<Point> my_plan;
    vector<Point> his_plan;
    if (states[this->name].shortPlan.empty() || states[fv_name].shortPlan.empty()) continue;

    mergeShortPlans(states[this->name].shortPlan, states[fv_name].shortPlan, my_plan, his_plan);

    if (my_plan.empty()) continue;

    double left_t_min_xz = INFINITY;
    double right_t_max_xz = -INFINITY;

    double left_t_min_y = INFINITY;
    double right_t_max_y = -INFINITY;

    // Проверка
    for (int i = 0; i < my_plan.size() - 1; i++) {
      // Проверка по XZ
      /// Считаем коэффэциенты  для квадратного уравнения
      Vector3 my_vel = (my_plan[i + 1].position - my_plan[i].position) / (my_plan[i + 1].arrivalTime - my_plan[i].arrivalTime);
      Vector3 his_vel = (his_plan[i + 1].position - his_plan[i].position) / (his_plan[i + 1].arrivalTime - his_plan[i].arrivalTime);

      double A = (my_vel.x - his_vel.x) * (my_vel.x - his_vel.x) + (my_vel.z - his_vel.z) * (my_vel.z - his_vel.z);

      double B = -2 * (my_plan[i].arrivalTime * A + ((my_plan[i].position.x - his_plan[i].position.x) + (my_vel.x - hisVel.x))
        * ((my_plan[i].position.z - his_plan[i].position.z) + (my_vel.z - hisVel.z)));

      double C = -2 * my_plan[i].arrivalTime * ((my_plan[i].position.x - his_plan[i].position.x) + (my_vel.x - hisVel.x))
        * ((my_plan[i].position.z - his_plan[i].position.z) + (my_vel.z - hisVel.z))
        + my_plan[i].arrivalTime * my_plan[i].arrivalTime * A
        + (my_plan[i].position.x - his_plan[i].position.x) * (my_plan[i].position.z - his_plan[i].position.z)
        - radiusWarn;

      /// Находим корни
      double D = B * B - 4 * A * C;
      if (D >= 0) {
        double t1 = (-B - sqrt(D)) / (2 * A);
        double t2 = (-B + sqrt(D)) / (2 * A);

        left_t_min_xz = min(t1, left_t_min_xz);
        right_t_max_xz = max(t2, right_t_max_xz);
      }
      // Проверка по Y
      double Ay = (my_vel.y - his_vel.y) * (my_vel.y - his_vel.y);

      double By = 2 * (my_vel.y - his_vel.y) * ((my_plan[i].position.y - his_plan[i].position.y) + my_plan[i].arrivalTime * (my_vel.y - his_vel.y));

      double Cy = ((my_plan[i].position.y - his_plan[i].position.y) + my_plan[i].arrivalTime * (my_vel.y - his_vel.y))
        * ((my_plan[i].position.y - his_plan[i].position.y) + my_plan[i].arrivalTime * (my_vel.y - his_vel.y))
        - heightWarn;

      /// Находим корни
      double Dy = By * By - 4 * Ay * Cy;
      if (Dy >= 0) {
        double t1_y = (-By - sqrt(Dy)) / (2 * Ay);
        double t2_y = (-By + sqrt(Dy)) / (2 * Ay);

        left_t_min_y = min(t1_y, left_t_min_y);
        right_t_max_y = max(t2_y, right_t_max_y);
      }

    }
    if ((left_t_min_xz - right_t_max_y) * (left_t_min_y - right_t_max_xz) <= 0) return;



    json conflict;
    conflict["name"] = fv->name;
    conflict["tBeg"] = max(left_t_min_xz, left_t_min_y);
    conflict["tEnd"] = min(right_t_max_xz, right_t_max_y);

    conflicts.push_back(conflict);

    // ну и вот тут надо что то делать так как мы понмиаем что у нас тут конфликт
  }
}

*/