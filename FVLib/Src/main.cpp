#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>

#include "FV.h"

using namespace std;

mutex mtx;
mutex data1;
condition_variable cv;
condition_variable new_data;
bool flag = false;
bool solved = false;
int counter = 0;

void timer(int32_t& timer)
{
    while (true)
    {
        flag = true;
        cv.notify_one();
        timer += 1;
        this_thread::sleep_for(chrono::seconds(1));
    }
}

void solving(FV& point, AirSituationState& states)
{
    vector<double> acceleration = { 1,1,1 };
    map<string, void*> dict;
    dict["acceleration"] = &acceleration;
    while (true)
    {
        {
            unique_lock<mutex> lk(mtx);
            cv.wait(lk);//, [] {return flag;});
        }
        flag = false;

        // —чет 
        cout << "—чет" << endl;
        point.next(1,states.time,dict);
        

        data1.lock();
        // ѕерекладка данных
        cout << "ѕерекладка данных" << endl;
        Plane plane_point = point.getPlane();
        states.planes.clear();
        states.planes.push_back(plane_point);

        data1.unlock();

        solved = true;
        new_data.notify_one();

    }
}


int main()
{
    setlocale(LC_ALL, "ru");

    MaterialPoint point = MaterialPoint("point", 0, 0, 0, 1, 0, 0);

    AirSituationState air_states = AirSituationState();
    air_states.time = 0;
    air_states.planes = vector<Plane>();

    thread time(timer,ref(air_states.time));
    time.detach();

    thread solve(solving,ref(point),ref(air_states));

    solve.detach();

    while (true)
    {
        {
            unique_lock<mutex> lk(mtx);
            // ∆дем пока данные вычисл€тс€ 
            new_data.wait(lk);//, [] {return solved;});
        }
        solved = false;
        data1.lock();
        //ѕечатаем
        for (int i = 0; i < air_states.planes.size();i++)
        {   
            Plane& plane = air_states.planes[i];

            cout << "-------" << i+1 <<"------" << endl;
            cout << "Position(" << plane.x << ", " << plane.y << ", " << plane.z << ")" << endl;
            cout << "Velocity(" << plane.speedX << ", " << plane.speedY << ", " << plane.speedZ << ")" << endl;
            cout << "|Velocity| = " << plane.speed << endl;
            cout << "-------------" << endl;
        }
        data1.unlock();
    }

    return 0;
}



