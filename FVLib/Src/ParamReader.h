//
//  Created by Spiridonov Arseniy on 23.03.2020.
//  Copyright © 2020 Spiridonov Arseniy. All rights reserved.
//

#pragma once

#include <fstream>
#include <functional>
#include <map>
#include <string>
#include <vector>

//#include "Tools.h"

using namespace std;

class ParamReader {
public:
    //  Конструктор класса
    ParamReader() = default;
    ParamReader(ifstream &fin);
    ParamReader(const string &settingsFileName);
    //  Чтение bool из строки файла
    bool readBool(const string &varName) const;
    //  Чтение int из стройки файла
    int readInt(const string &varName) const;
    //  Чтение unsigned из строки файла
    unsigned readUInt(const string &varName) const;
    //  Чтение double из стройки файла
    double readDouble(const string &varName) const;
    //  Чтение string из стройки файла
    string readString(const string &varName) const;
    //  Чтение вектора double значений из строки файла
    void readDoubleArray(const string &varName, const unsigned size, vector<double> &res) const;
    //  Чтение вектора double значений из строки файла
    void readIntArray(const string &varName, const unsigned size, vector<int> &res) const;
    //  Чтение вектора double значений из строки файла
    void readUIntArray(const string &varName, const unsigned size, vector<unsigned> &res) const;
    //  Чтение вектора string значений из строки файла
    void readStringArray(const string &varName, const unsigned size, vector<string> &res) const;
    //  Чтение вектора double значений из строки файла
    void readDoubleArray_2d(const string &varName, const unsigned rowSize, const unsigned colSize,
                            vector<vector<double>> &res) const;

    //  Проверка наличия переменной в прочитанном файле
    bool checkVarPresence(const string &varName) const;
    //  Номер строки, содержащей переменную
    int getLineNumberForVariable(const string &varName) const;

    //  Шаблонные методы чтения параметров
    //  Метод чтения скалярного параметра
    template <typename T>
    void readGenParam(const string &methodName, const string &varName, function<T (const string &)> f, T &res) const {
        if (!checkVarPresence(varName)) {
            throw logic_error(methodName + ": Bad name '@" + varName + "'!");
        }

        unsigned ind = vars.at(varName) + 1;
        string exceptionHeader = methodName + ", variable '@" + varName + "', line " + to_string(ind + 1) + ": ";

        auto checkInd = [ind](const pair<string, int> &p) {
            return p.second == ind;
        };
        map<string, int>::const_iterator it = find_if(vars.begin(), vars.end(), checkInd);
        if (it != vars.end() || ind >= file.size()) {
            throw domain_error(exceptionHeader + "No value in the input file!");
        }

        safeRead<T>(exceptionHeader, varName, ind, f, res);
    }

    //  Метод чтения массива скалярных параметров
    template <typename T>
    void readGenArray(const string &methodName, const string &varName, const unsigned int size,
                      function<T (const string &)> f, vector<T> &res) const {
        if (!checkVarPresence(varName)) { throw logic_error(methodName + ": Bad name '@" + varName + "'!"); }

        res.clear();
        unsigned ind = vars.at(varName) + 1;
        const string exceptionHeader = methodName + ", variable '@" + varName + "', line " +
            to_string(ind + 1) + ": ";
        auto checkInd = [ind, size](const pair<string, int> &p) {
            return p.second >= ind && p.second <= ind + size - 1;
        };
        map<string, int>::const_iterator it = find_if(vars.begin(), vars.end(), checkInd);
        if (it != vars.end() || ind + size - 1 >= file.size()) {
            throw domain_error(exceptionHeader + "No necessary number of values (" + to_string(size) +
                                ") in the input file!");
        }

        res.reserve(size);
        for (size_t i = 0; i < size; i++) {
            T tmp;
            safeRead<T>(exceptionHeader, varName, ind, f, tmp);
            res.push_back(tmp);
            ind++;
        }
    };

    //  Метод обработки сообщений ошибок при чтении скалярных параметров
    template <typename T>
    void safeRead(const string &exceptionHeader, const string &varName, const unsigned int lineNum,
                  function<T (const string &)> f, T &res) const {
        try {
            res = f(file[lineNum]);
        } catch (const domain_error &e) {
            throw domain_error(exceptionHeader + e.what());
        } catch (const invalid_argument &e) {
            throw invalid_argument(exceptionHeader + e.what());
        } catch (const out_of_range &e) {
            throw out_of_range(exceptionHeader + e.what());
        } catch (const logic_error &e) {
            throw logic_error(exceptionHeader + e.what());
        }
    }

    //  Преобразование из string в bool
    static bool myStob(const string &s);
    //  Костылик. Шаблонный метод не принимает стандартную функцию stoi
    //  Преобразование из string в int
    static int myStoi(const string &s);
    //  Преобразование из string в unsigned
    static unsigned myStou(const string &s);
    //  Костылик. Шаблонный метод не принимает стандартную функцию stod
    //  Преобразование из string в double
    static double myStod(const string &s);
    //  Костылик. Чтобы чтение строки из файла подходило для шаблонного метола readGenParam
    //  Преобразование из string в string
    static string myStos(const string &s);
    //  Преобразование из string в double
    static vector<double> stringToVectorDouble(const string &s, const unsigned colSize);

private:
    //  Список всех строк файла
    vector<string> file;
    //  Список всех имён переменных с их индексами строк, в которых они записаны
    map<string, int> vars;
    //  Чтение данных из входного файлового потока
    void readInputStream(ifstream &in);
};

vector<string> splitStringBySeparator(string s, char sep)
{
    size_t pos = 0;
    vector<string> result;
    while ((pos = s.find(sep)) != std::string::npos) {
        result.push_back(s.substr(0, pos));
        s.erase(0, pos + 1);
    }
    return result;

}