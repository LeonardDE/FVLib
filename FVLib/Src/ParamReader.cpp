//
//  Created by Spiridonov Arseniy on 23.03.2020.
//  Copyright © 2020 Spiridonov Arseniy. All rights reserved.
//

#include <exception>
#include <algorithm>
#include <utility>
#include "ParamReader.h"
//#include "Tools.h"

void ParamReader::readInputStream(ifstream &in) {
    string tempString, testString;
    string badChars = "\r\n\t ";

    if (in.is_open()) {
        while (!in.eof()) {
            //  If the file is open and end of the file isn't reached so read a string
            getline(in, tempString);

            while (!tempString.empty() &&
                   find(badChars.begin(), badChars.end(),*(tempString.end()-1)) != badChars.end()) {
                tempString.erase(tempString.end() - 1);
            }
            //  All non-empty no-comment strings are pushed into the storage
            if (tempString[0] != '#') {
                file.push_back(tempString);
            }
            //  If the current string is variable name, i.e. it begins from the symbol '@'
            if (tempString[0] == '@' && (isalpha(tempString[1]) || tempString[1] == '_')) {
                testString.resize(tempString.size() - 1);
                copy(tempString.begin() + 1, tempString.end(), testString.begin());
                vars[testString] = static_cast<int>(file.size()-1);
            }
        }
        in.close();

    } else {
        throw logic_error("Unable to open input file!");
    }
}

ParamReader::ParamReader(ifstream &fin) {
    readInputStream(fin);
    fin.close();
}

ParamReader::ParamReader(const string &settingsFileName) {
    ifstream fin(settingsFileName);
    readInputStream(fin);
    fin.close();
}

/// todo: может быть в методы чтения передавать ещё граничные константы, в пределах которых хотелось бы прочитать
/// значение. Если прочитанное значение выходит за границы, то бросать исключения.

//  Чтение bool из строки файла
bool ParamReader::readBool(const string &varName) const {
    bool res;
    readGenParam<bool>("readBool", varName, myStob, res);
    return res;
}

//  Чтение int из стройки файла
int ParamReader::readInt(const string &varName) const {
    int res;
    readGenParam<int>("readInt", varName, myStoi, res);
    return res;
}

//  Чтение unsigned из строки файла
unsigned ParamReader::readUInt (const string &varName) const {
    unsigned res;
    readGenParam<unsigned>("readUInt", varName, myStou, res);
    return res;
}

//  Чтение double из стройки файла
double ParamReader::readDouble(const string &varName) const {
    double res;
    readGenParam<double>("readDouble", varName, myStod, res);
    return res;
}

//  Чтение string из стройки файла
string ParamReader::readString(const string &varName) const {
    string res;
    readGenParam<string>("readString", varName, myStos, res);
    return res;
}

//  Чтение vector<double> из файла
void ParamReader::readDoubleArray (const string &varName, const unsigned size, vector<double> &res) const {
    readGenArray<double>("readDoubleArray", varName, size, myStod, res);
}

//  Чтение вектора int значений из строки файла
void ParamReader::readIntArray(const string &varName, const unsigned size, vector<int> &res) const {
    readGenArray<int>("readIntArray", varName, size, myStoi, res);
}

//  Чтение вектора unsigned значений из строки файла
void ParamReader::readUIntArray(const string &varName, const unsigned size, vector<unsigned> &res) const {
    readGenArray<unsigned>("readUIntArray", varName, size, myStou, res);
}

//  Чтение вектора string значений из строки файла
void ParamReader::readStringArray(const string &varName, const unsigned size, vector<string> &res) const {
    readGenArray<string>("readStringArray", varName, size, myStos, res);
}

void ParamReader::readDoubleArray_2d(const string &varName, const unsigned rowSize, const unsigned colSize,
                                     vector<vector<double>> &res) const {
    auto strToVecDouble = [colSize](const string &str) {
        return stringToVectorDouble(str, colSize);
    };
    readGenArray<vector<double>>("readDoubleArray_2d", varName, rowSize, strToVecDouble, res);
}

//  Проверка наличия переменной в прочитанном файле
bool ParamReader::checkVarPresence(const string &varName) const {
    return vars.find(varName) != vars.end();
}

//  Номер строки, содержащей переменную
int ParamReader::getLineNumberForVariable(const string &varName) const {
    checkVarPresence(varName);
    return vars.at(varName);
}

//  Преобразование из string в bool
bool ParamReader::myStob(const string &s) {
    string tmp = s;
    transform(tmp.begin(), tmp.end(), tmp.begin(), ::tolower);
    if (s == "0" || s == "false") {
        return false;
    } else if (s == "1" || s == "true") {
        return true;
    } else {
        throw invalid_argument("Bad symbols! '1', '0', 'true', 'false' are acceptable");
    }
}

//  Костылик. Шаблонный метод не принимает стандартную функцию stoi
//  Преобразование из string в int
int ParamReader::myStoi(const string &s) {
    try {
        return stoi(s);
    } catch (invalid_argument &e) {
        throw invalid_argument("No conversion for int type!");
    } catch (out_of_range &e) {
        throw out_of_range("Out of range for int type!");
    }
}

//  Преобразование из string в unsigned
unsigned ParamReader::myStou(const string &s) {
    unsigned result;

    try {
        unsigned long lresult = stoul(s, nullptr, 10);
        result = static_cast<unsigned>(lresult);
        if (result != lresult) {
            throw out_of_range("Out of range for unsigned type!");
        }
    } catch (invalid_argument &e) {
        throw invalid_argument("No conversion for unsigned type!");
    }

    return result;
}

//  Костылик. Шаблонный метод не принимает стандартную функцию stod
//  Преобразование из string в double
double ParamReader::myStod(const string &s) {
    try {
        return stod(s);
    } catch (invalid_argument &e) {
        throw invalid_argument("No conversion for double type!");
    } catch (out_of_range &e) {
        throw out_of_range("Out of range for double type!");
    }
}

//  Преобразование из string в string
string ParamReader::myStos(const string &s) {
    //  Костылик. Чтобы чтение строки из файла подходило для шаблонного метода readGenParam
    return s;
}

//  Преобразование из string в double
vector<double> ParamReader::stringToVectorDouble(const string &s, const unsigned colSize) {
    vector<string> tmp = splitStringBySeparator(s, ' ');

    if (tmp.size() != colSize) {
        throw invalid_argument("Wrong number of the columns!");
    }

    vector<double> res;
    res.reserve(colSize);
    for (const string &el : tmp) {
        res.push_back(myStod(el));
    }

    return res;
}

