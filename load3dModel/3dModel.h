
#ifndef MODEL_H_
#define MODEL_H_

#include <iostream>

class Model
{
public:
    Model();
    void load(const std::string &path);

    std::vector<std::vector<double>> sensorData_3d;
};

#endif
