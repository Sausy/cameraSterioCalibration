#ifndef MODEL_H_
#define MODEL_H_

//#define YAML_VALUE_SCALE 1000
#define YAML_VALUE_SCALE 1000

class Model
{
public:
    Model();
    //load 3D sensor positions from a *.yaml file
    void load(const std::string &path);

    std::vector<std::vector<double>> sensorData_3d;
};

#endif
