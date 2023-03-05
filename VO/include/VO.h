#include"pybind11/pybind11.h"
#include"pybind11/stl.h"
#include "xtensor/xarray.hpp"
#define FORCE_IMPORT_ARRAY
#include <xtensor-python/pyarray.hpp>
#include<cmath>
#include<vector>
#include<string>

namespace py = pybind11;
using namespace py::literals;

struct VOInfo{
    std::vector<double> observation_vo;
    bool vo_flag;
    double exp_time;
    double exp_time2;
    bool collision_flag;
    double min_dis;
    int id;

    VOInfo(){};
};


class VO{
private:
    bool env_train_ = true;
    double exp_radius_ = 0.5;
    double ctime_threshold_ = 5;
    std::string mode_ = "rvo"; 

public:
    VO() {};
    VO(bool env_train, double exp_radius, double ctime_threshold);
    
    ~VO(){};
    
    VOInfo config_vo_circle2(std::vector<double>& s_state, std::vector<double>& o_state, std::vector<double>& action, std::string mode);
    
    VOInfo config_vo_car(std::vector<double>& s_state, std::vector<double>& o_state, std::vector<double>& action, std::string mode);

    VOInfo config_vo(std::vector<double>& s_state, std::vector<double>& o_state, std::vector<double>& action, std::string mode);


    bool vo_out_jud_vector(double vx, double vy, std::vector<double>& vo);

    static double distance_between_cars(xt::pyarray<double> state1, xt::pyarray<double> state2);

    static bool between_vector(std::vector<double>& line_left_vector, std::vector<double>& line_right_vector, std::vector<double> line_vector);

    static double cal_exp_tim(double rel_x, double rel_y, double rel_vx, double rel_vy, double r);
    
    static double clamp(double n, double minn, double maxn);

    static double cross_product(std::vector<double>& vector1, std::vector<double>& vector2);

    static double distance(std::vector<double>& point1, std::vector<double>& point2);
    
    static double wraptopi(double theta);

};

PYBIND11_MODULE(VO_util, m){
    m.doc() = "pybind11 VO plugin";
    xt::import_numpy();
    py::class_<VOInfo>(m, "VOInfo")
    .def(py::init())
    .def_readwrite("observation_vo", &VOInfo::observation_vo)
    .def_readwrite("vo_flag", &VOInfo::vo_flag)
    .def_readwrite("exp_time", &VOInfo::exp_time)
    .def_readwrite("collision_flag", &VOInfo::collision_flag)
    .def_readwrite("min_dis", &VOInfo::min_dis)
    .def_readwrite("id", &VOInfo::id)
    .def_readwrite("exp_time2", &VOInfo::exp_time2);
    
    py::class_<VO>(m, "VO")
    .def(py::init())
    .def(py::init<bool, double, double>())
    .def("config_vo_circle2", &VO::config_vo_circle2, py::return_value_policy::move)
    .def("config_vo_car", &VO::config_vo_car, py::return_value_policy::move)
    .def("config_vo", &VO::config_vo, py::return_value_policy::move)
    .def("vo_out_jud_vector", &VO::vo_out_jud_vector)
    .def_static("between_vector", &VO::between_vector)
    .def_static("cal_exp_tim", &VO::cal_exp_tim)
    .def_static("clamp", &VO::clamp)
    .def_static("cross_product", &VO::cross_product)
    .def_static("distance", &VO::distance)
    .def_static("distance_between_cars", &VO::distance_between_cars)
    .def_static("wraptopi", &VO::wraptopi);
}
