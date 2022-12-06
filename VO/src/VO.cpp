#include<VO.h>

#define INF 1e10

VO::VO(bool env_train, double exp_radius, double ctime_threshold):env_train_(env_train), exp_radius_(exp_radius), ctime_threshold_(ctime_threshold){};

VOInfo VO::config_vo_circle2(std::vector<double>& s_state, std::vector<double>& o_state, std::vector<double>& action, std::string mode){
    if (fabs(o_state[2]) <= 1e-6 && fabs(o_state[3]) <= 1e-6){
        mode = "vo";
    }
    double x, y, vx, vy, r, mx, my, mvx, mvy, mr;
    x = s_state[0]; y = s_state[1]; vx = s_state[2]; vy = s_state[3]; r = s_state[4];
    mx = o_state[0]; my = o_state[1]; mvx = o_state[2]; mvy = o_state[3]; mr = o_state[4];
    

    bool vo_flag = false;
    bool collision_flag = false;

    double rel_x = x - mx;
    double rel_y = y - my;

    double dis_mr = sqrt(pow(rel_y, 2) + pow(rel_x, 2));
    double angle_mr = atan2(my - y, mx - x);

    double real_dis_mr = dis_mr;//sqrt(pow(rel_y, 2), pow(rel_x, 2));

    if (env_train_){
        if (dis_mr <= r + mr){
            dis_mr = r + mr;
            collision_flag = true;
        }
    }
    else{
        if (dis_mr <= r - exp_radius_ + mr){
            collision_flag = true;
        }

        if (dis_mr <= r + mr){
            dis_mr = r + mr;
        }
    }
    double ratio = (r + mr) / dis_mr;
    double half_angle = asin(ratio);

    double line_left_ori = VO::wraptopi(angle_mr + half_angle);
    double line_right_ori = VO::wraptopi(angle_mr - half_angle);
    
    double rel_vx, rel_vy;
    std::vector<double> vo;
    
    if (mode == "vo"){
        vo = {mvx, mvy, line_left_ori, line_right_ori};
        rel_vx = action[0] - mvx;
        rel_vy = action[1] - mvy;        
    }
    else if (mode == "rvo"){
        vo = {(vx + mvx)/2, (vy + mvy)/2, line_left_ori, line_right_ori};
        rel_vx = 2 * action[0] - mvx - vx;
        rel_vy = 2 * action[1] - mvy - vy;
    }
    double exp_time = INF;
    if (vo_out_jud_vector(action[0], action[1], vo)){
        vo_flag = false;
        exp_time = INF;
    }
    else{
        exp_time = cal_exp_tim(rel_x, rel_y, rel_vx, rel_vy, r+mr);
        if (exp_time < ctime_threshold_){
            vo_flag = true;
        }
        else{
            vo_flag = false;
            exp_time = INF;
        }
    }

    double input_exp_time = 1 / (exp_time + 0.2);
    double min_dis = real_dis_mr - mr;

    VOInfo vo_info;
    vo_info.observation_vo = {vo[0], vo[1], cos(vo[2]), sin(vo[2]), cos(vo[3]), sin(vo[3]), min_dis, input_exp_time};
    vo_info.vo_flag = vo_flag;
    vo_info.exp_time = exp_time;
    vo_info.collision_flag = collision_flag;
    vo_info.min_dis = min_dis;
    return vo_info;
}

VOInfo VO::config_vo_car(std::vector<double>& s_state, std::vector<double>& o_state, std::vector<double>& action, std::string mode){
    std::vector<double> rear_state = {s_state[0], s_state[1], s_state[2], s_state[3], s_state[9]};
    std::vector<double> fore_state = {s_state[4], s_state[5], s_state[6], s_state[7], s_state[9]};
    std::vector<double> rear_state2 = {o_state[0], o_state[1], o_state[2], o_state[3], o_state[9]};
    std::vector<double> fore_state2 = {o_state[4], o_state[5], o_state[6], o_state[7], o_state[9]};

    double vel = action[0] * 0.1 + s_state[13];
    double steer = action[1] * 0.1 + s_state[8];
    double v_dir = VO::wraptopi(steer + s_state[12]);
    std::vector<double> fore_action = {vel * cos(v_dir), vel * sin(v_dir)};
    std::vector<double> rear_action = {vel * cos(s_state[12]), vel * sin(s_state[12])};   

    VOInfo vo1 = config_vo_circle2(rear_state, rear_state2, rear_action, mode);
    VOInfo vo2 = config_vo_circle2(rear_state, fore_state2, rear_action, mode);
    VOInfo vo3 = config_vo_circle2(fore_state, rear_state2, fore_action, mode);
    VOInfo vo4 = config_vo_circle2(fore_state, fore_state2, fore_action, mode);

    VOInfo vo_info;
    vo_info.observation_vo.insert(vo_info.observation_vo.end(), vo1.observation_vo.begin(), vo1.observation_vo.end());
    vo_info.observation_vo.insert(vo_info.observation_vo.end(), vo2.observation_vo.begin(), vo2.observation_vo.end());
    vo_info.observation_vo.insert(vo_info.observation_vo.end(), vo3.observation_vo.begin(), vo3.observation_vo.end());
    vo_info.observation_vo.insert(vo_info.observation_vo.end(), vo4.observation_vo.begin(), vo4.observation_vo.end());
    
    vo_info.vo_flag = vo1.vo_flag | vo2.vo_flag | vo3.vo_flag | vo4.vo_flag;
    vo_info.exp_time = std::min(vo1.exp_time, std::min(vo2.exp_time, std::min(vo3.exp_time, vo4.exp_time)));
    vo_info.collision_flag = vo1.collision_flag | vo2.collision_flag | vo3.collision_flag | vo4.collision_flag;
    vo_info.min_dis = std::min(vo1.min_dis, std::min(vo2.min_dis, std::min(vo3.min_dis, vo4.min_dis)));
    
    // Current center action 
    // double steer = action[1] * 0.1 + s_state[8];
    // double radius = 2.7 / tan(fabs(steer));
    // double ma_steer_dir = atan2(1.35, radius);
    // if (action[1] < 0){
    //     ma_steer_dir = -ma_steer_dir;
    // }
    // double va_dir = VO::wraptopi(ma_steer_dir + s_state[12]);
    // std::vector<double> m_action = {vel * cos(va_dir), vel * sin(va_dir)};
    
    // // self center vel
    // double radius_1 = 2.7 / tan(fabs(s_state[14]));
    // double steer_dir = atan2(1.35, radius_1);
    // if (action[1] < 0){
    //     steer_dir = -steer_dir;
    // }
    // double v_dir = VO::wraptopi(steer_dir + s_state[12]);
    // std::vector<double> action1 = {s_state[13] * cos(v_dir), s_state[13] * sin(v_dir)};
    // std::vector<double> m_state = {(s_state[0] + s_state[4])/ 2, (s_state[1] + s_state[5])/2, action1[0], action1[1], s_state[9]};

    // // other agent center vel
    // double radius_2 = 2.7 / tan(fabs(o_state[12]));
    // double steer_dir2 = atan2(1.35, radius_2);
    // if (o_state[12] < 0){
    //     steer_dir2 = -steer_dir2;
    // }
    // double v_dir2 = VO::wraptopi(steer_dir2 + o_state[10]);
    // std::vector<double> action2 = {o_state[11] * cos(v_dir2), o_state[11] * sin(v_dir2)};
    // std::vector<double> m_state2 = {(o_state[0] + o_state[4])/ 2, (o_state[1] + o_state[5])/2, action2[0], action2[1], o_state[9]};

    // VOInfo vo_info = config_vo_circle2(m_state, m_state2, m_action, mode);

    return vo_info;
}

bool VO::vo_out_jud_vector(double vx, double vy, std::vector<double>& vo){
    std::vector<double> rel_vector = {vx - vo[0], vy - vo[1]};
    std::vector<double> line_left_vector = {cos(vo[2]), sin(vo[2])};
    std::vector<double> line_right_vector = {cos(vo[3]), sin(vo[3])};

    if (VO::between_vector(line_left_vector, line_right_vector, rel_vector)){
        return false;
    }
    else{
        return true;
    }
    
}

bool VO::between_vector(std::vector<double>& line_left_vector, std::vector<double>& line_right_vector, std::vector<double> line_vector){
    if (VO::cross_product(line_left_vector, line_vector) <= 0 && VO::cross_product(line_right_vector, line_vector) >= 0){
        return true;
    }else{
        return false;
    }
}

double VO::cal_exp_tim(double rel_x, double rel_y, double rel_vx, double rel_vy, double r){
    double a = (pow(rel_vx, 2), pow(rel_vy, 2));
    double b = 2 * rel_x * rel_vx + 2 * rel_y * rel_vy;
    double c = pow(rel_x, 2) + pow(rel_y, 2) - pow(r, 2);

    if (c <= 0){
        return 0.0;
    }

    double t;
    double temp = pow(b, 2) - 4 * a * c;
    if (temp <= 0){
        t = INF;
    }else{
        double t1 = (-b + sqrt(temp)) / (2 * a);
        double t2 = (-b - sqrt(temp)) / (2 * a);

        double t3 = t1 >= 0 ? t1 : INF;
        double t4 = t2 >= 0 ? t2 : INF;
        
        t = std::min(t3, t4);
    }

    return t;
}

double VO::cross_product(std::vector<double>& vector1, std::vector<double>& vector2){
    return (vector1[0] * vector2[1] - vector2[0] * vector1[1]);
}

double VO::distance(std::vector<double>& point1, std::vector<double>& point2){
    return sqrt(pow(point2[0] - point1[0], 2) + pow(point2[1] - point1[1], 2));
}

double VO::distance_between_cars(xt::pyarray<double> state1, xt::pyarray<double> state2){
    std::vector<double> rear_point1 = {state1(0), state1(1)};
    std::vector<double> rear_point2 = {state2(0), state2(1)};

    std::vector<double> fore_point1 = {state1(4), state1(5)};
    std::vector<double> fore_point2 = {state2(4), state2(5)};
    
    double dist1 = VO::distance(rear_point1, fore_point2);
    double dist2 = VO::distance(rear_point1, rear_point2);
    double dist3 = VO::distance(fore_point1, fore_point2);
    double dist4 = VO::distance(fore_point1, rear_point2);
    
    return std::min(dist1, std::min(dist2, std::min(dist3, dist4)));
}

double VO::wraptopi(double theta){
    if (theta > M_PI){
        theta = theta - 2 * M_PI;
    }
    if (theta < - M_PI){
        theta = theta + 2 * M_PI;
    }
    return theta;
}

double VO::clamp(double n, double minn, double maxn){
    return std::max(std::min(maxn, n), minn);
}
