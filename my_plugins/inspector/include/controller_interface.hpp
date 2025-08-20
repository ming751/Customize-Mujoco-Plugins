#pragma once
#include <map>
#include <vector>
#include <iostream>
#include <unordered_map>

#include <mujoco/mujoco.h>

#include "galbot/singorix_common/data_structure.hpp"
#include "galbot/singorix_control/joint_control.hpp"


namespace galbot
{
namespace singorix
{
namespace simulator
{


class ControllerInterface
{
public:
    ControllerInterface();

    bool init(const mjModel* m);

    void updateSensor(const mjModel* m, const mjData* d);
    void updateCtrl(const mjModel* m, mjData* d);

    friend class EmbOSAInterface;

private:
    // sensor and state
    double sensor_time;
    std::vector<int> joint_sensor_idx_;
    std::unordered_map<long long, JointSensor> joint_sensor_map_;
    std::unordered_map<long long, std::string> joint_sensor_idx_2_name_;
    std::vector<int> imu_sensor_idx_;
    std::unordered_map<long long, ImuSensor> sensor_imu_map_;
    std::unordered_map<long long, std::string> imu_sensor_idx_2_name_;
    std::vector<int> frame_state_idx_;
    std::unordered_map<long long, Frame> state_frame_map_;
    std::unordered_map<long long, std::string> frame_state_idx_2_name_;
    std::vector<int> effort_sensor_idx_;
    std::unordered_map<long long, EffortSensor> effort_sensor_map_;
    std::unordered_map<long long, std::string> effort_sensor_idx_2_name_;

    // actuator
    std::vector<bool> joint_do_ctrls_;
    std::vector<JointCommand> joint_cmd_;
    std::vector<std::shared_ptr<control::JointControl>> joint_control_inst_;
    std::unordered_map<std::string, int> joint_control_inst_name_2_idx_;

    int selected_body_;      // 被选中的body ID
    mjtNum target_pos_[3];  // 目标位置
    mjtNum stiffness_;      // 虚拟弹簧刚度
    mjtNum damping_;        // 虚拟弹簧阻尼
};

} // simulator
} // singorix
} // galbot
