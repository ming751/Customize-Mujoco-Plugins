#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <thread>

#include "galbot/singorix_common/utils/time.hpp"
#include "galbot/singorix_simulator/interface/controller_interface.hpp"

#include <glog/logging.h>

using namespace galbot::singorix;
using namespace galbot::singorix::simulator;

ControllerInterface::ControllerInterface() {};

bool ControllerInterface::init(const mjModel* m)
{
    joint_sensor_idx_.clear();
    joint_sensor_map_.clear();
    joint_sensor_idx_2_name_.clear();
    imu_sensor_idx_.clear();
    sensor_imu_map_.clear();
    imu_sensor_idx_2_name_.clear();
    frame_state_idx_.clear();
    state_frame_map_.clear();
    frame_state_idx_2_name_.clear();
    effort_sensor_idx_.clear();
    effort_sensor_map_.clear();
    effort_sensor_idx_2_name_.clear();
    // 添加传感器数据
    for (int i = 0; i < m->nsensor; ++i) {
        // 获取传感器名称
        // const char* sensor_name = mj_id2name(m, mjOBJ_SENSOR, i);
        // if (sensor_name == nullptr) {
        //     sensor_name = "Unnamed";
        // }

        // 获取传感器类型
        int sensor_type = m->sensor_type[i];

        // 检查传感器关联的obj
        int obj_id = m->sensor_objid[i];  // 获取关联的obj ID
        int obj_type = m->sensor_objtype[i];  // 获取关联的obj type
        long long map_id = obj_type << 32 + obj_id;
        const char* obj_name = mj_id2name(m, m->sensor_objtype[i], obj_id);  // 获取obj名称

        // 检查传感器是否关是关节数据
        if (sensor_type == mjSENS_JOINTPOS || sensor_type == mjSENS_JOINTVEL || sensor_type == mjSENS_JOINTACTFRC) {
            joint_sensor_map_.emplace(map_id, JointSensor());
            joint_sensor_idx_2_name_.emplace(map_id, obj_name);
        } else if (sensor_type == mjSENS_ACCELEROMETER || sensor_type == mjSENS_GYRO || sensor_type == mjSENS_MAGNETOMETER) {
            sensor_imu_map_.emplace(map_id, ImuSensor());
            imu_sensor_idx_2_name_.emplace(map_id, obj_name);
        } else if (sensor_type == mjSENS_FRAMEPOS || sensor_type == mjSENS_FRAMEQUAT 
                || sensor_type == mjSENS_FRAMELINVEL || sensor_type == mjSENS_FRAMEANGVEL) {
            state_frame_map_.emplace(map_id, Frame());
            frame_state_idx_2_name_.emplace(map_id, obj_name);
            std::cout << "ssssss " << map_id << " obj_name " << obj_name << std::endl;
        } else if (sensor_type == mjSENS_FORCE || sensor_type == mjSENS_TORQUE) {
            effort_sensor_map_.emplace(map_id, EffortSensor());
            effort_sensor_idx_2_name_.emplace(map_id, obj_name);
        }
    }
    for (auto it: joint_sensor_idx_2_name_) {
        joint_sensor_idx_.push_back(it.first);
    }
    for (auto it: imu_sensor_idx_2_name_) {
        imu_sensor_idx_.push_back(it.first);
    }
    for (auto it: frame_state_idx_2_name_) {
        frame_state_idx_.push_back(it.first);
    }
    for (auto it: effort_sensor_idx_2_name_) {
        effort_sensor_idx_.push_back(it.first);
    }

    joint_do_ctrls_.resize(m->nu);
    joint_cmd_.resize(m->nu);
    joint_control_inst_.resize(m->nu);
    for (int i = 0; i < m->nu; i++) {
        const char* actuator_name = mj_id2name(m, mjOBJ_ACTUATOR, i);
        joint_control_inst_name_2_idx_[actuator_name] = i;
        joint_do_ctrls_[i] = false;
        joint_cmd_[i].position = 0;
        joint_cmd_[i].velocity = 0;
        joint_cmd_[i].acceleration = 0;
        joint_cmd_[i].effort = 0;
        std::cout << "actuator_name: " << actuator_name << std::endl;
        joint_control_inst_[i] = std::make_shared<control::JointControl>(control::JCtrlMode::JCtrlPosVelEff2Eff);
    }
    return true;
}


void ControllerInterface::updateSensor(const mjModel* m, const mjData* d) 
{
    sensor_time = d->time;
    // std::cout << "updateSensor, time: " << sensor_time << std::endl;
    int data_index = 0;
    for (int i = 0; i < m->nsensor; ++i) {
        int sensor_type = m->sensor_type[i];
        int sensor_dim = m->sensor_dim[i];
        int obj_id = m->sensor_objid[i];  // 获取关联的obj ID
        int obj_type = m->sensor_objtype[i];  // 获取关联的obj type
        long long map_id = obj_type << 32 + obj_id;

        switch (sensor_type) {
            case mjSENS_JOINTPOS:
                joint_sensor_map_[map_id].position = d->sensordata[data_index];
                break;
            case mjSENS_JOINTVEL:
                joint_sensor_map_[map_id].velocity = d->sensordata[data_index];
                break;
            case mjSENS_JOINTACTFRC:
                joint_sensor_map_[map_id].effort = d->sensordata[data_index];
                break;
            case mjSENS_ACCELEROMETER:
                sensor_imu_map_[map_id].accel(0) = d->sensordata[data_index];
                sensor_imu_map_[map_id].accel(1) = d->sensordata[data_index+1];
                sensor_imu_map_[map_id].accel(2) = d->sensordata[data_index+2];
                break;
            case mjSENS_GYRO:
                sensor_imu_map_[map_id].gyro(0) = d->sensordata[data_index];
                sensor_imu_map_[map_id].gyro(1) = d->sensordata[data_index+1];
                sensor_imu_map_[map_id].gyro(2) = d->sensordata[data_index+2];
                break;
            case mjSENS_MAGNETOMETER:
                sensor_imu_map_[map_id].magnet(0) = d->sensordata[data_index];
                sensor_imu_map_[map_id].magnet(1) = d->sensordata[data_index+1];
                sensor_imu_map_[map_id].magnet(2) = d->sensordata[data_index+2];
            case mjSENS_FRAMEPOS:
                state_frame_map_[map_id].pose.position(0) = d->sensordata[data_index];
                state_frame_map_[map_id].pose.position(1) = d->sensordata[data_index+1];
                state_frame_map_[map_id].pose.position(2) = d->sensordata[data_index+2];
                break;
            case mjSENS_FRAMEQUAT:
                state_frame_map_[map_id].pose.orientation.w() = d->sensordata[data_index];
                state_frame_map_[map_id].pose.orientation.x() = d->sensordata[data_index+1];
                state_frame_map_[map_id].pose.orientation.y() = d->sensordata[data_index+2];
                state_frame_map_[map_id].pose.orientation.z() = d->sensordata[data_index+3];
                break;
            case mjSENS_FRAMELINVEL:
                state_frame_map_[map_id].twist.linear(0) = d->sensordata[data_index];
                state_frame_map_[map_id].twist.linear(1) = d->sensordata[data_index+1];
                state_frame_map_[map_id].twist.linear(2) = d->sensordata[data_index+2];
                break;
            case mjSENS_FRAMEANGVEL:
                state_frame_map_[map_id].twist.angular(0) = d->sensordata[data_index];
                state_frame_map_[map_id].twist.angular(1) = d->sensordata[data_index+1];
                state_frame_map_[map_id].twist.angular(2) = d->sensordata[data_index+2];
                break;
            case mjSENS_FORCE:
                effort_sensor_map_[map_id].force(0) = d->sensordata[data_index];
                effort_sensor_map_[map_id].force(1) = d->sensordata[data_index+1];
                effort_sensor_map_[map_id].force(2) = d->sensordata[data_index+2];
                break;
            case mjSENS_TORQUE:
                effort_sensor_map_[map_id].torque(0) = d->sensordata[data_index];
                effort_sensor_map_[map_id].torque(1) = d->sensordata[data_index+1];
                effort_sensor_map_[map_id].torque(2) = d->sensordata[data_index+2];
                break;
            default:
                break;
        }
        data_index += sensor_dim;
    }

    // for (auto it: joint_sensor_map_) {
    //     printf("%d %s %f %f %f\n", it.first, joint_sensor_idx_2_name_[it.first].c_str(), it.second.position, it.second.velocity, it.second.effort);
    // }

    // for (auto it: sensor_imu_map_) {
    //     printf("%d %s accel[%8.4f %8.4f %8.4f] gyro[%8.4f %8.4f %8.4f] magnet[%8.4f %8.4f %8.4f]\n", it.first, imu_sensor_idx_2_name_[it.first].c_str(), 
    //             it.second.accel[0], it.second.accel[1], it.second.accel[2], 
    //             it.second.gyro[0], it.second.gyro[1], it.second.gyro[2], 
    //             it.second.magnet[0], it.second.magnet[1], it.second.magnet[2]);
    // }

    // for (auto it: state_frame_map_) {
    //     printf("%d %s pose[%8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f] twist[%8.4f %8.4f %8.4f %8.4f %8.4f %8.4f]\n", it.first, frame_state_idx_2_name_[it.first].c_str(), 
    //         it.second.pose.position(0), 
    //         it.second.pose.position(1), 
    //         it.second.pose.position(2), 
    //         it.second.pose.orientation.w(), 
    //         it.second.pose.orientation.x(), 
    //         it.second.pose.orientation.y(), 
    //         it.second.pose.orientation.z(),
    //         it.second.twist.linear(0),
    //         it.second.twist.linear(1),
    //         it.second.twist.linear(2),
    //         it.second.twist.angular(0),
    //         it.second.twist.angular(1),
    //         it.second.twist.angular(2));
    // }
    // const mjtNum gravity[3] = {0, 0, -9.81};
    // for(auto it: effort_sensor_map_){
    //     std::string name = effort_sensor_idx_2_name_[it.first];
    //     int body_id = mj_name2id(m, mjOBJ_BODY, "right_suction_cup_link1");
    //     mjtNum mass = body_id >= 0 ? m->body_mass[body_id] : 0;
    //     // 计算重力分量（世界坐标系到传感器坐标系）
    //     mjtNum gravity_force[3], gravity_torque[3];
    //     mjtNum sensor_rot[9];
    //     // 使用转置矩阵进行坐标变换（世界坐标系 -> 传感器坐标系）
    //     mju_transpose(sensor_rot, d->site_xmat + 9*it.first, 3, 3);
    //     mju_mulMatVec(gravity_force, sensor_rot, gravity, 3, 3);
    //     mju_scl(gravity_force, gravity_force, mass, 3);
    //     // 计算重力扭矩（需要质心偏移量，这里假设质心在传感器原点）
    //     // 如果有质心偏移，需使用 mj_subtree com 计算
    //     mju_zero(gravity_torque, 3);
    //     // 补偿重力影响
    //     it.second.force(0) -= gravity_force[0];
    //     it.second.force(1) -= gravity_force[1];
    //     it.second.force(2) -= gravity_force[2];
    //     printf("%d %s force[%8.4f %8.4f %8.4f] torque[%8.4f %8.4f %8.4f]\n", it.first, effort_sensor_idx_2_name_[it.first].c_str(),
    //         it.second.force(0),
    //         it.second.force(1),
    //         it.second.force(2),
    //         it.second.torque(0),
    //         it.second.torque(1),
    //         it.second.torque(2));
    // }
}


void ControllerInterface::updateCtrl(const mjModel* m, mjData* d) 
{
    // std::cout << "updateCtrl: " << getTimestampMilliseconds() << std::endl;
    for (size_t i = 0; i < m->nu; i++) {
        if (joint_do_ctrls_[i]) {
            int trn_id = m->actuator_trnid[2 * i];
            int qpos_addr = m->jnt_qposadr[trn_id];
            int qvel_addr = m->jnt_dofadr[trn_id];
            mjtNum joint_position = d->qpos[qpos_addr];
            mjtNum joint_velocity = d->qvel[qvel_addr];
            joint_control_inst_[i]->updateState(m->opt.timestep, joint_position, joint_velocity);
            d->ctrl[i] = joint_control_inst_[i]->compute(m->opt.timestep, joint_cmd_[i].position, joint_cmd_[i].velocity, joint_cmd_[i].effort);
        }
    }
}
