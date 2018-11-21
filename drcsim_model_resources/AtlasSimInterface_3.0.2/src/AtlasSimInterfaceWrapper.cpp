
/*
 * Copyright 2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <iostream>
#include <cstring>
#include <vector>
#include "AtlasControlTypes.h"
#include "AtlasSimInterfaceWrapper.h"
#include "AtlasSimInterfaceTypes.h"
#include "AtlasVectorTypes.h"
#include "AtlasSimInterface.h"
#include <sstream>
#include <malloc.h>

const std::string  AtlasSimInterfaceWrapper::behaviors[Atlas::NUM_BEHAVIORS] = {"None", "Freeze", "StandPrep", "Stand", "Walk", "Step", "Manipulate", "User"};
AtlasControlInput  AtlasSimInterfaceWrapper::input;
AtlasControlOutput AtlasSimInterfaceWrapper::output;
AtlasRobotState    AtlasSimInterfaceWrapper::robot_state;

std::string printVector(const AtlasVec3f &vec){
    std::stringstream output;
    output<<"{"<<vec.n[0] <<
            ", "<<vec.n[1] <<
            ", "<<vec.n[2]<<"}";
    return output.str();
}

std::string printQuaternion(const AtlasQuaternion &q){
    std::stringstream output;
    output<<" m_qw ="<<q.m_qw <<
            " m_qx ="<<q.m_qx <<
            " m_qy ="<<q.m_qy <<
            " m_qz ="<<q.m_qz;
    return output.str();
}

std::string printAction(const AtlasBehaviorStepAction &a){
    std::stringstream output;
    output<< "knee_nominal="  << a.knee_nominal <<
             " lift_height"   << a.lift_height <<
             " max_body_accel"<< a.max_body_accel <<
             " max_foot_vel"  << a.max_foot_vel<<
             " max_body_accel"<< a.max_body_accel <<
             " step_duration" << a.step_duration <<
             " step_end_dist" << a.step_end_dist <<
             " sway_duration" << a.sway_duration <<
             " sway_end_dist" << a.sway_end_dist <<
             " swing_height" << a.swing_height <<
             " toe_off" <<a.toe_off;
    return output.str();
}

std::string printAction(const AtlasBehaviorWalkAction &a){
    std::stringstream output;
    output<< "step_duration="  << a.step_duration <<
             " swing_height"   << a.swing_height ;
    return output.str();
}


std::string printStepData(const AtlasBehaviorStepData &s){
    std::stringstream output;
    output<< "duration =" << s.duration <<
             " foot_index =" << s.foot_index <<
             " step_index =" << s.step_index <<
             " swing_height =" << s.swing_height <<
             " yaw =" << s.yaw <<
             "\nnormal =" << printVector(s.normal) <<
             "\nposition =" << printVector(s.position);
    return output.str();
}

std::string printSpec(const AtlasBehaviorStepSpec &s){
    std::stringstream output;
    output <<
              "action =" << printAction(s.action) <<
              " foot_index =" << s.foot_index <<
              " step_index =" << s.step_index <<
              "\ndesired step spec - foot:\n" <<
              "normal =" << printVector(s.foot.normal) <<
              "\nposition =" << printVector(s.foot.position)<<
              "\nyaw =" << s.foot.yaw;
    return output.str();
}

std::string printSpec(const AtlasBehaviorWalkSpec &s){
    std::stringstream output;
    output<<"action =" << printAction(s.action) <<
            " foot_index =" << s.foot_index <<
            " step_index =" << s.step_index <<
            "\ndesired step spec - foot:\n" <<
            "normal =" << printVector(s.foot.normal) <<
            "\nposition =" << printVector(s.foot.position)<<
            "\nyaw =" << s.foot.yaw;
    return output.str();
}

std::string printJoints(const AtlasJointState* j){
    std::stringstream output;
    for (int i=0; i < NUM_JOINTS; i++){
        output << "J:" << i <<
                  "  f_d ="<< j[i].f <<
                  "  q_d ="<< j[i].q <<
                  "  qd_d ="<< j[i].qd <<
                  "\n";
    }

    return output.str();
}

std::string printFootSensorArray(const AtlasFootSensor* f){
    std::stringstream output;
    for (int i=0; i < NUM_FOOT_SENSORS; i++){
        output << "Sensor:" << i <<
                  "  fz ="<< f[i].fz <<
                  "  mx ="<< f[i].mx <<
                  "  my ="<< f[i].my <<
                  "\n";
    }

    return output.str();
}

std::string printIMUData(const AtlasIMUData &imu){
    std::stringstream output;
    output << "imu_timestamp"<<imu.imu_timestamp << " Seq_id ="<<imu.seq_id <<
              "\n angular velocity ="<<printVector(imu.angular_velocity) <<
              "\n linear_acceleration ="<<printVector(imu.linear_acceleration) <<
              "\n orientation_estimate ="<<printQuaternion(imu.orientation_estimate)<<
              "\n";

    return output.str();
}

std::string printWristSensorArray(const AtlasWristSensor* f){
    std::stringstream output;
    for (int i=0; i < NUM_WRIST_SENSORS; i++){
        output << "Sensor:" << i <<
                  "  f ="<< printVector(f[i].f) <<
                  "  m ="<< printVector(f[i].m) <<
                  "\n";
    }

    return output.str();
}

void printInput(const AtlasSim::AtlasControlInput &input){
    std::cout<<"Printing output"<<std::endl;
    std::stringstream output;
    output << "\n --- Joint Values ---\n";
    for (int i=0; i < NUM_JOINTS; i++){
        output << "J:" << i <<
                  "  f_d ="<< input.j[i].f_d <<
                  "  q_d ="<< input.j[i].q_d <<
                  "  qd_d ="<< input.j[i].qd_d <<
                  "  k_q_p ="<< input.jparams[i].k_q_p <<
                  "  k_q_i ="<< input.jparams[i].k_q_i <<
                  "  kq_d_p ="<< input.jparams[i].k_qd_p <<
                  "\n";
    }

    output << "\n\n --- Manipulate Params ---\n"<<
              "use_demo =" << input.manipulate_params.use_demo_mode <<
              "\nuse_desired =" << input.manipulate_params.use_desired <<
              "\ndesired :\n " <<
              "com_v0 =" << input.manipulate_params.desired.com_v0 <<
              " com_v1 =" << input.manipulate_params.desired.com_v1 <<
              " pelvis_height =" << input.manipulate_params.desired.pelvis_height <<
              " pelvis_pitch =" << input.manipulate_params.desired.pelvis_pitch <<
              " pelvis_roll =" << input.manipulate_params.desired.pelvis_roll <<
              " pelvis_yaw =" << input.manipulate_params.desired.pelvis_yaw;



    output << "\n\n --- Stand Params ---\n"<<
              "placeholder =" <<input.stand_params.placeholder;

    output << "\n --- Step Params ---\n"<<
              "use_demo_walk =" <<input.step_params.use_demo_walk<<
              " use_spec =" <<input.step_params.use_spec<<
              " use_relative_height =" <<input.step_params.use_relative_step_height <<
              " pelvis_orientation_offset =" << printQuaternion(input.step_params.pelvis_orientation_offset) <<
              "\ndesired step:\n" << printStepData(input.step_params.desired_step) <<
              "\ndesired step spec:\n" << printSpec(input.step_params.desired_step_spec);

    output << "\n --- Walk Params ---\n"<<
              "use_demo_walk =" <<input.walk_params.use_demo_walk<<
              " use_spec =" <<input.walk_params.use_spec<<
              " use_relative_height =" <<input.walk_params.use_relative_step_height;
    output <<"\nStep Queue =";
    for(int i = 0; i < NUM_REQUIRED_WALK_STEPS; i++) {
        output<< printStepData(input.walk_params.step_queue[i]);
    }
    output <<"\nWalk spec queue =";
    for(int i = 0; i < NUM_REQUIRED_WALK_STEPS; i++) {
        output<< printSpec(input.walk_params.walk_spec_queue[i]);
    }
    output <<"\n\n\n";
    std::cout <<output.rdbuf();
}


void printState(const AtlasRobotState &state){
    std::cout<<"Printing State"<<std::endl;
    std::stringstream output;
    output << "Joint Values : \n"<<printJoints(state.j) <<
              "t ="<<state.t <<
              "Foot Sensor :\n" <<printFootSensorArray(state.foot_sensors) <<
              "IMU Data :\n" <<printIMUData(state.imu) <<
              "Wrist Sensor :\n" <<printWristSensorArray(state.wrist_sensors);

    output<<"\n\n";
    std::cout<<output.rdbuf();
}

void printOutput(const AtlasControlOutput& o){
    std::cout<<"Printing Output"<<std::endl;
    std::stringstream output;

    output << " \n * * * ** * * ** * * ** * * ** * * ** * * ** * * ** * * * \n";

    output << "\n --- Joint Torques ---\n";
    for (int i=0; i < NUM_JOINTS; i++){
        output << "f_out:" << i << "  " << o.f_out[i] << "\n";
    }

    output << "\n --- Position Data ---\n";
    output << " Position " << printVector( o.pos_est.position) << "\n";
    output << " Velocity " << printVector( o.pos_est.velocity) << "\n";

    output << "\n --- Foot Position Estimate ---\n";
    for (int i = 0; i < NUM_FEET; ++i) {
        output << " foot_pos_est " << i <<"  " << printVector( o.foot_pos_est[i]) << "\n";
    }

    output << "\n --- Behavior Feedback ---\n";
    output << " status flag " << o.behavior_feedback.status_flags <<" \n";
    output << " trans_from_behavior_index " << o.behavior_feedback.trans_from_behavior_index <<" \n";
    output << " trans_to_behavior_index " << o.behavior_feedback.trans_to_behavior_index <<" \n";

    output << "\n --- Stand Feedback ---\n";
    output << " status flag " << o.stand_feedback.status_flags <<" \n";

    output << "\n --- Step Feedback ---\n";
    output << " step time remaining " << o.step_feedback.t_step_rem <<" \n";
    output << " current Step Index " << o.step_feedback.current_step_index <<" \n";
    output << " next Step Index " << o.step_feedback.next_step_index_needed <<" \n";
    output << " status flag " << o.step_feedback.status_flags <<" \n";

    output << "\n --- Step Feedback Step Data ---\n";
    output << "step data step index " << o.step_feedback.desired_step_saturated.step_index << "\n";
    output << "step data foot index " << o.step_feedback.desired_step_saturated.foot_index << "\n";
    output << "step data duration " << o.step_feedback.desired_step_saturated.duration << "\n";
    output << "step data position " << printVector(o.step_feedback.desired_step_saturated.position) << "\n";
    output << "step data yaw " << o.step_feedback.desired_step_saturated.yaw << "\n";
    output << "step data normal " << printVector(o.step_feedback.desired_step_saturated.normal) << "\n";
    output << "step data swing height " << o.step_feedback.desired_step_saturated.swing_height << "\n";

    output << "\n --- Step Feedback Step Data Spec ---\n";
    output << "step data step index " << o.step_feedback.desired_step_spec_saturated.step_index << "\n";
    output << "step data foot index " << o.step_feedback.desired_step_spec_saturated.foot_index << "\n";

    output << "\n --- Step Feedback Step Data Spec Foot Data ---\n";
    output << "step data foot data position " << printVector(o.step_feedback.desired_step_spec_saturated.foot.position) << "\n";
    output << "step data foot data normal " << printVector(o.step_feedback.desired_step_spec_saturated.foot.normal) << "\n";
    output << "step data foot data yaw " << o.step_feedback.desired_step_spec_saturated.foot.yaw << "\n";

    output << "\n --- Step Feedback Step Data Spec Step Action ---\n";
    output << "step data step action step duration " << o.step_feedback.desired_step_spec_saturated.action.step_duration << "\n";
    output << "step data step action sway duration " << o.step_feedback.desired_step_spec_saturated.action.sway_duration << "\n";
    output << "step data step action swing height " << o.step_feedback.desired_step_spec_saturated.action.swing_height << "\n";
    output << "step data step action lift height " << o.step_feedback.desired_step_spec_saturated.action.lift_height << "\n";
    output << "step data step action toe off " << o.step_feedback.desired_step_spec_saturated.action.toe_off << "\n";
    output << "step data step action knee nominal " << o.step_feedback.desired_step_spec_saturated.action.knee_nominal << "\n";
    output << "step data step action max body accel " << o.step_feedback.desired_step_spec_saturated.action.max_body_accel << "\n";
    output << "step data step action max foot vel " << o.step_feedback.desired_step_spec_saturated.action.max_foot_vel << "\n";
    output << "step data step action sway end dist " << o.step_feedback.desired_step_spec_saturated.action.sway_end_dist << "\n";
    output << "step data step action step end dist " << o.step_feedback.desired_step_spec_saturated.action.step_end_dist << "\n";

    output << "\n --- Walk Feedback ---\n";
    output << " walk feedback time rem " << o.walk_feedback.t_step_rem << "\n";
    output << " walk feedback current step index " << o.walk_feedback.current_step_index << "\n";
    output << " walk feedback next step index needed" << o.walk_feedback.next_step_index_needed << "\n";
    output << " walk feedback status flags" << o.walk_feedback.status_flags << "\n";

    output << "\n --- Walk Feedback step_queue_saturated ---\n";
    for (int i = 0; i < NUM_REQUIRED_WALK_STEPS; ++i) {
        output << "step data step index " << o.walk_feedback.step_queue_saturated[i].step_index << "\n";
        output << "step data foot index " << o.walk_feedback.step_queue_saturated[i].foot_index << "\n";
        output << "step data duration " << o.walk_feedback.step_queue_saturated[i].duration << "\n";
        output << "step data position " << printVector(o.walk_feedback.step_queue_saturated[i].position) << "\n";
        output << "step data yaw " << o.walk_feedback.step_queue_saturated[i].yaw << "\n";
        output << "step data normal " << printVector(o.walk_feedback.step_queue_saturated[i].normal) << "\n";
        output << "step data swing height " << o.walk_feedback.step_queue_saturated[i].swing_height << "\n";
    }

    output << "\n --- Walk Feedback walk_spec_queue_saturated ---\n";
    for (int i = 0; i < NUM_REQUIRED_WALK_STEPS; ++i) {
        output << "\n --- Walk Feedback Walk Spec ---\n";
        output << "step data step index " << o.walk_feedback.walk_spec_queue_saturated[i].step_index << "\n";
        output << "step data foot index " << o.walk_feedback.walk_spec_queue_saturated[i].foot_index << "\n";

        output << "\n --- Walk Feedback Walk Spec Foot Data ---\n";
        output << "step data foot data position " << printVector(o.walk_feedback.walk_spec_queue_saturated[i].foot.position) << "\n";
        output << "step data foot data normal " << printVector(o.walk_feedback.walk_spec_queue_saturated[i].foot.normal) << "\n";
        output << "step data foot data yaw " << o.walk_feedback.walk_spec_queue_saturated[i].foot.yaw << "\n";

        output << "\n --- Walk Feedback Walk Step Action ---\n";
        output << "step data step action step duration " << o.walk_feedback.walk_spec_queue_saturated[i].action.step_duration << "\n";
        output << "step data step action swing height " << o.walk_feedback.walk_spec_queue_saturated[i].action.swing_height << "\n";

    }

    output << "\n --- Manipulation Feedback ---\n";
    output << " manipulation feedback " << o.manipulate_feedback.status_flags <<"\n";
    output << " manipulation feedback clamped pelvis height" << o.manipulate_feedback.clamped.pelvis_height <<"\n";
    output << " manipulation feedback clamped pelvis yaw" << o.manipulate_feedback.clamped.pelvis_yaw <<"\n";
    output << " manipulation feedback clamped pelvis pitch" << o.manipulate_feedback.clamped.pelvis_pitch <<"\n";
    output << " manipulation feedback clamped pelvis roll" << o.manipulate_feedback.clamped.pelvis_roll <<"\n";
    output << " manipulation feedback clamped com v0" << o.manipulate_feedback.clamped.com_v0 <<"\n";
    output << " manipulation feedback clamped com v1" << o.manipulate_feedback.clamped.com_v1 <<"\n";

    output << " manipulation feedback internal desired pelvis height" << o.manipulate_feedback.internal_desired.pelvis_height <<"\n";
    output << " manipulation feedback internal desired pelvis yaw" << o.manipulate_feedback.internal_desired.pelvis_yaw <<"\n";
    output << " manipulation feedback internal desired pelvis pitch" << o.manipulate_feedback.internal_desired.pelvis_pitch <<"\n";
    output << " manipulation feedback internal desired pelvis roll" << o.manipulate_feedback.internal_desired.pelvis_roll <<"\n";
    output << " manipulation feedback internal desired com v0" << o.manipulate_feedback.internal_desired.com_v0 <<"\n";
    output << " manipulation feedback internal desired com v1" << o.manipulate_feedback.internal_desired.com_v1 <<"\n";

    output << " manipulation feedback internal sensed pelvis height" << o.manipulate_feedback.internal_sensed.pelvis_height <<"\n";
    output << " manipulation feedback internal sensed pelvis yaw" << o.manipulate_feedback.internal_sensed.pelvis_yaw <<"\n";
    output << " manipulation feedback internal sensed pelvis pitch" << o.manipulate_feedback.internal_sensed.pelvis_pitch <<"\n";
    output << " manipulation feedback internal sensed pelvis roll" << o.manipulate_feedback.internal_sensed.pelvis_roll <<"\n";
    output << " manipulation feedback internal sensed com v0" << o.manipulate_feedback.internal_sensed.com_v0 <<"\n";
    output << " manipulation feedback internal sensed com v1" << o.manipulate_feedback.internal_sensed.com_v1 <<"\n";


    output << " \n * * * ** * * ** * * ** * * ** * * ** * * ** * * ** * * * \n";





    std::cout<<output.rdbuf();
}

void createAtlasMessage(AtlasControlInput &input, AtlasRobotState &state){
    float f_d[] = {0, -27.6,  0,  0,  0,  0,  -23.5,  -105.7,  24.1,  0,  0,  0,  -23.5,  -105.7,  24.1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 };
    float q_d[] = {0, 0.00225254, 0, -0.1106, -0.00692196, 0.069, -0.472917, 0.932996, -0.440059, -0.0689798, 0.00692196, -0.069, -0.472917, 0.932996, -0.440059, 0.0689798, -0.299682, -1.30067, 1.85276, 0.492914, 0.00165999, -0.000957671, 0.0130531, 0.299682, 1.30067, 1.85276, -0.492914, 0.00165999, 0.000957671, 0.0130531};
    float k_q_p[] = {5000, 5000, 5000, 20, 5, 900, 2000, 2000, 2900, 300, 5, 900, 2000, 2000, 2900, 300, 2000, 1000, 200, 200, 50, 100, 50, 2000, 1000, 200, 200, 50, 100, 50 }    ;

    for(int i = 0 ; i < NUM_JOINTS; i++){
        input.j[i].f_d = f_d[i];
        input.j[i].q_d = q_d[i];
        input.j[i].qd_d = 0;
        input.jparams[i].k_q_p = k_q_p[i];
        input.jparams[i].k_q_i = 0;
        input.jparams[i].k_qd_p = 0;
    }

    input.manipulate_params.use_demo_mode = 0;
    input.manipulate_params.use_desired = 0;
    input.manipulate_params.desired.com_v0 = 0;
    input.manipulate_params.desired.com_v1 = 0;
    input.manipulate_params.desired.pelvis_height = 0;
    input.manipulate_params.desired.pelvis_pitch = 0;
    input.manipulate_params.desired.pelvis_roll = 0;
    input.manipulate_params.desired.pelvis_yaw = 0;

    input.stand_params.placeholder = 0;

    input.step_params.use_demo_walk = 0;
    input.step_params.use_spec = 0;
    input.step_params.use_relative_step_height = 0;
    input.step_params.pelvis_orientation_offset.m_qw = 1;
    input.step_params.pelvis_orientation_offset.m_qx= 0;
    input.step_params.pelvis_orientation_offset.m_qy = 0;
    input.step_params.pelvis_orientation_offset.m_qz = 0;
    input.step_params.desired_step.duration = 0;
    input.step_params.desired_step.foot_index = 0;
    input.step_params.desired_step.normal = AtlasVec3f(0,0, 1);
    input.step_params.desired_step.position = AtlasVec3f(0, 0, 0);

    input.step_params.desired_step_spec.action.knee_nominal = 0;
    input.step_params.desired_step_spec.action.lift_height = 0;
    input.step_params.desired_step_spec.action.max_body_accel = 0;
    input.step_params.desired_step_spec.action.max_foot_vel = 0;
    input.step_params.desired_step_spec.action.step_duration = 0.7;
    input.step_params.desired_step_spec.action.step_end_dist = 0.1;
    input.step_params.desired_step_spec.action.sway_duration = 0.1;
    input.step_params.desired_step_spec.action.sway_end_dist = 0.1;
    input.step_params.desired_step_spec.action.swing_height = 0;
    input.step_params.desired_step_spec.action.toe_off = 1;
    input.step_params.desired_step_spec.foot_index = 0;
    input.step_params.desired_step_spec.step_index = -1;
    input.step_params.desired_step_spec.foot.normal = AtlasVec3f(0,0,1);
    input.step_params.desired_step_spec.foot.position = AtlasVec3f(0, 0, 0);
    input.step_params.desired_step_spec.foot.yaw = 0;

    //    --- Walk Params ---
    input.walk_params.use_demo_walk =0;
    input.walk_params.use_spec =0 ;
    input.walk_params.use_relative_step_height =0;

    for(int i = 0; i < NUM_REQUIRED_WALK_STEPS; i++){
        input.walk_params.step_queue[i].duration =0;
        input.walk_params.step_queue[i].foot_index =0;
        input.walk_params.step_queue[i].step_index =i+1;
        input.walk_params.step_queue[i].swing_height =0;
        input.walk_params.step_queue[i].yaw =0;
        input.walk_params.step_queue[i].normal = AtlasVec3f(0,0,1);
        input.walk_params.step_queue[i].position = AtlasVec3f(0, 0, 0);

        input.walk_params.walk_spec_queue[i].action.step_duration = 0.7;
        input.walk_params.walk_spec_queue[i].action.swing_height = 0;
        input.walk_params.walk_spec_queue[i].foot_index = 0;
        input.walk_params.walk_spec_queue[i].step_index = -1;
        input.walk_params.walk_spec_queue[i].foot.normal= AtlasVec3f(0,0,1);
        input.walk_params.walk_spec_queue[i].foot.position= AtlasVec3f(0,0,0);
        input.walk_params.walk_spec_queue[i].foot.yaw = 0;
    }



    float f[] = {-24.716, -209.265, -70.1586, 5.34532, 0.191892, 11.9869, -765.367, 0.225469, 603.243, 2.92637, -0.0219217, 27.122, -699.417, 0.471382, 605.231, -2.17513, 4.53683, 35.0354, 0.692322, -5.76384, 0.354284, 0.130367, -0.014413, -5.97943, -16.8133, 0.758741, 3.80336, 0.36023, -0.092665, 0.00312316};
    float q[] ={-0.00879109, -0.01640370, -0.00002298, -0.11060000, -0.02743300, 0.07550870, -0.47291700, 0.93300700, -0.44003100, -0.06893710, 0.00694640, -0.06107350, -0.47321700, 0.93302100, -0.44005900, 0.06856740, -0.29958900, -1.30064000, 1.85245000, 0.49290200, 0.00166031, -0.00095674, 0.01305310, 0.29959800, 1.30062000, 1.85242000, -0.49289600, 0.00166037, 0.00095625, 0.01305310};
    float qd[] = {-0.210872, 0, 0, 0, -0.212101, 0, 0, 0, 0, 0, -0.211287, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0	};

    for (int i=0; i < NUM_JOINTS; i++){
        state.j[i].f = f[i];
        state.j[i].q = q[i];
        state.j[i].qd = qd[i];
    }

    state.t = 0.065;
    state.foot_sensors[0].fz = -9728.01;
    state.foot_sensors[0].mx = -10.3991;
    state.foot_sensors[0].my = 600.257;

    state.foot_sensors[1].fz = -10036.8;
    state.foot_sensors[1].mx = 1.50046;
    state.foot_sensors[1].my = 600.676;

    state.imu.imu_timestamp = 6500;
    state.imu.seq_id = 0;
    state.imu.angular_velocity = AtlasVec3f(-0.0681492, 0.19163, 0.211394);
    state.imu.linear_acceleration = AtlasVec3f(15.5433, 6.96587, 14.2996);
    state.imu.orientation_estimate = AtlasQuaternion(0.999679, AtlasVec3f(0.00071175, -0.0253319, 0.000248198));

    state.wrist_sensors[0].f = AtlasVec3f(-8.06964, -19.0157, -27.5865);
    state.wrist_sensors[0].m = AtlasVec3f(-0.828544, -3.93909, 2.85594);

    state.wrist_sensors[1].f = AtlasVec3f(-16.2156, -18.6545, 28.9086);
    state.wrist_sensors[1].m = AtlasVec3f(0.806007, 4.68765, 3.57651);


}

//////////////////////////////////////////////////
AtlasSimInterfaceWrapper::AtlasSimInterfaceWrapper()
{
    //    std::cout<<__PRETTY_FUNCTION__ <<std::endl;
    this->atlasSimInterface = create_atlas_sim_interface();
}

AtlasSimInterfaceWrapper::~AtlasSimInterfaceWrapper()
{
    //    std::cout<<__PRETTY_FUNCTION__ <<std::endl;
    destroy_atlas_sim_interface();
}

//////////////////////////////////////////////////
AtlasControlInput* AtlasSimInterfaceWrapper::getInput()
{

    return &input;
}


AtlasControlOutput* AtlasSimInterfaceWrapper::getOutput()
{

    return &output;
}

AtlasRobotState* AtlasSimInterfaceWrapper::getRobotState()
{

    return &robot_state;
}


int AtlasSimInterfaceWrapper::get_version_major()
{
    //    std::cout<<__PRETTY_FUNCTION__ <<std::endl;
    return this->atlasSimInterface->get_version_major();
}

//////////////////////////////////////////////////
int AtlasSimInterfaceWrapper::get_version_minor()
{
    //    std::cout<<__PRETTY_FUNCTION__ <<std::endl;
    return this->atlasSimInterface->get_version_minor();
}

//////////////////////////////////////////////////
int AtlasSimInterfaceWrapper::get_version_point()
{
    //    std::cout<<__PRETTY_FUNCTION__ <<std::endl;
    return this->atlasSimInterface->get_version_point();
}

//////////////////////////////////////////////////
/// \brief AtlasSimInterfaceWrapper::process_control_input
/// \param control_input
/// \param robot_state
/// \param control_output
/// \return
AtlasErrorCode AtlasSimInterfaceWrapper::process_control_input(
        const AtlasSim::AtlasControlInput& control_input,
        const AtlasSim::AtlasRobotState& robot_state,
        AtlasSim::AtlasControlOutput& control_output)
{
//    std::cout<<"Sending command to simulator"<<std::endl;
    //    printInput(control_input);
    //    printState(robot_state);
    //    AtlasControlInput input ;
    //    AtlasRobotState state ;
    //    createAtlasMessage(input, state);
    //    AtlasSim::AtlasControlOutput out;
    return this->atlasSimInterface->process_control_input(control_input, robot_state, control_output);
    //    std::cout<<__PRETTY_FUNCTION__ <<std::endl;
    //    control_output = out;
    //    return NO_ERRORS;
}

AtlasErrorCode AtlasSimInterfaceWrapper::process_control_input()
{
    std::cout<<"Sending command to simulator using static variables"<<std::endl;
    //    printInput(input);
    //    printState(robot_state);
    return this->atlasSimInterface->process_control_input(input, robot_state, output);
}


//////////////////////////////////////////////////
AtlasErrorCode AtlasSimInterfaceWrapper::reset_control()
{
    std::cout<<__PRETTY_FUNCTION__ <<std::endl;
    return this->atlasSimInterface->reset_control();
}

//////////////////////////////////////////////////
AtlasErrorCode AtlasSimInterfaceWrapper::set_desired_behavior(const char* behavior)
{
    std::cout<<__PRETTY_FUNCTION__ <<std::endl;
    std::cout<<"Setting behaviour to : "<<behavior <<std::endl;
    //    AtlasErrorCode  result = this->atlasSimInterface->set_desired_behavior(std::string(behavior));
    //    std::cout << "Error code :"<<this->atlasSimInterface->get_error_code_text(result)<<std::endl;
    //    return result;
    return this->atlasSimInterface->set_desired_behavior(std::string(behavior).c_str());
}

//////////////////////////////////////////////////
AtlasErrorCode AtlasSimInterfaceWrapper::get_desired_behavior(AtlasBehavior &behavior)
{
    //    std::cout<<__PRETTY_FUNCTION__ <<std::endl;
    this->desired_behavior="";
    AtlasSim::AtlasErrorCode result = this->atlasSimInterface->get_desired_behavior(this->desired_behavior);
//    std::cout <<"Desired Behavior : "<<this->desired_behavior<<std::endl;
    for (size_t i = 0; i < Atlas::NUM_BEHAVIORS; i++){
        if(this->desired_behavior.compare(behaviors[i]) == 0){
            behavior = AtlasBehavior(i);
//            std::cout<<"Desired behavior :" <<behaviors[i]<<std::endl;
            break;
        }
    }
    return result;
}

//////////////////////////////////////////////////
AtlasErrorCode AtlasSimInterfaceWrapper::get_current_behavior(AtlasBehavior &behavior)
{
    //    std::cout<<__PRETTY_FUNCTION__ <<std::endl;
    this->current_behavior="";
    AtlasSim::AtlasErrorCode result = this->atlasSimInterface->get_current_behavior(this->current_behavior);
//    std::cout <<"Current Behavior : "<<this->current_behavior<<std::endl;
    for (size_t i = 0; i < Atlas::NUM_BEHAVIORS; i++){
        if(this->current_behavior.compare(behaviors[i]) == 0){
            behavior = AtlasBehavior(i);
//            std::cout<<"Current behavior :" <<behaviors[i]<<std::endl;
            break;
        }
    }
    return result;
}

const char *AtlasSimInterfaceWrapper::get_behavior_string(Atlas::AtlasBehavior behavior)
{
    if(behavior < NUM_BEHAVIORS)
        return behaviors[(int)behavior].c_str();
    else {
//        std::cout << "behavior "<<behavior<<" not found";
        return "";
    }
}

//////////////////////////////////////////////////
AtlasErrorCode AtlasSimInterfaceWrapper::get_num_behaviors(int& num_behaviors)
{
    //    std::cout<<__PRETTY_FUNCTION__ <<std::endl;
    return this->atlasSimInterface->get_num_behaviors(num_behaviors);
}

//////////////////////////////////////////////////
AtlasErrorCode AtlasSimInterfaceWrapper::get_behavior_at_index(int index,
                                                               AtlasBehavior &behavior)
{
    //    std::cout<<__PRETTY_FUNCTION__ <<std::endl;
    AtlasSim::AtlasErrorCode result = this->atlasSimInterface->get_behavior_at_index(index, this->desired_behavior);
    for (size_t i = 0; i < Atlas::NUM_BEHAVIORS; i++){
        if(this->current_behavior.compare(behaviors[i]) == 0){
            behavior = AtlasBehavior(i);
            break;
        }
    }
    return result;
}

//////////////////////////////////////////////////
AtlasErrorCode AtlasSimInterfaceWrapper::get_behavior_joint_weights(const AtlasBehavior &behavior,
                                                                    float joint_control_weights[NUM_JOINTS])
{
    //    std::cout<<__PRETTY_FUNCTION__ <<std::endl;
    return this->atlasSimInterface->get_behavior_joint_weights(behaviors[behavior],joint_control_weights);
}

//////////////////////////////////////////////////
AtlasErrorCode AtlasSimInterfaceWrapper::get_current_behavior_joint_weights(
        float joint_control_weights[NUM_JOINTS])
{
    //    std::cout<<__PRETTY_FUNCTION__ <<std::endl;
    return this->atlasSimInterface->get_current_behavior_joint_weights(joint_control_weights);
}

//////////////////////////////////////////////////
AtlasErrorCode AtlasSimInterfaceWrapper::get_estimated_position(
        AtlasPositionData& robot_pos_est,
        AtlasVec3f foot_pos_est[Atlas::NUM_FEET])
{
    //    std::cout<<__PRETTY_FUNCTION__ <<std::endl;
    return NO_ERRORS;// this->atlasSimInterface->get_estimated_position(robot_pos_est, foot_pos_est);
}

//////////////////////////////////////////////////
const char* AtlasSimInterfaceWrapper::get_error_code_text(AtlasErrorCode ec)
{
    //    std::cout<<__PRETTY_FUNCTION__ <<std::endl;
    return this->atlasSimInterface->get_error_code_text(ec).c_str();
}


