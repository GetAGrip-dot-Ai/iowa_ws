/* Copyright 2018 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
 ============================================================================*/
#include "ros/ros.h"
#include "xarm/wrapper/xarm_api.h"
#include "xarm_api/xarm_driver.h"
#include "xarm_api/xarm_ros_client.h"
// #include "xarm_api/csv.h"

#include <iostream>
#include <string>
#include <cmath>
#include <signal.h>

#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>

template <typename T>
int sgn(T val)
{
  return (T(0) < val) - (val < T(0));
}

/*
std::vector<std::vector<float>> get_positions(std::string filename) {
    // std::string filename = "/home/husky-xarm/ref_joints";
    std::string line;
    std::ifstream file_stream (filename);
    std::vector<std::vector<float>> positions;
    if (file_stream.is_open()) {
        while (getline(file_stream, line)) {
            // std::cout << line << '\n';
            std::stringstream sstr(line);
            std::vector<float> v;
            while(sstr.good()) {
                std::string substr;
                getline(sstr, substr, ',');
                v.emplace_back(std::stof(substr));
            }
            positions.emplace_back(v);
            // for (std::size_t i = 0; i < v.size(); i++) std::cout << v[i] << std::endl;
        }
        file_stream.close();
    }

    return positions;
}
*/

class JoystickController
{
public:
  JoystickController(const std::string robot_ip, double frequency);
  // ~JoystickController() {
  // arm->set_mode(0);
  // arm->set_state(0);
  // sleep_milliseconds(1000);
  // float stop_velocity[6] = {0, 0, 0, 0, 0, 0};
  // int ret = arm->vc_set_cartesian_velocity(stop_velocity);
  // printf("vc_set_cartesian_velocity, ret=%d\n", ret);
  // printf(" JoystickController Exited\n");
  // delete arm;
  // }

  /*
  void reset() {
  set_axis_srv_.request.id = 8;
    set_axis_srv_.request.data = en;
    motion_ctrl_client_.call(set_axis_srv_);
  }
  */

  void motion_enable()
  {
    set_axis_srv_.request.id = 8;
    set_axis_srv_.request.data = 1;
    motion_ctrl_client_.call(set_axis_srv_);
  }

  void set_mode(int mode)
  {
    set_mode_srv_.request.data = mode;
    set_mode_client_.call(set_mode_srv_);
  }

  void set_state(int state)
  {
    set_state_srv_.request.data = state;
    set_state_client_.call(set_state_srv_);
  }

  void set_max_acc_line(float max_acc)
  {
    set_max_acc_srv_.request.data = max_acc;
    set_max_acc_client_.call(set_max_acc_srv_);
  }

  void clear_error()
  {
    clear_error_client_.call(clear_error_srv_);
  }

  void go_home()
  {
    float jnt_vel_rad = 0.1;
    float jnt_acc_rad = 15;
    go_home_move_srv_.request.mvvelo = jnt_vel_rad;
    go_home_move_srv_.request.mvacc = jnt_acc_rad;
    go_home_move_srv_.request.mvtime = 0;
    go_home_client_.call(go_home_move_srv_);
  }

  void cartesian_move_timed(std::vector<float> twist_vec)
  {
    bool is_tool_coord = true;
    uint8_t is_sync = 2;
    float duration = 1.0;
    /*
      move_velo_srv_.request.is_sync = true;
      move_velo_srv_.request.speeds = line_v;
      move_velo_srv_.request.is_tool_coord = is_tool_coord;
      move_velo_srv_.request.duration = duration;
      // _call_service(velo_move_line_client_, move_velo_srv_);
      velo_move_line_client_.call(move_velo_srv_);
      // return move_velo_srv_.response.ret;
    */
    cart_timed_velo_srv_.request.speeds = twist_vec;
    cart_timed_velo_srv_.request.is_tool_coord = is_tool_coord;
    cart_timed_velo_srv_.request.is_sync = is_sync;
    cart_timed_velo_srv_.request.duration = duration;
    velo_move_line_client_.call(cart_timed_velo_srv_);

    // std::cout << " >> cart_timed_velo_srv_.response.ret = " << cart_timed_velo_srv_.response.ret << std::endl;
  }

  void cartesian_servo(std::vector<float> move)
  {
    // req = MoveRequest()
    // req.pose = list(get_position().datas)
    // req.mvvelo = 0
    // req.mvacc = 0
    // req.mvtime = 0
    // loop_num = time_secs*float(freq)
    // sleep_sec = 1.0/float(freq)
    // ret = 0
    // step = 0.3
    // move_servo_cart_
  }

  void camera_single_shot()
  {
    std_msgs::String t;
    t.data = "t";
    camera_trigger_pub_.publish(t);
    sleep(0.5);
  }

  void replay(std::string filename, int repeat_times = 1, int speed_factor = 1)
  {
    std::cout << " > trajectory play: " << filename << std::endl;
    clear_error();
    motion_enable();
    set_mode(0);
    set_state(0);
    sleep_milliseconds(500);
    // ---
    play_traj_srv_.request.traj_file = filename;
    play_traj_srv_.request.repeat_times = repeat_times;
    play_traj_srv_.request.speed_factor = speed_factor;
    traj_play_client_.call(play_traj_srv_);
    std::cout << " >> play_traj_srv_.response.ret = " << play_traj_srv_.response.ret << std::endl;
    // ---
    sleep_milliseconds(500);
    clear_error();
    // motion_enable();
    set_mode(5);
    set_state(0);
    std::cout << " --- " << std::endl;
  }

private:
  bool trigger_state;
  bool replay_trigger_state;
  std::vector<std::vector<float>> camera_positions;
  void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);
  void timer_Callback(const ros::TimerEvent &);

  ros::NodeHandle nh_;
  // XArmAPI *arm;
  // xarm_api::XArmROSClient xarm_client;

  int linear_, angular_;
  double l_scale_, a_scale_;
  float frequency;
  std::vector<float> requested_deltas;
  // cartesian_move_timed(line_v);

  ros::Publisher vel_pub_;
  ros::Publisher camera_trigger_pub_;
  ros::Subscriber joy_sub_;
  ros::Timer timer;

  ros::ServiceClient motion_ctrl_client_;
  xarm_msgs::SetAxis set_axis_srv_;

  ros::ServiceClient set_mode_client_;
  xarm_msgs::SetInt16 set_mode_srv_;

  ros::ServiceClient set_state_client_;
  xarm_msgs::SetInt16 set_state_srv_;

  ros::ServiceClient clear_error_client_;
  xarm_msgs::ClearErr clear_error_srv_;

  ros::ServiceClient go_home_client_;
  xarm_msgs::Move go_home_move_srv_;

  ros::ServiceClient velo_move_line_client_;
  xarm_msgs::MoveVelocity cart_timed_velo_srv_;

  ros::ServiceClient set_max_acc_client_;
  xarm_msgs::SetFloat32 set_max_acc_srv_;

  ros::ServiceClient move_servo_cart_client_;
  xarm_msgs::Move move_servo_cart_srv_;

  ros::ServiceClient traj_play_client_;
  xarm_msgs::PlayTraj play_traj_srv_;
};

JoystickController::JoystickController(const std::string robot_ip /* = "192.168.1.213" */, double frequency = 100.0)
{
  std::string filename = "/home/husky-xarm/ref_joints";
  // camera_positions = get_positions(filename);
  frequency = frequency;
  requested_deltas.emplace_back(0);
  requested_deltas.emplace_back(0);
  requested_deltas.emplace_back(0);
  requested_deltas.emplace_back(0);
  requested_deltas.emplace_back(0);
  requested_deltas.emplace_back(0);

  clear_error();
  motion_enable();
  set_mode(0);
  set_state(0);

  motion_ctrl_client_ = nh_.serviceClient<xarm_msgs::SetAxis>("/xarm/motion_ctrl");
  set_mode_client_ = nh_.serviceClient<xarm_msgs::SetInt16>("/xarm/set_mode");
  set_state_client_ = nh_.serviceClient<xarm_msgs::SetInt16>("/xarm/set_state");
  clear_error_client_ = nh_.serviceClient<xarm_msgs::ClearErr>("/xarm/clear_err");
  velo_move_line_client_ = nh_.serviceClient<xarm_msgs::MoveVelocity>("/xarm/velo_move_line_timed");
  set_max_acc_client_ = nh_.serviceClient<xarm_msgs::SetFloat32>("/xarm/set_max_acc_line");
  move_servo_cart_client_ = nh_.serviceClient<xarm_msgs::Move>("/xarm/move_servo_cart");
  traj_play_client_ = nh_.serviceClient<xarm_msgs::PlayTraj>("/xarm/play_traj");

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy_teleop/joy", 1, &JoystickController::joyCallback, this);
  timer = nh_.createTimer(ros::Duration(1.0 / frequency), &JoystickController::timer_Callback, this);
  // timer = nh_.createTimer(ros::Duration(1.0/frequency), std::bind(&JoystickController::timer_Callback, this));

  camera_trigger_pub_ = nh_.advertise<std_msgs::String>("commands_to_syncbox", 10);

  set_mode(5);
  set_state(0);
  sleep_milliseconds(2000);

  set_max_acc_line(50000.0);
  requested_deltas = {0, 0, 0, 0, 0, 0};
}

void JoystickController::joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
  // check if the reset button is pressed
  bool reset_pressed = joy->buttons[6];
  bool drive_husky_slow = joy->buttons[4];
  bool drive_husky_fast = joy->buttons[5];

  float left_stick_x = joy->axes[0];
  float left_stick_y = joy->axes[1];

  float right_stick_x = joy->axes[3];
  float right_stick_y = joy->axes[4];

  float d_pad_x = joy->axes[6];
  std::cout << d_pad_x << std::endl;
  float d_pad_y = joy->axes[7];

  if (!(reset_pressed || drive_husky_slow || drive_husky_fast))
  {
    const float max_linear_speed = 700.0; // mm/s
    const float max_rotation_speed = 0.2; // rad/s

    float x_ax = max_linear_speed * d_pad_y;
    float y_ax = -max_linear_speed * d_pad_x;
    float z_ax = max_linear_speed * std::pow(right_stick_y, 3);

    // x_ax = std::abs(x_ax) < 0.2 * max_linear_speed ? 0 : x_ax;
    // y_ax = std::abs(y_ax) < 0.2 * max_linear_speed ? 0 : y_ax;
    // z_ax = std::abs(z_ax) < 0.2 * max_linear_speed ? 0 : z_ax;

    float w_x_ax = -max_rotation_speed * std::pow(left_stick_x, 3);
    float w_y_ax = -max_rotation_speed * std::pow(left_stick_y, 3);
    float w_z_ax = -max_rotation_speed * std::pow(right_stick_x, 3);

    std::cout << "\t\t {x_ax y_ax z_ax} = {"
              << x_ax << ", "
              << y_ax << ", "
              << z_ax << "}" << std::endl;

    requested_deltas = {x_ax, y_ax, z_ax, w_x_ax, w_y_ax, w_z_ax};

    // print_nvect(" >> tcp_spd_limit:  ", arm->tcp_speed_limit, 2);
    // print_nvect(" >> tcp_acc_limit:  ", arm->tcp_acc_limit, 2);
    // printf(" >> arm->tcp_jerk = %f\n", arm->tcp_jerk);
  }
  else if (reset_pressed)
  {
    // sleep_milliseconds(500);
    // if (arm->error_code != 0) arm->clean_error();
    // if (arm->warn_code != 0) arm->clean_warn();
    std::cout << " >> Clear Error\n";
    clear_error();
    sleep_milliseconds(100);
    set_mode(5);
    set_state(0);
    sleep_milliseconds(100);
  }
  else
  {
    requested_deltas = {0, 0, 0, 0, 0, 0};
  }

  // Camera Trigger ===
  bool trigger_released = (joy->buttons[1] != trigger_state) && joy->buttons[1];
  if (trigger_released)
  {
    camera_single_shot();
    std::cout << " >> Image Trigger!!\n";
  }
  trigger_state = joy->buttons[1];

  // Replay Trigger ===
  bool replay_trigger_released = (joy->buttons[2] != replay_trigger_state) && joy->buttons[2];
  if (replay_trigger_released)
  {
    replay("test.traj");
  }
  replay_trigger_state = joy->buttons[2];
}

void JoystickController::timer_Callback(const ros::TimerEvent &)
{
  // std::cout << "\t\t joy->axes[3] = " << joy->axes[3] << std::endl;
  // std::cout << "\t\t {x_ax y_ax z_ax} = {"
  //     << line_v.at(0) << ", "
  //     << line_v.at(1) << ", "
  //     << line_v.at(2) << "}" << std::endl;
  set_max_acc_line(50000.0);
  cartesian_move_timed(requested_deltas);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "xarm_jostick_teleop_node");
  const std::string robot_ip = "192.168.1.214";
  double frequency = 10.0;
  JoystickController teleop_turtle(robot_ip, frequency);
  ros::spin();
}
