#include "ros/ros.h"
#include <xarm_api/xarm_driver.h>

#include <iostream>
#include <string>
#include <cmath>
#include <signal.h>

#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>

#define FREQUENCY 50
#define JOYSTICK_TOPIC "joy_relay"

class JoystickController
{
public:
  JoystickController(const std::string robot_ip, double frequency);

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
    float duration = 1.0;

    cart_timed_velo_srv_.request.speeds = twist_vec;
    cart_timed_velo_srv_.request.is_tool_coord = is_tool_coord;
    cart_timed_velo_srv_.request.duration = duration;
    velo_move_line_client_.call(cart_timed_velo_srv_);
  }

  void cartesian_servo(std::vector<float> move)
  {
    move_servo_cart_srv_.request.pose = move;
    move_servo_cart_srv_.request.mvtime = 1;
    ROS_WARN_STREAM(move[0]);
    move_servo_cart_client_.call(move_servo_cart_srv_);
  }

  void camera_send_serial(std::string send)
  {
    std_msgs::String t;
    t.data = send.c_str();
    camera_trigger_pub_.publish(t);
  }

  void camera_single_shot()
  {
    camera_send_serial("t");
    // sleep(0.5);
  }

  void camera_repeat_shot(int hz)
  {
    std::cout << "camera repeated shot running at "
              << hz << "hz\n";
    camera_send_serial("c");
    sleep(1);
    // std::cout << ""
    camera_send_serial(std::to_string(hz));
    sleep(1);
    camera_send_serial("r");
    std::cout << "done sending command to camera\n";
    // sleep(0.5);
  }

  void stop_camera()
  {
    camera_send_serial("s");
    // sleep(0.5);
  }

  void replay(std::string filename, int repeat_times = 1, int speed_factor = 1)
  {
    // TODO: read the current mode instead of assuming mode 1
    std::cout << " > trajectory play: " << filename << std::endl;
    clear_error();
    motion_enable();
    set_mode(0);
    set_state(0);
    node.setParam("/xarm/wait_for_finish", true);
    // sleep_milliseconds(500);
    //  ---
    play_traj_srv_.request.traj_file = filename;
    play_traj_srv_.request.repeat_times = repeat_times;
    play_traj_srv_.request.speed_factor = speed_factor;
    traj_play_client_.call(play_traj_srv_);
    std::cout << " >> play_traj_srv_.response.ret = " << play_traj_srv_.response.ret << std::endl;
    // ---
    // sleep_milliseconds(500);
    clear_error();
    node.setParam("/xarm/wait_for_finish", false);
    // motion_enable();
    set_mode(1);
    set_state(0);
    std::cout << " --- " << std::endl;
  }

private:
  float frequency;
  bool trigger_state;
  bool replay_trigger_state;
  std::vector<std::vector<float>> camera_positions;
  void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);
  void timer_Callback(const ros::TimerEvent &);

  ros::NodeHandle node;

  std::vector<float> requested_deltas;
  std::vector<float> previous_requested_deltas;

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
  requested_deltas = {0, 0, 0, 0, 0, 0};
  previous_requested_deltas = {0, 0, 0, 0, 0, 0};

  clear_error();
  motion_enable();
  set_mode(0);
  set_state(0);

  motion_ctrl_client_ = node.serviceClient<xarm_msgs::SetAxis>("/xarm/motion_ctrl");
  set_mode_client_ = node.serviceClient<xarm_msgs::SetInt16>("/xarm/set_mode");
  set_state_client_ = node.serviceClient<xarm_msgs::SetInt16>("/xarm/set_state");
  clear_error_client_ = node.serviceClient<xarm_msgs::ClearErr>("/xarm/clear_err");
  velo_move_line_client_ = node.serviceClient<xarm_msgs::MoveVelocity>("/xarm/velo_move_line_timed");
  set_max_acc_client_ = node.serviceClient<xarm_msgs::SetFloat32>("/xarm/set_max_acc_line");
  move_servo_cart_client_ = node.serviceClient<xarm_msgs::Move>("/xarm/move_servo_cart");
  traj_play_client_ = node.serviceClient<xarm_msgs::PlayTraj>("/xarm/play_traj");

  joy_sub_ = node.subscribe<sensor_msgs::Joy>(JOYSTICK_TOPIC, 1, &JoystickController::joyCallback, this);
  timer = node.createTimer(ros::Duration(1.0 / frequency), &JoystickController::timer_Callback, this);
  // timer = nh_.createTimer(ros::Duration(1.0/frequency), std::bind(&JoystickController::timer_Callback, this));

  camera_trigger_pub_ = node.advertise<std_msgs::String>("commands_to_syncbox", 10);

  set_mode(1);
  set_state(0);
  sleep_milliseconds(2000);

  requested_deltas = {0, 0, 0, 0, 0, 0};
  previous_requested_deltas = {0, 0, 0, 0, 0, 0};
  trigger_state = false;
}

void JoystickController::joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
  ROS_WARN_STREAM("in callback!!!!");
  // check if the reset button is pressed
  
  bool reset_pressed = joy->buttons[7];

  bool drive_husky_slow = joy->buttons[6];
  bool drive_husky_fast = joy->buttons[7];

  bool trigger_camera = joy->buttons[0];

  float left_stick_x = joy->axes[0];
  float left_stick_y = joy->axes[1];

  float right_stick_x = joy->axes[3];
  float right_stick_y = joy->axes[4];

  float d_pad_x = joy->axes[6];
  float d_pad_y = joy->axes[7];

  if (!(reset_pressed || drive_husky_slow || drive_husky_fast))
  {
    const float max_linear_speed = 100.0 / 100;                 // mm/s
    const float max_rotation_speed = 22.5 * (3.14 / 180) / 100; // deg/s

    const float max_linear_acc = 10000;

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
  }
  else if (reset_pressed)
  {
    std::cout << " >> Errors cleared\n";
    clear_error();
    sleep_milliseconds(100);
    set_mode(1);
    set_state(0);
    sleep_milliseconds(100);
  }
  else
  {
    requested_deltas = {0, 0, 0, 0, 0, 0};
  }

  // Camera trigger
  bool trigger_pressed = (!trigger_state) && trigger_camera;
  std::cout << "trigger states: {"
            << trigger_state << ", "
            << trigger_camera << "}\n";

  if (trigger_pressed)
  {
    std::cout << " >> Image Trigger!!\n";
    camera_single_shot();
  }
  trigger_state = trigger_camera;

  // // Replay Trigger ===
  // bool replay_trigger_released = (joy->buttons[7] != replay_trigger_state) && joy->buttons[7];
  // if (replay_trigger_released)
  // {
  //   camera_repeat_shot(5);
  //   replay("test.traj");
  //   stop_camera();
  // }
  // replay_trigger_state = joy->buttons[7];
}

void JoystickController::timer_Callback(const ros::TimerEvent &)
{
  cartesian_servo(requested_deltas);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "xarm_jostick_teleop_test");
  const std::string robot_ip = "192.168.1.196";
  JoystickController teleop_turtle(robot_ip, FREQUENCY);
  ros::spin();
}
