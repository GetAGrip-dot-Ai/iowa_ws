#include "ros/ros.h"
#include "std_msgs/Float64.h"
// #include "amiga_xarm/Patch.h"
#include "amiga_xarm_msgs/Patch.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include <xarm_api/xarm_driver.h>

#include <iostream>
#include <string>
#include <cmath>
#include <chrono>
#include <thread>

#include <signal.h>

#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>

#define FREQUENCY 50

#define PATCH_TOPIC "scrapper/patch_cmd"

// void subscriberCallback2(const geometry_msgs::Point::ConstPtr& msg) {
    // ROS_INFO_STREAM("" << msg->x);
// }

// void subscriberCallback(const amiga_xarm::Patch::ConstPtr& msg) {
    // ROS_INFO_STREAM("" << msg->position.x);
// }

// int main(int argc, char **argv){
    // ros::init(argc, argv, "eggmass_scrapper");
    // ros::NodeHandle nh("~");
    // ROS_INFO_STREAM("PLAIN CPP NODE] namespace of nh = " << nh.getNamespace());
    // uint32_t queue_size = 1;
    // ros::Subscriber local_subscriber = nh.subscribe("patch_cmd", queue_size, subscriberCallback);
    // ros::spin();
    // return 0;
// }

std::vector<geometry_msgs::Point> get_scrapper_path_sideways(geometry_msgs::Point center, int width, int height);


class ScrapperController {
public:
  ScrapperController(const std::string robot_ip, double frequency);
  void cartesian_servo_trajectory_play(const std::vector<geometry_msgs::Point> &path_);
  void state_update(const xarm_msgs::RobotMsg::ConstPtr &msg);

  void motion_enable()
  {
    ROS_INFO_STREAM("   > enable motion!");
    set_axis_srv_.request.id = 8;
    set_axis_srv_.request.data = 1;
    motion_ctrl_client_.call(set_axis_srv_);
  }

  void set_mode(int mode)
  {
    ROS_INFO_STREAM("   > set mode " << mode << "!");
    set_mode_srv_.request.data = mode;
    set_mode_client_.call(set_mode_srv_);
  }

  void set_state(int state)
  {
    ROS_INFO_STREAM("   > set state " << state << "!");
    set_state_srv_.request.data = state;
    set_state_client_.call(set_state_srv_);
  }

  void clear_co0() {
    set_digital_io_srv_.request.io_num = 1;  
    set_digital_io_srv_.request.value = 0;
    set_dout_gpoio_client_.call(set_digital_io_srv_);
  }

  void set_co0() {
    set_digital_io_srv_.request.io_num = 1;  
    set_digital_io_srv_.request.value = 1;
    set_dout_gpoio_client_.call(set_digital_io_srv_);
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

  void move_scrapper(std::vector<float> pose)
  {
    move_line_scrapper_srv_.request.pose = pose;
    move_line_client_.call(move_line_scrapper_srv_);
  }

  void move_scrapper_path(const std::vector<geometry_msgs::Point>& tcp_path, float speed)
  {
    move_line_scrapper_srv_.request.mvvelo = 100;
    move_line_scrapper_srv_.request.mvacc = 200;
    move_line_scrapper_srv_.request.mvtime = 0;
    move_line_scrapper_srv_.request.mvradii = 0;

    std::this_thread::sleep_for (std::chrono::seconds(5));

    // track path
    // for(auto &next_point: tcp_path){
    for(int k = 0; k < tcp_path.size(); k++){
        auto next_point = tcp_path[k];
        std::vector<float> pose = {next_point.x, next_point.y, next_point.z, 3.1416, -1.5708, 0};
        ROS_INFO_STREAM(" -> " << pose[0] << ", " << pose[1]
            << ", " << pose[2] << ", " << pose[3] << ", " << pose[4] << ", " << pose[5]);
        move_scrapper(pose);
        if (k == 0) {
            std::this_thread::sleep_for (std::chrono::seconds(3));
        } else {
            std::this_thread::sleep_for (std::chrono::seconds(1));
        }
    }
  }

  void move_scrapper_to_pose(std::vector<float> pose, float speed)
  {
    ROS_INFO_STREAM("> move_scrapper_to_pose(...)");
    ROS_INFO_STREAM(" -> " << pose[0] << ", " << pose[1]
        << ", " << pose[2] << ", " << pose[3] << ", " << pose[4] << ", " << pose[5]);
    move_line_scrapper_srv_.request.mvvelo = 80;
    move_line_scrapper_srv_.request.mvacc = 100;
    move_line_scrapper_srv_.request.mvtime = 0;
    move_line_scrapper_srv_.request.mvradii = 0;
    set_mode(0);
    set_state(0);
    move_scrapper(pose);
  }

  void move_scrapper_to_default() {
    auto ne = geometry_msgs::Point();
    ne.x = 500; ne.y = 0; ne.z = 720;
    std::vector<float> pose = {ne.x, ne.y, ne.z, 3.1416, -0.5*3.1416, 0};
    // std::vector<float> pose = {500,0,500,3.1416,-1.5708,0};
    move_line_scrapper_srv_.request.mvvelo = 100;
    move_line_scrapper_srv_.request.mvacc = 200;
    move_line_scrapper_srv_.request.mvtime = 0;
    move_line_scrapper_srv_.request.mvradii = 0;
    ROS_INFO_STREAM("> move_scrapper_to_default()");
    ROS_INFO_STREAM(" -> " << pose[0] << ", " << pose[1]
        << ", " << pose[2] << ", " << pose[3] << ", " << pose[4] << ", " << pose[5]);
    motion_enable();
    set_mode(0);
    set_state(0);
    move_scrapper(pose);
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
  int current_mode;
  int current_state;
  int current_error;
  float frequency;
  bool trigger_state;
  bool replay_trigger_state;
  std::vector<std::vector<float>> camera_positions;
  // void scrapper_callback(const amiga_xarm_msgs::Patch::ConstPtr &);
  void scrapper_callback(const geometry_msgs::PointStamped::ConstPtr &);
  void timer_callback(const ros::TimerEvent &);
  bool hacky_wrong_flag;

  ros::NodeHandle node;

  std::vector<float> requested_deltas;
  std::vector<float> previous_requested_deltas;
  std::vector<float> current_angles_vec;
  std::vector<float> current_pose_vec;

  ros::Publisher vel_pub_;
  ros::Publisher camera_trigger_pub_;
  ros::Subscriber patch_sub_;
  ros::Subscriber robot_state_sub_;
  ros::Timer timer;

  ros::ServiceClient motion_ctrl_client_;
  xarm_msgs::SetAxis set_axis_srv_;

  ros::ServiceClient set_mode_client_;
  xarm_msgs::SetInt16 set_mode_srv_;

  ros::ServiceClient set_state_client_;
  xarm_msgs::SetInt16 set_state_srv_;

  ros::ServiceClient set_dout_gpoio_client_;
  xarm_msgs::SetDigitalIO set_digital_io_srv_;

  ros::ServiceClient clear_error_client_;
  xarm_msgs::ClearErr clear_error_srv_;

  ros::ServiceClient go_home_client_;
  xarm_msgs::Move go_home_move_srv_;

  ros::ServiceClient move_line_client_;
  xarm_msgs::Move move_line_scrapper_srv_;

  ros::ServiceClient velo_move_line_client_;
  xarm_msgs::MoveVelocity cart_timed_velo_srv_;

  ros::ServiceClient set_max_acc_client_;
  xarm_msgs::SetFloat32 set_max_acc_srv_;

  ros::ServiceClient move_servo_cart_client_;
  xarm_msgs::Move move_servo_cart_srv_;

  ros::ServiceClient traj_play_client_;
  xarm_msgs::PlayTraj play_traj_srv_;
};

ScrapperController::ScrapperController(const std::string robot_ip /* = "192.168.1.213" */, 
                                       double frequency = 100.0) {
  hacky_wrong_flag = true;
  // std::string filename = "/home/husky-xarm/ref_joints";
  // camera_positions = get_positions(filename);
  frequency = frequency;
  requested_deltas = {0, 0, 0, 0, 0, 0};
  previous_requested_deltas = {0, 0, 0, 0, 0, 0};

  current_mode = 4;
  current_state = 0;
  current_error= 0;

  current_angles_vec = {0, 0, 0, 0, 0, 0};
  current_pose_vec = {0, 0, 0, 0, 0, 0};

  clear_error();
  motion_enable();
  set_mode(0);
  set_state(0);

  motion_ctrl_client_ = node.serviceClient<xarm_msgs::SetAxis>("/xarm/motion_ctrl");
  set_mode_client_ = node.serviceClient<xarm_msgs::SetInt16>("/xarm/set_mode");
  set_state_client_ = node.serviceClient<xarm_msgs::SetInt16>("/xarm/set_state");
  clear_error_client_ = node.serviceClient<xarm_msgs::ClearErr>("/xarm/clear_err");

  move_servo_cart_client_ = node.serviceClient<xarm_msgs::Move>("/xarm/move_servo_cart");
  move_line_client_ = node.serviceClient<xarm_msgs::Move>("/xarm/move_line");
  set_dout_gpoio_client_ = node.serviceClient<xarm_msgs::SetDigitalIO>("/xarm/set_controller_dout");

  velo_move_line_client_ = node.serviceClient<xarm_msgs::MoveVelocity>("/xarm/velo_move_line_timed");
  set_max_acc_client_ = node.serviceClient<xarm_msgs::SetFloat32>("/xarm/set_max_acc_line");
  traj_play_client_ = node.serviceClient<xarm_msgs::PlayTraj>("/xarm/play_traj");

  robot_state_sub_ = node.subscribe<xarm_msgs::RobotMsg>("/xarm/xarm_states", 3, &ScrapperController::state_update, this);
  // patch_sub_ = node.subscribe<amiga_xarm_msgs::Patch>(PATCH_TOPIC, 1, &ScrapperController::scrapper_callback, this);
  patch_sub_ = node.subscribe<geometry_msgs::PointStamped>(PATCH_TOPIC, 1, &ScrapperController::scrapper_callback, this);
  timer = node.createTimer(ros::Duration(1.0 / frequency), &ScrapperController::timer_callback, this);
  // timer = nh_.createTimer(ros::Duration(1.0/frequency), std::bind(&ScrapperController::timer_callback, this));

  camera_trigger_pub_ = node.advertise<std_msgs::String>("commands_to_syncbox", 10);

  set_mode(1);
  set_state(0);
  sleep_milliseconds(2000);

  requested_deltas = {0, 0, 0, 0, 0, 0};
  previous_requested_deltas = {0, 0, 0, 0, 0, 0};
  trigger_state = false;
    // to default
    move_scrapper_to_default();
    std::this_thread::sleep_for (std::chrono::seconds(5));
    for (int k = 0; k < 2; k++) {
        set_co0();
        std::this_thread::sleep_for (std::chrono::seconds(1));
        clear_co0();
        std::this_thread::sleep_for (std::chrono::seconds(1));
    }
    // auto center = geometry_msgs::Point();
    // center.x = 600;
    // center.y = 0;
    // center.z = 450;
    // auto path_ = get_scrapper_path_sideways(center, 50, 100);
    // move_scrapper_path(path_, 20);
}

void ScrapperController::state_update(const xarm_msgs::RobotMsg::ConstPtr &msg) {
    current_mode = msg->mode;
    current_state = msg->state;
    current_error = msg->err;
    for (int k = 0; k < 6; k++){
        current_pose_vec[k] = msg->pose[k];
        // WARN/TODO: angles will change base on robot DOF
        current_angles_vec[k] = msg->angle[k];
    }
}

void ScrapperController::cartesian_servo_trajectory_play(const std::vector<geometry_msgs::Point> &path_) {
  // cartesian_servo(requested_deltas);
}


std::vector<geometry_msgs::Point> get_scrapper_path_sideways(geometry_msgs::Point center, int width, int height){
  width = 100;
  height = 100;
  int height_step = 20;

    auto path_ = std::vector<geometry_msgs::Point>();
    auto NW = geometry_msgs::Point();
    NW.x = center.x + 25;
    NW.y = center.y; //  + width/2;
    NW.z = center.z; //  + height/2;
    path_.emplace_back(NW);

    int num_horizontal_passes = 5;

    for(int k = 0; k <= num_horizontal_passes; k++) {
        auto nn = geometry_msgs::Point();
        auto start = path_.back();
        nn.x = path_.back().x; // x is unchanged
        nn.y = path_.back().y - width;
        nn.z = path_.back().z;
        path_.emplace_back(nn);
        path_.emplace_back(start);
        if (k != (num_horizontal_passes-1)) {
          nn.x = start.x; // x is unchanged
          nn.y = start.y;
          nn.z = start.z - height_step;
          path_.emplace_back(nn);
        }
    }

    return path_;
}

std::vector<geometry_msgs::Point> get_scrapper_path(geometry_msgs::Point NE, int width, int height){
    auto path_ = std::vector<geometry_msgs::Point>();
    path_.emplace_back(NE);
    // add NE
    // add NE - height
    int width_step = 0; 
    int num_vertical_passes = 3;
    if (width_step != 0) {
        num_vertical_passes = 1.0 + width/width_step;
    }

    for(int k = 0; k <= num_vertical_passes; k++) {
        auto dir = ((k % 2) == 0) ? -1 : 1;
        // add vertical end
        auto next_vertical_point = geometry_msgs::Point();
        next_vertical_point.x = path_.back().x;                   // x is unchanged
        next_vertical_point.y = path_.back().y;
        next_vertical_point.z = path_.back().z + dir * height;
        path_.emplace_back(next_vertical_point);
        // add next vertical start
        auto next_horizontal_point = geometry_msgs::Point();
        next_horizontal_point.x = path_.back().x;                   // x is unchanged
        next_horizontal_point.y = path_.back().y + width_step;
        next_horizontal_point.z = path_.back().z;
        path_.emplace_back(next_horizontal_point);
    }

    return path_;
}

// void ScrapperController::scrapper_callback(const amiga_xarm_msgs::Patch::ConstPtr &patch) {
void ScrapperController::scrapper_callback(const geometry_msgs::PointStamped::ConstPtr &patch) {
  if (!hacky_wrong_flag) return;
  hacky_wrong_flag = false;
    // TODO
    ROS_INFO_STREAM(" Target Position: " 
            << patch->point.x << ", " 
            << patch->point.y << ", " 
            << patch->point.z);
    auto north_east = geometry_msgs::Point();
    north_east.x = patch->point.x;
    north_east.y = patch->point.y;
    north_east.z = patch->point.z;
    // auto north_east = geometry_msgs::Point();
    // north_east.x = 750; north_east.y = 0; north_east.z = 500;
    std::vector<float> target_pose = {north_east.x, north_east.y, north_east.z, 3.1416, -0.5*3.1416, 0};
    int width = 10;
    int height = 60;
    auto tcp_path = get_scrapper_path_sideways(north_east, width, /*height*/ 100);
    // auto tcp_path = get_scrapper_path(north_east, width, /*height*/ 100);
    // auto tcp_path = get_scrapper_path(north_east, patch->width, patch->height);

    float speed = 100;
    ROS_INFO_STREAM("Moving to default -> ");
    // to target
    move_scrapper_to_pose(target_pose, speed);

    /* scrapping action */ {
        set_co0();
        std::this_thread::sleep_for (std::chrono::seconds(1));
        move_scrapper_path(tcp_path, speed);
        clear_co0();
    }
    clear_co0();
    clear_co0();
    // to default
    move_scrapper_to_default();
    std::this_thread::sleep_for (std::chrono::seconds(15));
    hacky_wrong_flag = true;
}

void ScrapperController::timer_callback(const ros::TimerEvent &)
{
  // cartesian_servo(requested_deltas);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "eggmass_scrapper");
  const std::string robot_ip = "192.168.1.213";
  ScrapperController scrapper(robot_ip, FREQUENCY);
  ros::spin();
}
