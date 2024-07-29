#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/info/info.h>

#include <chrono>
#include <functional>
#include <future>
#include <iostream>
#include <thread>
#include <string>
#include <utility>
#include <cmath>

#include <stdint.h>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>

#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::placeholders;
using namespace mavsdk;
using std::chrono::seconds;
using std::this_thread::sleep_for;

namespace mission
{
    class FlyMission : public rclcpp::Node
    {
    public:
        FlyMission();
    private:
        std::unique_ptr<mavsdk::Mavsdk> _mavsdk;
        std::unique_ptr<mavsdk::Action> _action;
        std::shared_ptr<mavsdk::Telemetry> _telemetry;
        std::shared_ptr<mavsdk::Mission> _mission;
        std::unique_ptr<mavsdk::Offboard> _offboard;
        std::vector<mavsdk::Mission::MissionItem> mission_items;

        std::atomic<int> waypoint;
        std::atomic<float> next_waypoint_latitude;
        std::atomic<float> next_waypoint_longitude;
        std::atomic<float> last_waypoint_latitude;
        std::atomic<float> last_waypoint_longitude;

        mavsdk::Telemetry::Position drone_start_pos;
        mavsdk::Telemetry::Position drone_pos;
        mavsdk::Telemetry::Position drone_pos_avoid;

        std::atomic<float> drone_latitude;
        std::atomic<float> drone_longitude;
        std::atomic<float> drone_avoid_latitude;
        std::atomic<float> drone_avoid_longitude;

        std::atomic<float> drone_lat_norm;
        std::atomic<float> drone_lon_norm;

        std::atomic<float> depthValue_center;
        std::atomic<float> depthValue_left;
        std::atomic<float> depthValue_right;

        float depth_threshold_center;
        float depth_threshold_side;

        bool in_air;
        int state = 0;
        double R = 6371000.0;
        bool avoid_right;
        
        float distance_avoid = 0;
        bool flag_avoid = false;
        int trasa;
        double total_distance;

        std::chrono::time_point<std::chrono::steady_clock> start_time_;
        std::chrono::time_point<std::chrono::steady_clock> end_time_;

        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _srvUpload;
        void cbUpload(const std::shared_ptr<std_srvs::srv::Trigger::Request> aRequest, const std::shared_ptr<std_srvs::srv::Trigger::Response> aResponse);

        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _srvTakeOff;
        void cbTakeOff(const std::shared_ptr<std_srvs::srv::Trigger::Request> aRequest, const std::shared_ptr<std_srvs::srv::Trigger::Response> aResponse);

        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _srvStartMission;
        void cbStartMission(const std::shared_ptr<std_srvs::srv::Trigger::Request> aRequest, const std::shared_ptr<std_srvs::srv::Trigger::Response> aResponse);

        void upload();
        void takeOff();
        std::shared_ptr<mavsdk::System> getSystem(mavsdk::Mavsdk& aMavsdk);

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _depthSub;
        void cbDepth(const sensor_msgs::msg::Image::SharedPtr msg);

        void position();
        void avoid();
    };
}