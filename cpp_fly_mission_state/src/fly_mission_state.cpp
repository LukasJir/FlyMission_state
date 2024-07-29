#include "cpp_fly_mission_state/fly_mission_state.hpp" 

using namespace std::placeholders;
using namespace mavsdk;
using std::chrono::seconds;
using std::this_thread::sleep_for;

mavsdk::Mission::MissionItem make_mission_item(
    double latitude_deg,  //zemepisna sirka ve stupnich
    double longitude_deg,  //zemepisna vyska ve stupnich
    float relative_altitude_m,  //nadmorska vyska relativni k vysce takeoffu, metry
    float speed_m_s,  //rychlost k dalsimu waypointu
    bool is_fly_through,  //true = proleti bez zastaveni, false = zastavi na waypointu
    float gimbal_pitch_deg,  //"stoupani" gimbalu ve stupnich
    float gimbal_yaw_deg,  //"natoceni" gimbalu ve stupnich
    Mission::MissionItem::CameraAction camera_action)
{
    Mission::MissionItem new_item{};
    new_item.latitude_deg = latitude_deg;
    new_item.longitude_deg = longitude_deg;
    new_item.relative_altitude_m = relative_altitude_m;
    new_item.speed_m_s = speed_m_s;
    new_item.is_fly_through = is_fly_through;
    new_item.gimbal_pitch_deg = gimbal_pitch_deg;
    new_item.gimbal_yaw_deg = gimbal_yaw_deg;
    new_item.camera_action = camera_action;
    return new_item;
}

namespace mission
{
    FlyMission::FlyMission() : Node("FlyMission")
    {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        _mavsdk = std::make_unique<mavsdk::Mavsdk>();

        mavsdk::ConnectionResult connectionResult = _mavsdk.get()->add_any_connection("udp://:14541");

        if(connectionResult != mavsdk::ConnectionResult::Success)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Connection failed");

            throw std::runtime_error("Connection failed");
        }

        auto system = getSystem(*_mavsdk);

        if(system == nullptr)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Timed out waiting for system");

            throw std::runtime_error("Timed out waiting for system");
        }

        _action = std::make_unique<mavsdk::Action>(system);
        _telemetry = std::make_shared<mavsdk::Telemetry>(system);
        _mission = std::make_unique<mavsdk::Mission>(system);
        _offboard = std::make_unique<mavsdk::Offboard>(system);

        _srvUpload = this->create_service<std_srvs::srv::Trigger>("mission_flier/upload", std::bind(&FlyMission::cbUpload, this, _1, _2));
        _srvTakeOff = this->create_service<std_srvs::srv::Trigger>("mission_flier/take_off", std::bind(&FlyMission::cbTakeOff, this, _1, _2));
        _srvStartMission = this->create_service<std_srvs::srv::Trigger>("mission_flier/start_mission", std::bind(&FlyMission::cbStartMission, this, _1, _2));
        _depthSub = this->create_subscription<sensor_msgs::msg::Image>("/depth_camera", 10, std::bind(&FlyMission::cbDepth, this, _1));
    }

    void FlyMission::cbDepth(const sensor_msgs::msg::Image::SharedPtr msg) 
    {
        uint32_t width = msg->width;
        uint32_t height = msg->height;
        uint32_t x_center = width/2;
        uint32_t y_center = height/2;
        uint32_t x_left = width/5;
        uint32_t x_right = 4*width/5;
        const float* depthData = reinterpret_cast<const float*>(msg->data.data());

        auto find_min_depth = [&](uint32_t x, uint32_t y, int offset_x, int offset_y){  //funkce pro nalezeni minimalni hodnoty hloubky obrazu, pro danou cast projde 25 bodu v obraze (5x5)
            float min_depth = std::numeric_limits<float>::max();
            for(int i = -2; i <= 2; ++i){
                for(int j = -2; j <= 2; ++j){
                    size_t index = (y + offset_y * i) * width + (x + offset_x * j);
                    if(index < width * height){
                        min_depth = std::min(min_depth, depthData[index]);
                    }
                }
            }
            return min_depth;
        };

        depthValue_center = find_min_depth(x_center, y_center, 40, 10);     //hodnoty hloubek veprostred, v leve a prave casti
        depthValue_left = find_min_depth(x_left, y_center, 40, 10);
        depthValue_right = find_min_depth(x_right, y_center, 40, 10);

        std::cout << "depthValue_center: " << depthValue_center << '\n';
        std::cout << "depthValue_left: " << depthValue_left << '\n';
        std::cout << "depthValue_right: " << depthValue_right << '\n';

        position();
        avoid();

        depth_threshold_center = 10;
        depth_threshold_side = 5;
 
        if(in_air && (depthValue_center < depth_threshold_center || depthValue_left < depth_threshold_side || depthValue_right < depth_threshold_side)){ 
            std::cout << "Obstacle detected! Avoiding.\n";
            state = 0;
            flag_avoid = true;   
        }

        if((state == 4) && (distance_avoid > 0.5)){     //po urazeni urcite vzdalenosti pri vyhybani pokracuje v letu k waypointu
            state = 5;
            flag_avoid = true;
        }
    }

    void FlyMission::position()
    {
        drone_pos = _telemetry->position();
        drone_latitude = drone_pos.latitude_deg;     //aktualni zem. sirka dronu
        drone_longitude = drone_pos.longitude_deg;   //aktualni zem. vyska dronu

        drone_lat_norm = (drone_latitude - 37.4130)*100000;     //souradnice dronu pro vykresleni do grafu
        drone_lon_norm = (drone_longitude + 121.9993)*100000;

        drone_avoid_latitude = drone_pos_avoid.latitude_deg;
        drone_avoid_longitude = drone_pos_avoid.longitude_deg;

        distance_avoid = (std::sqrt(std::pow(drone_latitude-drone_avoid_latitude,2)+std::pow(drone_longitude-drone_avoid_longitude,2)))*10000; 

        std::cout << "distance_avoid: " << distance_avoid << '\n';

        std::cout << "drone_lat norm: " << std::fixed << std::setprecision(3) << drone_lat_norm << '\n';
        std::cout << "drone_lon norm: " << std::fixed << std::setprecision(3) << drone_lon_norm << '\n';

        //mereni uletene vzdalenosti
        static double lat1 = drone_pos.latitude_deg;
        static double lon1 = drone_pos.longitude_deg;

        double lat2 = drone_pos.latitude_deg;
        double lon2 = drone_pos.longitude_deg;

        double d_x = (lat2 - lat1)*(M_PI/180.0);
        double d_y = (lon2 - lon1)*(M_PI/180.0);

        double a = std::pow(std::sin(d_x/2),2) + std::cos(lat1*(M_PI/180.0))*std::cos(lat2*(M_PI/180.0))*std::pow(std::sin(d_y/2),2);
        double b = R*2*std::atan2(std::sqrt(a), std::sqrt(1 - a));

        total_distance += b;

        std::cout << "Distance traveled: " << std::fixed << std::setprecision(3) << total_distance << " meters" << '\n';

        lat1 = lat2;
        lon1 = lon2;
    }

    void FlyMission::avoid()
    {
        if(flag_avoid){

            const mavsdk::Mission::Result pause_mission_result = _mission.get()->pause_mission();   //pozastaveni mise

            if (pause_mission_result != mavsdk::Mission::Result::Success) {
                std::cerr << "Failed to pause mission:" << pause_mission_result << '\n';
            }

            _offboard.get()->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});   //vytvoreni nuloveho setpointu

            mavsdk::Offboard::Result offboard_result = _offboard.get()->start();     //switch do offboard modu

            if(offboard_result != mavsdk::Offboard::Result::Success) {
                std::cerr << "Offboard start failed: " << offboard_result << '\n';
                return;
            }

            switch(state){
                case 0:
                    std::cout << "state: " << state << '\n';
                    drone_pos_avoid = _telemetry->position();   //pocatecni pozice pri uhybani
                    state++;
                
                case 1:
                    std::cout << "state: " << state << '\n';

                    sleep_for(std::chrono::milliseconds(1000));     //chvili pockat, aby se dron plne zastavil
                    avoid_right = depthValue_right > depthValue_left;   //promenna pro rozhodnuti smeru vyhybani
                    state++;

                case 2:
                    std::cout << "state: " << state << '\n';

                    if(avoid_right){
                        std::cout << "Going righthand.\n";
                    }else{
                        std::cout << "Going leftthand.\n";
                    }
                    state++;
                    
                case 3:
                    std::cout << "state: " << state << '\n';

                    if(avoid_right){
                        _offboard.get()->set_velocity_body({0.0f, 0.0f, 0.0f, 45.0f});  //otoceni po smeru hodin, 45 stupnu/s
                        sleep_for(std::chrono::milliseconds(2000));     // -> otoceni o 90 stupnu doprava
                        state++;
                    }else{
                        _offboard.get()->set_velocity_body({0.0f, 0.0f, 0.0f, -45.0f});  //otoceni proti smeru hodin, 45 stupnu/s
                        sleep_for(std::chrono::milliseconds(2000));     // -> otoceni o 90 stupnu doleva
                        state++;
                    }

                case 4:
                    std::cout << "state: " << state << '\n';

                    _offboard.get()->set_velocity_body({3.0f, 0.0f, 0.0f, 0.0f});   //let dopredu
                  
                    flag_avoid = false;
                    distance_avoid = 0;  
                    depthValue_center = 20;
                    depthValue_left = 20;
                    depthValue_right = 20;  
                    break;

                case 5:
                    std::cout << "state: " << state << '\n';

                    _offboard.get()->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});   //zastaveni dronu po uhybnem manevru, jinak dela zvlastni pohyby
                    sleep_for(std::chrono::milliseconds(1000));
                    state++;

                case 6:
                    std::cout << "state: " << state << '\n';

                    mavsdk::Offboard::Result offboard_result2 = _offboard.get()->stop();    //switch z offboard modu
                    if(offboard_result2 != mavsdk::Offboard::Result::Success) {
                        std::cerr << "Offboard stop failed: " << offboard_result2 << '\n';
                        return;
                    }

                    std::cout << "Resuming mission.\n";

                    const mavsdk::Mission::Result start_mission_again_result = _mission.get()->start_mission();     //znovu spusteni mise
                    if (start_mission_again_result != mavsdk::Mission::Result::Success) {
                        std::cerr << "Resuming mission failed: " << start_mission_again_result << '\n';
                    }

                    flag_avoid = false;
                    break;
            }
        }
    }

    void FlyMission::cbStartMission(const std::shared_ptr<std_srvs::srv::Trigger::Request> aRequest, const std::shared_ptr<std_srvs::srv::Trigger::Response> aResponse)
    {
        std::atomic<bool> want_to_pause{false};
        
        start_time_ = std::chrono::steady_clock::now();     //pocatecni cas

        _mission.get()->subscribe_mission_progress([this, &want_to_pause](mavsdk::Mission::MissionProgress mission_progress) {
            std::cout << "Mission status update: " << mission_progress.current << " / "
                    << mission_progress.total << '\n';

            if (mission_progress.current >= 2) {
                want_to_pause = true;
            }

            if (mission_progress.current == mission_progress.total) {
                end_time_ = std::chrono::steady_clock::now();   //koncovy cas

                auto flight_duration = std::chrono::duration_cast<std::chrono::seconds>(end_time_ - start_time_);   //celkovy cas
                std::cout << "Flight duration: " << flight_duration.count() << " seconds\n";
            }
        });

        mavsdk::Mission::Result start_mission_result = _mission.get()->start_mission();     //start mise
        if (start_mission_result != mavsdk::Mission::Result::Success) {
            std::cerr << "Starting mission failed: " << start_mission_result << '\n';
        }

        total_distance = 0.0;
    }

    void FlyMission::upload()
    {
        std::cout << "Creating and uploading mission\n";

        //trasa = 1;
        //trasa = 2;
        trasa = 3;

        if(trasa == 1){
            mission_items.push_back(make_mission_item(
                37.41245,
                -121.9989,
                14.3f,      //20 bez prekazek
                5.0f,
                false,
                -90.0f,
                30.0f,
                mavsdk::Mission::MissionItem::CameraAction::None));

            mission_items.push_back(make_mission_item(
                37.4128,
                -121.9998,
                14.3f,
                5.0f,
                false,
                -90.0f,
                30.0f,
                mavsdk::Mission::MissionItem::CameraAction::None));

            mission_items.push_back(make_mission_item(
                37.4135,
                -121.9993,
                14.3f,
                5.0f,
                false,
                -90.0f,
                30.0f,
                mavsdk::Mission::MissionItem::CameraAction::None));

            mission_items.push_back(make_mission_item(
                37.4129,
                -121.9987,
                14.3f,
                5.0f,
                true,
                -45.0f,
                0.0f,
                mavsdk::Mission::MissionItem::CameraAction::None));
        }

        if(trasa == 2){
            mission_items.push_back(make_mission_item(
                37.4130,
                -121.9984,
                14.6f,
                5.0f,
                false,
                -90.0f,
                30.0f,
                mavsdk::Mission::MissionItem::CameraAction::None));

            mission_items.push_back(make_mission_item(
                37.4129,
                -121.9996,
                14.6f,
                5.0f,
                false,
                -90.0f,
                30.0f,
                mavsdk::Mission::MissionItem::CameraAction::None));

            mission_items.push_back(make_mission_item(
                37.4135,
                -121.99923,
                14.6f,
                5.0f,
                false,
                -90.0f,
                30.0f,
                mavsdk::Mission::MissionItem::CameraAction::None));
        }

        if(trasa == 3){
            mission_items.push_back(make_mission_item(
                37.41335,
                -121.9995,
                16.7f,
                5.0f,
                false,
                -90.0f,
                30.0f,
                mavsdk::Mission::MissionItem::CameraAction::None));
        }

        mavsdk::Mission::MissionPlan mission_plan{};
        mission_plan.mission_items = mission_items;
        const mavsdk::Mission::Result upload_result = _mission.get()->upload_mission(mission_plan);
    }

    void FlyMission::cbUpload(const std::shared_ptr<std_srvs::srv::Trigger::Request> aRequest, const std::shared_ptr<std_srvs::srv::Trigger::Response> aResponse)
    {
        try
        {
            upload();
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';

            aResponse->success = false;
            aResponse->message = "Upload failed...";
        }

        aResponse->success = true;
        aResponse->message = "Uploading...";
    }

    void FlyMission::takeOff()
    {
        const auto arm_result = _action.get()->arm();
        if(arm_result != mavsdk::Action::Result::Success)
        {
            std::cerr << "Arming failed: " << arm_result << '\n';
            throw std::runtime_error("Arming error");
        }

        std::cout << "Armed\n";

        const auto takeoff_result = _action.get()->takeoff();

        if(takeoff_result != mavsdk::Action::Result::Success)
        {
            std::cerr << "Takeoff failed: " << takeoff_result << '\n';
            throw std::runtime_error("Takeoff error");
        }

        auto in_air_promise = std::promise<void>{};
        auto in_air_future = in_air_promise.get_future();

        _telemetry.get()->subscribe_landed_state(
            [this, &in_air_promise](mavsdk::Telemetry::LandedState state) {
                if(state == mavsdk::Telemetry::LandedState::InAir)
                {
                    std::cout << "Taking off has finished.\n";
                    _telemetry.get()->subscribe_landed_state(nullptr);
                    in_air_promise.set_value();
                    in_air = true;
                }
            });

        in_air_future.wait_for(std::chrono::seconds(10));
        if(in_air_future.wait_for(std::chrono::seconds(3)) == std::future_status::timeout)
        {
            std::cerr << "Takeoff timed out.\n";
            throw std::runtime_error("Takeoff timeout error");
        }
    }

    void FlyMission::cbTakeOff(const std::shared_ptr<std_srvs::srv::Trigger::Request> aRequest, const std::shared_ptr<std_srvs::srv::Trigger::Response> aResponse)
    {
        try
        {
            takeOff();
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';

            aResponse->success = false;
            aResponse->message = "Could not take off...";
        }

        aResponse->success = true;
        aResponse->message = "Flying...";
    }

    std::shared_ptr<mavsdk::System> FlyMission::getSystem(mavsdk::Mavsdk& aMavsdk)
    {
        std::cout << "Waiting to discover system...\n";
        auto prom = std::promise<std::shared_ptr<mavsdk::System>>{};
        auto fut = prom.get_future();

        aMavsdk.subscribe_on_new_system([&aMavsdk, &prom]() {
            auto system = aMavsdk.systems().back();

            if(system->has_autopilot())
            {
                std::cout << "Discovered autopilot\n";

                aMavsdk.subscribe_on_new_system(nullptr);
                prom.set_value(system);
            }
        });

        if(fut.wait_for(std::chrono::seconds(5)) == std::future_status::timeout)
        {
            std::cerr << "No autopilot found.\n";
            return nullptr;
        }

        return fut.get();
    }
}

int main(int argc, char *argv[])
{
    std::cout << "Starting MissionFlier node..." << std::endl;

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);

    try
    {
        rclcpp::spin(std::make_shared<mission::FlyMission>());
    }
    catch(const std::runtime_error& e)
    {
        std::cout << "Exception: " << e.what() << std::endl;
    }
    catch(...)
    {

    }

    rclcpp::shutdown();
    
    return 0;
}