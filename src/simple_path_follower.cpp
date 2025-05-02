#include <unistd.h>
#include <cmath>
#include <vector>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"

#include "common/ros2_sport_client.h" // Unitree Go2 SDK 제공

class Go2PathFollower : public rclcpp::Node
{
public:
    Go2PathFollower() : Node("go2_path_follower"), current_target_idx_(0), t_(-1.0)
    {
        // 현재 위치 구하기
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&Go2PathFollower::odom_callback, this, std::placeholders::_1));

        // 상태 받기 (초기 위치 잡기)
        state_sub_ = this->create_subscription<unitree_go::msg::SportModeState>(
            "sportmodestate", 10, std::bind(&Go2PathFollower::state_callback, this, std::placeholders::_1));

        // 요청 퍼블리셔
        req_pub_ = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);

        // 타이머: 경로 생성 & 요청 전송
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(dt_ * 1000)),
            std::bind(&Go2PathFollower::timer_callback, this));

        // 경로 설정 단위 m
        path_points_ = {
            {1.0, 0.0},
            {2.0, 2.0},
            {2.0, 4.0},
            {4.0, 4.0}};

        RCLCPP_INFO(this->get_logger(), "Go2PathFollower 노드 시작됨");
    }

private:
    struct Point
    {
        double x;
        double y;
    };

    void state_callback(const unitree_go::msg::SportModeState::SharedPtr msg)
    /**
     * @brief 초기 위치를 설정한다.
     * @param msg 현재로봇 좌표
     */
    {
        if (t_ < 0.0)
        {
            px0_ = msg->position[0];
            py0_ = msg->position[1];
            yaw0_ = msg->imu_state.rpy[2];
            RCLCPP_INFO(this->get_logger(), "초기 위치 설정됨: (%.2f, %.2f, %.2f)", px0_, py0_, yaw0_);
            //! 0 is idle
            //! 1 is trot
            //! 2 is trot running
            //! 3 is forward climbing mode
            //! 4 is reverse climbing mode
            sport_req_.SwitchGait(req_, 3); // 로봇 걸음걸이 바꾸기
            req_pub_->publish(req_);
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    /**
     * @brief odometry 좌표를 받아온다.
     * @param msg odometry 좌표
     */
    {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;

        auto q = msg->pose.pose.orientation;
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        current_yaw_ = std::atan2(siny_cosp, cosy_cosp);
    }

    void timer_callback()
    {
        t_ += dt_;
        if (t_ < 0.0)
            return;

        if (current_target_idx_ >= path_points_.size())
        {
            RCLCPP_INFO(this->get_logger(), "모든 목표 도달 완료. 요청 중단.");
            return;
        }

        // 경로 생성
        std::vector<PathPoint> path;
        double time_seg = 0.2; // 점과 점 사이를 이동하는데 걸리는 시간
        double time_temp = t_ - time_seg;

        for (size_t i = current_target_idx_; i < path_points_.size(); ++i)
        {
            PathPoint path_point_tmp;

            path_point_tmp.timeFromStart = (i - current_target_idx_) * time_seg;

            std::vector<Point> temppoints = generateWaypoints(path_points_[i],path_points_[i+1]);
            for(size_t j = 0; j < temppoints.size(); j++)
            {


                double target_x = temppoints[j].x;
                double target_y = temppoints[j].y;

                        // 현재 좌표계에 맞춰 변환
                double dx = target_x - px0_;
                double dy = target_y - py0_;

                dx = 0.5*0.5;
                dy = 0;

                path_point_tmp.x = dx * std::cos(yaw0_) - dy * std::sin(yaw0_) + px0_;
                path_point_tmp.y = dx * std::sin(yaw0_) + dy * std::cos(yaw0_) + py0_;
                path_point_tmp.yaw = yaw0_; // 기본 yaw 유지
                path_point_tmp.vx = 0.5;    // x 방향 속도 0.5 m/s (임의)
                path_point_tmp.vy = 0.0;    // y 방향 속도 0
                path_point_tmp.vyaw = 0.0;  // 회전 속도 0

                path.push_back(path_point_tmp);






                // double target_x = path_points_[i].first;
                // double target_y = path_points_[i].second;

                // // 현재 좌표계에 맞춰 변환
                // double dx = target_x - px0_;
                // double dy = target_y - py0_;

                // path_point_tmp.x = dx * std::cos(yaw0_) - dy * std::sin(yaw0_) + px0_;
                // path_point_tmp.y = dx * std::sin(yaw0_) + dy * std::cos(yaw0_) + py0_;
                // path_point_tmp.yaw = yaw0_; // 기본 yaw 유지
                // path_point_tmp.vx = 0.5;    // x 방향 속도 0.5 m/s (임의)
                // path_point_tmp.vy = 0.0;    // y 방향 속도 0
                // path_point_tmp.vyaw = 0.0;  // 회전 속도 0

                // path.push_back(path_point_tmp);

            }


        }

        sport_req_.TrajectoryFollow(req_, path);
        req_pub_->publish(req_);
        RCLCPP_INFO(this->get_logger(), "Go2 경로 요청 전송 중...");
    }

    std::vector<Point> generateWaypoints(Point start, Point end, double step = 0.15)
    {
        std::vector<Point> waypoints;

        double dx = end.x - start.x;
        double dy = end.y - start.y;
        double distance = std::sqrt(dx * dx + dy * dy);

        int numSteps = std::floor(distance / step);

        for (int i = 0; i <= numSteps; ++i)
        {
            double t = static_cast<double>(i) / numSteps;
            Point wp;
            wp.x = start.x + t * dx;
            wp.y = start.y + t * dy;
            waypoints.push_back(wp);
        }

        return waypoints;
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr state_sub_;
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    SportClient sport_req_;
    unitree_api::msg::Request req_;

    //std::vector<std::pair<double, double>> path_points_;
    std::vector<Point> path_points_;
    size_t current_target_idx_;

    double current_x_ = 0.0, current_y_ = 0.0, current_yaw_ = 0.0;
    double px0_ = 0.0, py0_ = 0.0, yaw0_ = 0.0;
    double t_;
    const double dt_ = 0.05;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Go2PathFollower>());
    rclcpp::shutdown();
    return 0;
}
