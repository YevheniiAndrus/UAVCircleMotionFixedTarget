#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>

int to_gradus(double radians){
    return radians * 180 / M_PI;
}

double to_radians(int gradus){
    return static_cast<double>(gradus) * M_PI / 180;
}

void update_guidance(const double velocity, const double acceleration, const double phi,
            double& delta_x, double& delta_y, double& delta_phi){

    delta_x = velocity * std::cos(phi);
    delta_y = velocity * std::sin(phi);

    delta_phi = acceleration / velocity;
}

double bearing_angle(const double phi, const double theta){
    int phi_g = to_gradus(phi) % 360;
    int theta_g = to_gradus(theta) % 360;

    int n = 180 - (phi_g + theta_g);
    return to_radians(n);
}

void draw_cartesian(cv::Mat img, int width, int height){
    cv::line(img, cv::Point(0, height / 2), cv::Point(width, height / 2), cv::Scalar(0, 0, 0), 2);
    cv::line(img, cv::Point(width / 2, 0), cv::Point(width / 2, height), cv::Scalar(0, 0, 0), 2);
}

cv::Point2d cartesian_to_opencv(const cv::Point2d& point, int width, int height){
    double x = point.x + width / 2;
    double y = height / 2 - point.y;
    return cv::Point2d(x, y);
}

int main(){
    int img_width = 1024;
    int img_height = 768;
    cv::Mat blank = cv::Mat(cv::Size(img_width, img_height), CV_8UC3, cv::Scalar(255, 255, 255));
    draw_cartesian(blank, img_width, img_height);

    // target point
    cv::Point2d cartesian_target(100, 50);
    cv::circle(blank, cartesian_to_opencv(cartesian_target, img_width, img_height), 2, cv::Scalar(0, 0, 255), 2, cv::LineTypes::FILLED);

    // initial point
    cv::Point2d cartesian_uav(-150, 120);
    cv::circle(blank, cartesian_to_opencv(cartesian_uav, img_width, img_height), 2, cv::Scalar(0, 255, 0), 2, cv::LineTypes::FILLED);

    // simulation
    double phi = M_PI/2 - M_PI / 10; // initial azimut
    cv::Point vec(cartesian_uav.x - cartesian_target.x, cartesian_uav.y - cartesian_target.y);
    double target_to_uav_angle = std::atan2(vec.y, vec.x);
    double n_angle = 0.0;
    double velocity = 4.0;
    double r_ref = 30;
    double K = 1.0;

    for(int i = 0; i < 1000; ++i){
        target_to_uav_angle = std::atan2(cartesian_uav.x - cartesian_target.x, cartesian_uav.y - cartesian_target.y);
        n_angle = bearing_angle(phi, target_to_uav_angle);

        double acceleration = velocity * velocity / r_ref + K * std::sin(n_angle);

        double delta_x, delta_y, delta_phi;
        update_guidance(velocity, acceleration, phi, delta_x, delta_y, delta_phi);

        cartesian_uav.x = cartesian_uav.x + delta_x;
        cartesian_uav.y = cartesian_uav.y + delta_y;
        phi = phi - delta_phi;

        cv::circle(blank, cartesian_to_opencv(cartesian_uav, img_width, img_height), 2, cv::Scalar(0, 255, 0), 2, cv::LineTypes::FILLED);

    }

    cv::imshow("UAV", blank);
    cv::waitKey(0);
    return 0;
}