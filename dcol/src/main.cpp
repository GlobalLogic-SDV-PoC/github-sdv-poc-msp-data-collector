#include "dcol/app.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    dcol::App app;
    app.start();
    return 0;
}
