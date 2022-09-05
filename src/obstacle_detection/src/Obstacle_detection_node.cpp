
#include "Obstacle_detection_core.h"
#include "fusion.h"
#include <thread>


int socketfd;
int ok;
struct order order;





int main(int argc, char **argv)
{
    std::thread t1(intial_socket,std::ref(socketfd));


    std::thread  t2(recvMessage_socket,order,std::ref(ok));

    ros::init(argc, argv, "obstacle_detection");

    ros::NodeHandle nh;

    Obstacledetection core(nh);
    ros::AsyncSpinner spinner(2); // Use 2 threads
    spinner.start();
    ros::waitForShutdown();
    
    t1.join();
    t2.join();

    return 0;
}