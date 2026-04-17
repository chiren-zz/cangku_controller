#include"rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/Path.hpp"
#include "geometry_msgs/Twist.hpp"


class jiepath_facmd : public rclcpp::Node
{
    public:
        jiepath_facmd() : Node ("jie_path_fa_cmd")
        {
            subscribe = this->create_subscription<nav_msgs::msg::path>(
                "/path",10,std::bind(&jiepath_facmd::topic_callback,this,_1));

            publisher_ = this->creater_publisher<geometry_msgs::Twist>("/cmd_vel",10);
            timer_ = this->create_wall_timer(
                500ms,std::bind(&jiepath_facmd::timer_callback,this);
            )
             
        }
    private:
        rclcpp::subscripton::shareptr subscribe;
        rclcpp::publisher::shareptr publisher_
        rclcpp::Timerbase::Shareptr timer_;
         void timer_back()
         {
            //计算出速度发布出去
         }
         void topic_back()
         {
            //
         }
    
        
        
        
};

int main(int argc,char * argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<jiepath_facmd>());
    rclcpp::shutdown();
    return 0;

}