#include"rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/Path.hpp"
#include "geometry_msgs/Twist.hpp"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/utils.h>
#include <geometry/msg/quaternion.hpp>
#include <iostream>
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <vector>
#include <mutex>
using namespace std::chrono_literals;



class jiepath_facmd : public rclcpp::Node
{
    public:
        //初始化函数
        jiepath_facmd() : Node ("jie_path_fa_cmd")
        {
            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tf_listener_= std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            subscribe_path = this->create_subscription<nav_msgs::msg::Path>(
                "/path",10,std::bind(&jiepath_facmd::path_callback,this,_1));
            subscribe_costmap = this->create_subscription<nav_msgs::msg::occupancy_grid>(
                "/occupancy_grid/costmap",10,std::bind(&jiepath_facmd::path_callback,this,_1));
            
            publisher_ = this->create_publisher<geometry_msgs::Twist>("/cmd_vel",10);
            timer_ = this->create_wall_timer(
                500ms,std::bind(&jiepath_facmd::timer_callback,this)
            );
             
        }
    private:
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subscribe_path;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        nav_msgs::msg::Path current_path;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_; 
        struct path_pose()
        {
            double x;
            double y;
            double yaw;
        }
         void timer_callback()
         {
            auto fa_cmd_vel = geometry_msgs::msg::Twist();
            //计算出速度发布出去
            if(current_path.poses.empty())
            {
                fa_cmd_vel.linear.x =0;
                fa_cmd_vel.linear.y = 0;
                fa_cmd_vel.linear.z = 0;
               
                RCLCPP_WARN(this->get_logger(),"未收到路径，发布0速度");
                
            }
            else
            {
                //正常计算速度发布逻辑

            }
            publisher_->publish(fa_cmd_vel);
         }
         void path_callback(const nav_msgs::msg::Path::SharedPtr path_msg)//传参数进去
         {
            if(path_mag->poses is emptry)
            {
                return 0;
            }
            std::vector<path_pose> internal_path;//这个数组是动态的

            //
            try
            {
            //查询变换map2baselink
            auto transform_stamped = tf_buffer_->lookup_transform("base_link","map",tf2::TimePointZero,100ms); 
            int length_path = length[path_msg];
            for(int i=0;i<=length_path;i++)
            {
                auto pose=path_msgs->poses;
                geometry_msgs::msg::PointStamped point_in_base;
                tf2::doTransform(pose,point_in_base,transform_stamped);
                //提取我们需要的x和y还有角度
                path_pose p;
                p.x = point_in_base.pose.position.x;
                p.y = point_in_base.pose.position.y;
                p.yaw = tf2::getYaw(point_in_base.pose.position.orientation);
                internal_path.push_back(p);

            }
            

            }
            catch(const tf2::TransformException & ex)
            {
                RCLCPP_ERROR(this->get_logger(),"无法获取变换 %s,ex.what()");


            }
                   
         }   
        
};

// void map2base_link_path(const )
// {
//     tf2_ros::Buffer tfBuffer;
//     tf2_ros::TransformListener tfListener(tfBuffer);

// };



int main(int argc,char * argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<jiepath_facmd>());
    rclcpp::shutdown();
    return 0;

}