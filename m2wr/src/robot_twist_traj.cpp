#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
int main(int argc, char **argv)
{
// Инициализация ноды с определённым именем для отладки
ros::init(argc, argv, "robot_twist_traj");
ros::NodeHandle nh;
ros::Rate loop_rate(5);
// Инициализация издателя в определённый топик cmd_vel
ros::Publisher traj_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
// Задаём линей ную и угловую скорость
geometry_msgs::Twist vel;
// vel.linear.x = 1.0;
// vel.angular.z = 0.5;
// В цикле публикуем текущее значение скорости
while (ros::ok())
{
vel.linear.x =1.0;
vel.linear.z = 0;
traj_pub.publish(vel);
ros::spinOnce();
loop_rate.sleep();
vel.linear.x = 0;
vel.angular.z = 1.0;
traj_pub.publish(vel);
ros::spinOnce();
loop_rate.sleep();
}
}