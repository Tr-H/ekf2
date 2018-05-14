/*************************************************************************
	> File Name: fuse_ekf.cpp
	> Author: 
	> Mail: 
	> Created Time: 2018年04月16日 星期一 15时49分34秒
 ************************************************************************/

#include<fuse_ekf_test/Node.hpp>

int main(int argc, char **argv) {
    ros::init(argc, argv, "fuse_ekf_test");
    ros::NodeHandle nh;
    //ros::NodeHandle pnh("~");
    fuse_ekf_test::Node node(nh);
    //ros::Subscriber subVisual_ = nh.subscribe("visual_odom", 1, &fuse_ekf_test::Node::visual_odom_cb, &node); 
    ros::spin();
    return 0;
}
