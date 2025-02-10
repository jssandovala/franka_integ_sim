#include "ros/ros.h"
#include <string>
#include <cmath>
#include <stdlib.h>
#include <Eigen/Dense>
#include <iostream>
#include <sstream>


std::vector<double> StringToVect(std::string ss){
    std::stringstream is( ss );
    std::vector<double> vec( ( std::istream_iterator<double>( is ) ), ( std::istream_iterator<double>() ) );
    return vec;
}


Eigen::Matrix4d vq_To_matrix(Eigen::Vector3d v,Eigen::Quaterniond q){
  Eigen::Matrix4d T;
  T.setIdentity();
  Eigen::Matrix3d R = q.normalized().toRotationMatrix();
  T.block<3,3>(0,0) = R;
  T.block<3,1>(0,3) = v;
  return T;  
}

Eigen::Matrix4d tr_xyz_rpy(
    const double x,
    const double y,
    const double z,
    const double roll,
    const double pitch,
    const double yaw ) {
  Eigen::Vector3d xx;
  xx << x,y,z;
  Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
  Eigen::Matrix4d Trans;
  Trans=vq_To_matrix(xx,q);
  return Trans;  
}


int main(int argc, char **argv)
{
    ros::init(argc, argv,"paramset");
    ros::NodeHandle nh;
    std::string x_y_z, r_p_y;
    std::vector<double> xyz, rpy;

    if(nh.getParam("panda_tool_xyz", x_y_z)){
        xyz=StringToVect(x_y_z);
    }
    else{xyz={0, 0, 0};}             //default_value


    if(nh.getParam("panda_tool_rpy", r_p_y)){
        rpy=StringToVect(r_p_y);
    }
    else{rpy={0, 0, 0};}             //default_value

    Eigen::Matrix4d F_TCP, F_T, T_TCP;
    std::vector<double> FT;
    // optimisation
    T_TCP << 1,0,0,0,0,1,0,0,0,0,1,0.3,0,0,0,1;
    F_T = tr_xyz_rpy(xyz[0],xyz[1],xyz[2],rpy[0],rpy[1],rpy[2]);
    F_TCP = F_T * T_TCP;

    
    // flange to TCP
    //F_TCP=tr_xyz_rpy(xyz[0],xyz[1],xyz[2],rpy[0],rpy[1],rpy[2]);

    FT={F_TCP(0,0),F_TCP(1,0),F_TCP(2,0),F_TCP(3,0),F_TCP(0,1),F_TCP(1,1),F_TCP(2,1),F_TCP(3,1),F_TCP(0,2),F_TCP(1,2),F_TCP(2,2),F_TCP(3,2),F_TCP(0,3),F_TCP(1,3),F_TCP(2,3),F_TCP(3,3)};

    std::string T;
    for (size_t i = 0; i < FT.size(); i++) {
		if (i != 0) {
			T.append(" ");
		}
		T.append(std::to_string(FT[i])) ;
	}
    
    nh.setParam("F_T_NE", T);

    return 0;
}

    