#ifndef _MOTIONGENERATORPLUGIN_H_
#define _MOTIONGENERATORPLUGIN_H_

#include <cnoid/Plugin>
#include <cnoid/ItemTreeView>
#include <cnoid/BodyItem>
#include <cnoid/ToolBar>

#include "../../src/Base/SpinBox.h"

#include <iostream>
#include <math.h>

#include "Kinematics.h"
#include "Link.h"
#include "Constant.h"

using namespace std;
using namespace cnoid;

//実機との符号調整
const int sign[] = {1,1,1,1,1,1,1,-1,1,-1,1,1, 1,1,1,1,1,1,1,1,1,1,1,1,1}; 
const double step[] = {0.005,0.005,0.005,0.5,0.5,0.5};
const double limit_max[] = {10.0,10.0,10.0,50.0,50.0,50.0};
const double limit_min[] = {-10.0,-10.0,-10.0,-50.0,-50.0,-50.0};

float deg2rad(float degree){
	return degree * M_PI / 180.0f;
}

float rad2deg(float radian){
	return radian * 180.0f / M_PI;
}

class MotionGeneratorPlugin : public Plugin
{
	private:
		Kinematics *kine;
		cit::Link ulink[Const::LINK_NUM];
		cit::Link RFLink, LFLink;
		cit::Link RFLink_org, LFLink_org;
		float servo_angle[Const::LINK_NUM];
		DoubleSpinBox *footSpin[6];
	public:

		MotionGeneratorPlugin() : Plugin("MotionGenerator")
	{
		require("Body");

		cit::initLink(ulink);
		kine = new Kinematics(ulink);

		for(int i=0;i<Const::LINK_NUM;i++)
			servo_angle[i] = 0.0f;
	}

		virtual bool initialize();
		void getCurrentJointState();
		void set_target_pos();
		void set_target_rot();
		void calcInverseKinematics();
		Eigen::Matrix<float,3,3> computeMatrixFromAngles(float r, float p, float y);
		void computeAnglesFromMatrix(Eigen::Matrix<float,3,3> R, float &r, float &p, float &y);
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(MotionGeneratorPlugin)

#endif
