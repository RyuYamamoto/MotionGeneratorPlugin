#ifndef _MOTIONGENERATORPLUGIN_H_
#define _MOTIONGENERATORPLUGIN_H_

#include <cnoid/Plugin>
#include <cnoid/ItemTreeView>
#include <cnoid/BodyItem>
#include <cnoid/ToolBar>

#include <iostream>
#include <math.h>

#include "Kinematics.h"
#include "Link.h"
#include "Constant.h"

using namespace std;
using namespace cnoid;

double deg2rad(double degree)
{
	return degree * M_PI / 180.0;
}

double rad2deg(double radian)
{
	return radian * 180.0 / M_PI;
}

//実機との符号調整
const int sign[] = {-1,1,-1,-1,1,1,-1,-1,1,1,1,1, 1,1,1,1,1,1,1,1,1,1,1,1,1}; 

class MotionGeneratorPlugin : public Plugin
{
private:
	Kinematics *kine;
	cit::Link ulink[Const::LINK_NUM];
	cit::Link RFLink, LFLink;
	float servo_angle[Const::LINK_NUM];
public:
    
    MotionGeneratorPlugin() : Plugin("MotionGenerator")
    {
        require("Body");

		cit::initLink(ulink);
		kine = new Kinematics(ulink);

		for(int i=0;i<Const::LINK_NUM;i++)
			servo_angle[i] = 0.0f;
    }
    
    virtual bool initialize()
    {
        ToolBar* bar = new ToolBar("MotionGenerator");
        bar->addButton("x++")
				->sigClicked().connect(bind(&MotionGeneratorPlugin::onButtonClicked, this, +0.005));
        bar->addButton("x--")
                ->sigClicked().connect(bind(&MotionGeneratorPlugin::onButtonClicked, this, -0.005));
        addToolBar(bar);
		bar->addButton("Get Current Jont State")
				->sigClicked().connect(bind(&MotionGeneratorPlugin::getCurrentJointState, this, 0.01));

        return true;
    }

	void getCurrentJointState(double dq)
	{
		ItemList<BodyItem> bodyItems = ItemTreeView::mainInstance()->selectedItems<BodyItem>();
		BodyPtr body = bodyItems[0]->body();
		cout << body->joint(0)->q() << endl;
		// 左脚関節
		servo_angle[0]	= body->joint(0)->q() * sign[0];
		servo_angle[1]	= body->joint(1)->q() * sign[1];
		servo_angle[2]  = body->joint(2)->q() * sign[2];
		servo_angle[3]	= body->joint(3)->q() * sign[3];
		servo_angle[4]	= body->joint(4)->q() * sign[4];
		servo_angle[5]	= body->joint(5)->q() * sign[5];
		// 右脚関節
		servo_angle[6]	= body->joint(6)->q() * sign[6];
		servo_angle[7]	= body->joint(7)->q() * sign[7];
		servo_angle[8]	= body->joint(8)->q() * sign[8];
		servo_angle[9]	= body->joint(9)->q() * sign[9];
		servo_angle[10] = body->joint(10)->q() * sign[10];
		servo_angle[11] = body->joint(11)->q() * sign[11];
		// 左腕
		servo_angle[14] = body->joint(14)->q() * sign[14];
		servo_angle[15] = body->joint(15)->q() * sign[15];
		// 右腕
		servo_angle[16] = body->joint(16)->q() * sign[16];
		servo_angle[17] = body->joint(17)->q() * sign[17];

		//順運動学計算
		kine->setJointAngle(servo_angle);
		kine->calcForwardKinematics();

		//足先位置・姿勢更新
		RFLink = ulink[Const::RR2];
		LFLink = ulink[Const::LR2];
	}

    void onButtonClicked(double dq)
    {
        ItemList<BodyItem> bodyItems = ItemTreeView::mainInstance()->selectedItems<BodyItem>();
		BodyPtr body = bodyItems[0]->body();

		RFLink.p(0) += dq; 
        if(kine->calcInverseKinematics(servo_angle, RFLink, LFLink)){
			// 左脚
			body->joint(0)->q() = servo_angle[0] * sign[0];
			body->joint(1)->q() = servo_angle[1] * sign[1]; body->joint(18)->q() = body->joint(1)->q(); body->joint(19)->q() = body->joint(1)->q() * (-1);
			body->joint(2)->q() = servo_angle[2] * sign[2]; body->joint(12)->q() = body->joint(2)->q() * (-1); body->joint(13)->q() = body->joint(2)->q();
			body->joint(3)->q() = servo_angle[3] * sign[3];
			body->joint(4)->q() = servo_angle[4] * sign[4];
			body->joint(5)->q() = servo_angle[5] * sign[5];
			// 右脚
			body->joint(6)->q() = servo_angle[6] * sign[6];
			body->joint(7)->q() = servo_angle[7] * sign[7]; body->joint(23)->q() = body->joint(7)->q(); body->joint(24)->q() = body->joint(7)->q();
			body->joint(8)->q() = servo_angle[8] * sign[8]; body->joint(20)->q() = body->joint(8)->q(); body->joint(21)->q() = body->joint(8)->q() * (-1);
			body->joint(9)->q() = servo_angle[9] * sign[9]; 
			body->joint(10)->q() = servo_angle[10] * sign[10];
			body->joint(11)->q() = servo_angle[11] * sign[11];
		}
		bodyItems[0]->notifyKinematicStateChange(true);
    }
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(MotionGeneratorPlugin)

#endif
