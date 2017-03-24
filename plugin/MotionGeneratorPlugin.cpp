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

//実機との符号調整
const int sign[] = {-1,1,-1,-1,1,1,1,-1,1,1,1,1, 1,1,1,1,1,1,1,1,1,1,1,1,1}; 

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
		for(int i=0;i<12;i++)
			servo_angle[i] = body->joint(i)->q() * sign[i];
		for(int i=14;i<18;i++)
			servo_angle[i] = body->joint(i)->q() * sign[i];

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

		RFLink.p(1) += dq; 
        if(kine->calcInverseKinematics(servo_angle, RFLink, LFLink)){
			for(int i=0;i<12;i++)
				body->joint(i)->q() = servo_angle[i] * sign[i];
			
			body->joint(12)->q() = body->joint(2)->q() * (-1);
			body->joint(13)->q() = body->joint(2)->q();
			body->joint(18)->q() = body->joint(1)->q();
			body->joint(19)->q() = body->joint(1)->q() * (-1);
			body->joint(20)->q() = body->joint(8)->q();
			body->joint(21)->q() = body->joint(8)->q() * (-1);
			body->joint(23)->q() = body->joint(7)->q();
			body->joint(24)->q() = body->joint(7)->q();
		}
		bodyItems[0]->notifyKinematicStateChange(true);
    }
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(MotionGeneratorPlugin)

#endif
