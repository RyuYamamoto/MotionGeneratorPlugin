#ifndef _MOTIONGENERATORPLUGIN_H_
#define _MOTIONGENERATORPLUGIN_H_

#include <cnoid/Plugin>
#include <cnoid/ItemTreeView>
#include <cnoid/BodyItem>
#include <cnoid/ToolBar>

#include <iostream>

#include "Kinematics.h"
#include "Link.h"
#include "Constant.h"

using namespace std;
using namespace cnoid;

//実機との符号調整
const int sign[] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}; 

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
        bar->addButton("Increment")
				->sigClicked().connect(bind(&MotionGeneratorPlugin::onButtonClicked, this, +0.04));
        bar->addButton("Decrement")
                ->sigClicked().connect(bind(&MotionGeneratorPlugin::onButtonClicked, this, -0.04));
        addToolBar(bar);

        return true;
    }
#if 0
	void getCurrentJointState()
	{
		ItemList<BodyItem> bodyItems = ItemTreeView::mainInstance()->selectedItems<BodyItem>();

		BodyPtr body = bodyItems[i]->body();
		for(int j=0;i<body->numJoints();i++)
			servo_angle[i]
	}
#endif

    void onButtonClicked(double dq)
    {
        ItemList<BodyItem> bodyItems = ItemTreeView::mainInstance()->selectedItems<BodyItem>();
        //cout << body->numJoints() << endl;
        for(size_t i=0; i < bodyItems.size(); ++i){
			BodyPtr body = bodyItems[i]->body();
			cout << body->numJoints() << endl;
			for(int j=0; j < body->numJoints(); ++j){
                cout << body->joint(j)->name() << endl;
				body->joint(j)->q() += dq;
			}
            bodyItems[i]->notifyKinematicStateChange(true);
        }
    }
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(MotionGeneratorPlugin)

#endif
