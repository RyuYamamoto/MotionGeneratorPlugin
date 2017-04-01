#include "MotionGeneratorPlugin.h"

Eigen::Matrix<float,3,3> MotionGeneratorPlugin::computeMatrixFromAngles(float r, float p, float y)
{
	Eigen::Matrix<float,3,3> R;

	R(0,0) = cos(p) * cos(y) - sin(r) * sin(p) * sin(y);
	R(0,1) = -cos(r) * sin(y);
	R(0,2) = sin(r) * cos(y) + sin(r) * cos(p) * sin(y);
	R(1,0) = cos(p) * sin(y) + sin(r) * sin(p) * cos(y);
	R(1,1) = cos(r) * cos(y);
	R(1,2) = sin(p) * sin(y) - sin(r) * cos(p) * cos(y);
	R(2,0) = -cos(r) * sin(p);
	R(2,1) = sin(r);
	R(2,2) = cos(r) * cos(p);	

	return R;
}

void MotionGeneratorPlugin::computeAnglesFromMatrix(Eigen::Matrix<float,3,3> R, float &r, float &p, float &y)
{
	float threshold = 0.001;
	if(abs(R(2,1) - 1.0) < threshold){ // R(2,1) = sin(x) = 1の時
		r = M_PI / 2;
		p = 0;
		y = atan2(R(1,0), R(0,0));
	}else if(abs(R(2,1) + 1.0) < threshold){ // R(2,1) = sin(x) = -1の時t
		r = - M_PI / 2;
		p = 0;
		y = atan2(R(1,0), R(0,0));
	}else{
		r = asin(R(2,1));
		p = atan2(-R(2,0), R(2,2));
		y = atan2(-R(0,1), R(1,1));
	}
}

bool MotionGeneratorPlugin::initialize()
{
	ToolBar* bar = new ToolBar("MotionGenerator");
	for(int i=0;i<6;i++) footSpin[i] = new DoubleSpinBox();

	bar->addButton("Get Current Jont State")
		->sigClicked().connect(bind(&MotionGeneratorPlugin::getCurrentJointState, this ));
	bar->addButton("TorqueON")
		->sigClicked().connect(bind(&MotionGeneratorPlugin::torqueON, this));
	bar->addButton("TorqueOFF")
		->sigClicked().connect(bind(&MotionGeneratorPlugin::torqueOFF, this));

	for(int i=0;i<6;i++){
		footSpin[i]->setAlignment(Qt::AlignCenter);
		footSpin[i]->setDecimals(4);
		footSpin[i]->setSingleStep(step[i]);
		footSpin[i]->setRange(limit_min[i], limit_max[i]);
	}

	for(int i=0;i<3;i++) footSpin[i]->sigValueChanged().connect(bind(&MotionGeneratorPlugin::set_target_pos, this));
	for(int i=3;i<6;i++) footSpin[i]->sigValueChanged().connect(bind(&MotionGeneratorPlugin::set_target_rot, this));
	for(int i=0;i<6;i++) bar->addWidget(footSpin[i]);

	addToolBar(bar);
	servoMotor->rs405cb_open();
	servoMotor->set_torque_all(64);

	return true;
}

void MotionGeneratorPlugin::torqueON()
{
	servoMotor->enable_torque_all(1);
}

void MotionGeneratorPlugin::torqueOFF()
{
	servoMotor->enable_torque_all(0);
}

void MotionGeneratorPlugin::getCurrentJointState()
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
	RFLink_org = RFLink = ulink[Const::RR2];
	LFLink_org = LFLink = ulink[Const::LR2];
}

void MotionGeneratorPlugin::set_target_pos()
{
	RFLink.p <<  RFLink_org.p(0) - footSpin[0]->value(), RFLink_org.p(1) + footSpin[1]->value(), RFLink.p(2) = RFLink_org.p(2) + footSpin[2]->value();;
	LFLink.p <<  LFLink_org.p(0) + footSpin[0]->value(), LFLink_org.p(1) + footSpin[1]->value(), LFLink.p(2) = LFLink_org.p(2) + footSpin[2]->value();;

	calcInverseKinematics();
}

void MotionGeneratorPlugin::set_target_rot()
{
	RFLink.R = computeMatrixFromAngles(deg2rad(footSpin[3]->value()), -1*deg2rad(footSpin[4]->value()), deg2rad(footSpin[5]->value()));
	LFLink.R = computeMatrixFromAngles(deg2rad(footSpin[3]->value()), deg2rad(footSpin[4]->value()), deg2rad(footSpin[5]->value()));

	calcInverseKinematics();
}

void MotionGeneratorPlugin::calcInverseKinematics()
{
	ItemList<BodyItem> bodyItems = ItemTreeView::mainInstance()->selectedItems<BodyItem>();
	BodyPtr body = bodyItems[0]->body();

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

	servoMotor->set_joint_angle(0,10,rad2deg(servo_angle[0])*100);
	servoMotor->set_joint_angle(1,10,rad2deg(servo_angle[1])*100);
}

