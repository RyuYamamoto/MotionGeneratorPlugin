#ifndef _RS405CB_H_
#define _RS405CB_H_

#include <string>
#include <vector>

//#include <boost/date_time.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp> 
#include "KSerialPort.h"

using namespace std;

const int ACCELITE_SERV_NUM = 19;
static short servo_rs_id[] = {1,2,3,4,5,6,7,8,9,10,11,12,15,16,17,19,20,21,23};

struct ServoStatus
{
	float pos;
	int time_elapsed;
	int speed;
	int load;
	int temperature;
	float voltage;
};

class rs405cb
{
	public:
		rs405cb(const string _portname, const int _baudrate)
			: portname(_portname), baudrate(_baudrate)
		{}
		bool rs405cb_open();
		bool rs405cb_close();
		int set_joint_angle(char id, unsigned short time, short angle);
		int get_joint_angle(char id, short *angle);
		int get_joint_burden(char id, short *burden);
		int get_servo_status(char id, ServoStatus *status);
		int enable_torque(char id, short mode);
		int set_torque(char id, int  max_torque);
		int set_torque_all(int max_torque);
		int enable_torque_all(short mode);
		int set_joint_angle_all(unsigned char addr, vector<short> data, short length);	
	private:
		string portname;
		int baudrate;
		KSerialPort port;
};

#endif
