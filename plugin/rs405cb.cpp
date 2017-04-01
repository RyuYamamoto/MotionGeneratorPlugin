#include "rs405cb.h"

bool rs405cb::rs405cb_open()
{
	if(port.open(portname.c_str()))
	{
		cerr << "ERROR:Com port open error" << endl;
		return false;
	}

	port.setBaudRate(baudrate);
	port.setParity(KSerialPort::KS_PARITY_NONE);
	return true;
}

bool rs405cb::rs405cb_close()
{
	port.close();
}

int rs405cb::set_joint_angle(char id, unsigned short time, short angle)
{
	vector<char> sendbuf;
	
	sendbuf.resize(12);
	
	sendbuf[0]  = (unsigned char)0xFA;
	sendbuf[1]  = (unsigned char)0xAF;
	sendbuf[2]  = (unsigned char)id+1;
	sendbuf[3]  = (unsigned char)0x00;
	sendbuf[4]  = (unsigned char)0x1E;
	sendbuf[5]  = (unsigned char)0x04;
	sendbuf[6]  = (unsigned char)0x01;
	sendbuf[7]  = (unsigned char)(angle&0x00FF);
	sendbuf[8]  = (unsigned char)((angle&0xFF00)>>8);
	sendbuf[9]  = (unsigned char)(time&0x00FF);
	sendbuf[10] = (unsigned char)((time&0xFF00)>>8);
	
	unsigned char sum = sendbuf[2];
	for(int i=3;i<11;i++)
		sum = (unsigned char)(sum ^ sendbuf[i]);
	sendbuf[11] = sum;
	
	boost::system::error_code err;
	int ret = port.write_some(sendbuf, err);

	return ret;
}

int rs405cb::get_joint_angle(char id, short *angle)
{
	vector<char> sendbuf;
	vector<char> readbuf;
	
	sendbuf.resize(8);
	readbuf.resize(26);
	
	sendbuf[0]  = (unsigned char)0xFA;
	sendbuf[1]  = (unsigned char)0xAF;
	sendbuf[2]  = (unsigned char)id+1;
	sendbuf[3]  = (unsigned char)0x09;
	sendbuf[4]  = (unsigned char)0x00;
	sendbuf[5]  = (unsigned char)0x00;
	sendbuf[6]  = (unsigned char)0x01;
	
	unsigned char sum = sendbuf[2];
	for(int i=3;i<7;i++)
		sum = (unsigned char)(sum ^ sendbuf[i]);
	sendbuf[7] = sum;

	boost::system::error_code err;
	int ret = port.write_some(sendbuf, err);
	if(ret < 8){
		cerr << "writeError" << endl;
		return -1;
	}

	boost::this_thread::sleep(boost::posix_time::milliseconds(50));

	unsigned long len = port.read_some(readbuf, err);
	if(len < readbuf.size()){
		cerr << "readError" << endl;
		return -2;
	}

	sum = readbuf[2];
	for(int i=3;i<26;i++)
		sum = sum ^ readbuf[i];
		
	if(sum){
		cerr << "sumError" << endl;
		return -3;
	}

	*angle = ((readbuf[8] << 8) & 0x0000FF00) | (readbuf[7] & 0x000000FF);

	return 0;
}

int rs405cb::get_joint_burden(char id, short *burden)
{
	vector<char> sendbuf;
	vector<char> readbuf;
	
	sendbuf.resize(8);
	readbuf.resize(26);
	
	sendbuf[0]  = (unsigned char)0xFA;				
	sendbuf[1]  = (unsigned char)0xAF;				
	sendbuf[2]  = (unsigned char)id+1;	
	sendbuf[3]  = (unsigned char)0x09;				
	sendbuf[4]  = (unsigned char)0x00;				
	sendbuf[5]  = (unsigned char)0x00;				
	sendbuf[6]  = (unsigned char)0x01;
	
	unsigned char sum = sendbuf[2];
	for(int i=3;i<7;i++)
		sum = (unsigned char)(sum ^ sendbuf[i]);
	sendbuf[7] = sum;
	
	boost::system::error_code err;
	int ret = port.write_some(sendbuf, err);
	if(ret < sendbuf.size()){
		cerr << "writeError" << endl;
		return -1;
	}

	boost::this_thread::sleep(boost::posix_time::milliseconds(50));

	unsigned long len = port.read_some(readbuf, err);
	if(len < readbuf.size()){
		cerr << "readError" << endl; 
		return -2;
	}

	sum = readbuf[2];
	for(int i=3;i<26;i++)
		sum = sum ^ readbuf[i];
	if(sum){
		cerr << "sumError" << endl;
		return -3;
	}

	*burden = ((readbuf[12] << 8) & 0x0000FF00) | (readbuf[13] & 0x000000FF);

	return 0;
}

int rs405cb::get_servo_status(char id, ServoStatus *status)
{
	vector<char> sendbuf;
	vector<char> readbuf;

	sendbuf.resize(8);
	readbuf.resize(26);
    
	status->pos = 0;
	status->time_elapsed = 0;
	status->speed = 0;
	status->load = 0;
	status->temperature = 0;
	status->voltage = 0;

	sendbuf[0]  = (unsigned char)0xFA;				
	sendbuf[1]  = (unsigned char)0xAF;				
	sendbuf[2]  = (unsigned char)id+1;	
	sendbuf[3]  = (unsigned char)0x09;				
	sendbuf[4]  = (unsigned char)0x00;				
	sendbuf[5]  = (unsigned char)0x00;				
	sendbuf[6]  = (unsigned char)0x01;
	
	unsigned char sum = sendbuf[2];
	for(int i=3;i<7;i++)
		sum = (unsigned char)(sum ^ sendbuf[i]);
	sendbuf[7] = sum;
	
	boost::system::error_code err;
	int ret = port.write_some(sendbuf, err);
	if(ret < sendbuf.size()){
		cerr << "writeError" << endl;
		return -1;
	}

	boost::this_thread::sleep(boost::posix_time::milliseconds(10));

	unsigned long len = port.read_some(readbuf, err);
	if( len < readbuf.size() ){
		cerr << "readError" << endl;
		return -2;
	}

	sum = readbuf[2];
	for(int i=3;i<26;i++)
		sum = sum ^ readbuf[i];

	if(sum){
		cerr << "sumError" << endl;
		len = port.read_some(readbuf, err); //dispose garbage
		return -3;
	}

	char pos = (((readbuf[8] << 8) & 0x0000FF00) | (readbuf[7] & 0x000000FF));
	status->pos = pos / 10.;
	status->time_elapsed = ((readbuf[10] << 8) & 0x0000FF00) | (readbuf[9] & 0x000000FF);
	status->speed = ((readbuf[12] << 8) & 0x0000FF00) | (readbuf[11] & 0x000000FF);
	status->load = ((readbuf[14] << 8) & 0x0000FF00) | (readbuf[13] & 0x000000FF);
	status->temperature = ((readbuf[16] << 8) & 0x0000FF00) | (readbuf[15] & 0x000000FF);
	status->voltage = (((readbuf[18] << 8) & 0x0000FF00) | (readbuf[17] & 0x000000FF)) / 100.;

	return 0;
}

int rs405cb::enable_torque(char id, short mode)
{
	vector<char> sendbuf;

	sendbuf.resize(9);
	
	sendbuf[0]  = (unsigned char)0xFA;
	sendbuf[1]  = (unsigned char)0xAF;
	sendbuf[2]  = (unsigned char)id+1;
	sendbuf[3]  = (unsigned char)0x00;
	sendbuf[4]  = (unsigned char)0x24;
	sendbuf[5]  = (unsigned char)0x01;
	sendbuf[6]  = (unsigned char)0x01;
	sendbuf[7]  = (unsigned char)(mode&0x00FF);
	
	unsigned char sum = sendbuf[2];
	for(int i=3;i<8;i++)
		sum = (unsigned char)(sum ^ sendbuf[i]);
	sendbuf[8] = sum;
	
	boost::system::error_code err;
	int ret = port.write_some(sendbuf, err);

	return ret;
}

int rs405cb::set_torque(char id, int max_torque)
{
	vector<char> sendbuf;
	
	sendbuf.resize(9);

	sendbuf[0]  = (unsigned char)0xFA;
	sendbuf[1]  = (unsigned char)0xAF;
	sendbuf[2]  = (unsigned char)id+1;
	sendbuf[3]  = (unsigned char)0x00;
	sendbuf[4]  = (unsigned char)0x23;
	sendbuf[5]  = (unsigned char)0x01;
	sendbuf[6]  = (unsigned char)0x01;
	sendbuf[7]  = (unsigned char)max_torque;
	
	unsigned char sum = sendbuf[2];
	for( int i=3;i<8;i++)
		sum = (unsigned char)(sum ^ sendbuf[i]);
	sendbuf[8] = sum;
	
	boost::system::error_code err;
	int ret = port.write_some(sendbuf, err);

	return ret;
}

int rs405cb::set_torque_all(int max_torque)
{
	for(int i=0;i<ACCELITE_SERV_NUM;i++)
		enable_torque(i, max_torque);	
}

int rs405cb::enable_torque_all(short mode)
{
	for(int i=0;i<ACCELITE_SERV_NUM;i++)
		enable_torque(i,mode);	
}

#if 0
int rs405cb::enable_torque_all(short mode)
{
	vector<char> sendbuf;
	
	sendbuf.resize(56);
	
	sendbuf[0]  = (unsigned char)0xFA;
	sendbuf[1]  = (unsigned char)0xAF;
	sendbuf[2]  = (unsigned char)0x00;
	sendbuf[3]  = (unsigned char)0x00;
	sendbuf[4]  = (unsigned char)0x24;
	sendbuf[5]  = (unsigned char)0x02;
	sendbuf[6]  = (unsigned char)0x18;
	for(int servo_no=1;servo_no<30;servo_no++){
		sendbuf[7+(servo_no-1)*2] = (unsigned char)servo_no;
		sendbuf[8+(servo_no-1)*2] = (unsigned char)(mode&0x00FF);
	}
	
	unsigned char sum = sendbuf[2];
	for( int i=3;i<24*2+7; i++ )
		sum = (unsigned char)(sum ^ sendbuf[i]);
	sendbuf[24*2+7] = sum;
	
	boost::system::error_code err;
	int ret = port.write_some(sendbuf, err);

	return ret;
}
#endif

int rs405cb::set_joint_angle_all(unsigned char addr, vector<short> data, short length)
{
	vector<char> sendbuf;
	unsigned char sum;
	int ret = -1;
	int servo_no = 0;

	sendbuf.resize(ACCELITE_SERV_NUM*3+8);

	sendbuf[0]  = (unsigned char)0xFA;
	sendbuf[1]  = (unsigned char)0xAF;
	sendbuf[2]  = (unsigned char)0x00;
	sendbuf[3]  = (unsigned char)0x00;
	sendbuf[4]  = (unsigned char)addr;
	sendbuf[5]  = (unsigned char)(length+1);
	sendbuf[6]  = (unsigned char)ACCELITE_SERV_NUM;
	
	if(length == 1){
		for(int i=0;i<ACCELITE_SERV_NUM;i++){
			sendbuf[servo_no * 2 + 7] = (unsigned char)servo_rs_id[i];
			sendbuf[servo_no * 2 + 8] = (unsigned char)(data[i]&0x00FF);
			servo_no++;
		}
		sum = sendbuf[2];
		for(int i=3;i<ACCELITE_SERV_NUM*2+7;i++)
			sum = (unsigned char)(sum ^ sendbuf[i]);
		sendbuf[ACCELITE_SERV_NUM*2+7] = sum;
		
		boost::system::error_code err;
		ret = port.write_some(sendbuf, err);
	}else if (length == 2){
		for(int i=0;i<ACCELITE_SERV_NUM;i++){
			sendbuf[servo_no * 3 + 7] = (unsigned char)servo_rs_id[i];
			sendbuf[servo_no * 3 + 8] = (unsigned char)(data[i]&0x00FF);
			sendbuf[servo_no * 3 + 9] = (unsigned char)((data[i] >> 8) & 0x00FF);
			servo_no ++;
		}
		sum = sendbuf[2];
		for( int i=3;i<ACCELITE_SERV_NUM*3+7;i++)
			sum = (unsigned char)(sum ^ sendbuf[i]);
		sendbuf[ACCELITE_SERV_NUM*3+7] = sum;
		
		boost::system::error_code err;
		ret = port.write_some(sendbuf, err);
	}

	return ret;
}
