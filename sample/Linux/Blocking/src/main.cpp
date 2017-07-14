/*
 * UDP传输
 */
//System Headers
#include <stdio.h>  
#include <iostream>
#include <stdlib.h>  
#include <string>
#include <cstring>
#include <unistd.h>
#include <errno.h>  
#include <sys/types.h>  
#include <sys/socket.h>  
#include <netinet/in.h>  
#include <csignal>
#include <stack>
#include <pthread.h>
#include <fstream>

//DJI Linux Application Headers
#include "LinuxSerialDevice.h"
#include "LinuxThread.h"
#include "LinuxSetup.h"
#include "LinuxCleanup.h"
#include "ReadUserConfig.h"
#include "LinuxMobile.h"
#include "LinuxFlight.h"
#include "LinuxInteractive.h"
#include "LinuxWaypoint.h"
#include "LinuxCamera.h"

//DJI OSDK Library Headers
#include "DJI_Follow.h"
#include "DJI_Flight.h"
#include "DJI_Version.h"
#include "DJI_WayPoint.h"

using namespace std;
using namespace DJI;
using namespace DJI::onboardSDK;

enum CONTROL
{
	TAKEOFF = 1,
	MONITORED_TAKEOFF,
	ATTITUDE_CONTROL,
	ATTITUDE_ALTITUDE_CONTROL,
	MOVE_BY_OFFSET,
	MOVE_WITH_VELOCITY,
	GO_HOME,
	LAND
};
struct COMMAND
{
	CONTROL control; //向无人机发送的哪种命令
	int argc; //参数的个数
	float argv[5]; //命令的参数
};

void initSocket(int &);
int RecvData(int sock, void *buf, int size);
//! Main function for the Linux sample. Lightweight. Users can call their own API calls inside the Programmatic Mode else on Line 68. 

LinuxSerialDevice* serialDevice; 
CoreAPI* api; 
Flight* flight; 
LinuxThread* read1;
int blockingTimeout = 1; //Seconds
int socket_fd;
void sigroutine(int dunno)
{
	ackReturnData landingStatus = landing(api, flight,blockingTimeout);
	int cleanupStatus = cleanup(serialDevice, api, flight, read1);
	if (cleanupStatus == -1)
	{
		std::cout << "Unable to cleanly destroy OSDK infrastructure. There may be residual objects in the system memory.\n";
	}
	std::cout << "Program exited successfully." << std::endl;
	
	//close socket
	close(socket_fd);  
	exit(1);
}

pthread_mutex_t mutex;//控制stackCommand的互斥锁
std::stack<COMMAND> stackCommand;//用来存储command的stack，当作栈来用
void *RecvCommand(void *argv);

ofstream outfile("out.txt",ios::out);
int main()
{
	signal(SIGTSTP,sigroutine);	
	signal(SIGINT,sigroutine);

	serialDevice = new LinuxSerialDevice(UserConfig::deviceName, UserConfig::baudRate);
	api = new CoreAPI(serialDevice);
	flight = new Flight(api);
	read1 = new LinuxThread(api, 2);

	//---------------连接飞控-------------------//
	int setupStatus = setup(serialDevice, api, read1);
	if (setupStatus == -1)
	{
		std::cout << "This program will exit now. \n";
		return 0;
	}
	unsigned short broadcastAck = api->setBroadcastFreqDefaults(1);
	usleep(500000);

	//---------------连接socket------------------//
	initSocket(socket_fd);
	std::cout<<"wait for command！"<<std::endl;
	//创建并执行子线程
	int res;
	pthread_t a_thread;
	void *thread_result;
	res = pthread_create(&a_thread,NULL,RecvCommand,NULL);
	if(res!=0)
	{
		perror("Thread creation failed!");	
		exit(EXIT_FAILURE);
	}

	COMMAND command;	
	ackReturnData ackStatus;
	int status;
	time_t start,end;
	int count = 0;
	while(1)
	{
		usleep(50000);
		pthread_mutex_lock(&mutex);
		if(stackCommand.empty())
		{
			pthread_mutex_unlock(&mutex);
			continue;
		}
		command = stackCommand.top();
		while(!stackCommand.empty())
		{
			stackCommand.pop();
		}
		pthread_mutex_unlock(&mutex);

		if(command.control == ATTITUDE_CONTROL ||command.control == ATTITUDE_ALTITUDE_CONTROL ||\
				command.control == MOVE_BY_OFFSET||command.control == MOVE_WITH_VELOCITY)
		{
			if(flight->getStatus() == 1)//1代表无人机在地面未起飞，参考flight->getStatus()的返回值
				ackStatus = monitoredTakeoff(api, flight, blockingTimeout);
		}
		double yaw = flight->getYaw();
		count++;
		end = clock();
		double t = (double)(end-start);
		outfile<<"------------"<<count<<": "<<t/CLOCKS_PER_SEC*1000<<std::endl;
		start = end;
		switch(command.control)
		{
			case TAKEOFF:
				//ackReturnData takeoff(Flight* flight, int timeout = 1);
				break;

			case MONITORED_TAKEOFF:
				std::cout<<"monitored takeoff!"<<std::endl;
				ackStatus = monitoredTakeoff(api, flight, blockingTimeout);
				break;

			case ATTITUDE_CONTROL:
				std::cout<<"attitude control!"<<std::endl;
				status = attitudeControl(api, flight, command.argv[0], command.argv[1], command.argv[2], command.argv[4]); //Holds current height
				break;

			case ATTITUDE_ALTITUDE_CONTROL:
				std::cout<<"attitude altitude control!"<<std::endl;
				status = attitudeAltitudeControl(api, flight, command.argv[0], command.argv[1], command.argv[2], command.argv[3],command.argv[4]);
				break;

			case MOVE_BY_OFFSET:
				std::cout<<"move by offset!"<<std::endl;
				status = moveByPositionOffset(api, flight, command.argv[0], command.argv[1], command.argv[2], yaw,command.argv[4]);
				break;

			case MOVE_WITH_VELOCITY:
				std::cout<<"move with velocity!"<<std::endl;
				status = moveWithVelocity(api, flight, command.argv[0], command.argv[1], command.argv[2], yaw,command.argv[4]);
				break;

			case GO_HOME:
				std::cout<<"go home!"<<std::endl;
				ackStatus = goHome(flight,blockingTimeout);
				break;

			case LAND:
				std::cout<<"landing!"<<std::endl;
				ackStatus = landing(api, flight,blockingTimeout);
				break;

			default:
				std::cout<<"指令错误"<<std::endl;
				break;
		}
	}
	
	pthread_cancel(a_thread);

	std::cout<<"退出while循环"<<std::endl;
	//! Cleanup
	ackReturnData landingStatus = landing(api, flight,blockingTimeout);
	int cleanupStatus = cleanup(serialDevice, api, flight, read1);
	if (cleanupStatus == -1)
	{
		std::cout << "Unable to cleanly destroy OSDK infrastructure. There may be residual objects in the system memory.\n";
		return 0;
	}
	std::cout << "Program exited successfully." << std::endl;
	
	//close socket
	close(socket_fd);  
	return 0;
}

void initSocket(int &socket_fd)
{
	//初始化Socket  
	if( (socket_fd = socket(AF_INET, SOCK_DGRAM, 0)) == -1 )
	{  
		printf("create socket error: %s(errno: %d)\n",strerror(errno),errno);  
		exit(0);  
	}  
	//初始化  
	struct sockaddr_in servaddr;  
	memset(&servaddr, 0, sizeof(servaddr));  
	servaddr.sin_family = AF_INET;  
	servaddr.sin_addr.s_addr = htonl(INADDR_ANY);//IP地址设置成INADDR_ANY,让系统自动获取本机的IP地址。  
	servaddr.sin_port = htons(5000);//设置的端口为4000
	//将本地地址绑定到所创建的套接字上  
	const int rcv_size = 310*1024;//设置接收缓冲区大小
	if(setsockopt(socket_fd,SOL_SOCKET,SO_RCVBUF,(char *)&rcv_size,sizeof(rcv_size))<0)
	{
		perror("set the size of rcv_buf");
		exit(0);
	}
	if( bind(socket_fd, (struct sockaddr*)&servaddr, sizeof(servaddr)) == -1)
	{  
		printf("bind socket error: %s(errno: %d)\n",strerror(errno),errno);  
		exit(0);  
	}  
}

int RecvData(int sock, void *buf, int size)
{
	int err;
	struct sockaddr_in clientaddr;
	socklen_t clientaddrLen = sizeof(clientaddr);
	int sum = size;
	int index = 0;
	while (size != 0)
	{
		err = recvfrom(sock, (char*)buf + index, size, 0,(struct sockaddr *)&clientaddr,&clientaddrLen);
		if (err == -1) return -1;
		else if (err == 0) return -1;
		size -= err;
		index += err;
	}
	return sum - size;
}

void *RecvCommand(void *argv)
{
	COMMAND command;	
	clock_t start,end;
	double t;
	int count = 0;//计数接收到的指令数量
	while(1)	
	{
		int ret = RecvData(socket_fd,&command,sizeof(struct COMMAND));
		if(ret<0)
		{
			//close socket
			close(socket_fd);  

			std::cout<<"接收数据发生错误"<<std::endl;
			ackReturnData landingStatus = landing(api, flight,blockingTimeout);
			int cleanupStatus = cleanup(serialDevice, api, flight, read1);
			if (cleanupStatus == -1)
			{
				std::cout << "Unable to cleanly destroy OSDK infrastructure. There may be residual objects in the system memory.\n";
				break;
			}
			std::cout << "Program exited successfully." << std::endl;

		}

		pthread_mutex_lock(&mutex);
		stackCommand.push(command);
		pthread_mutex_unlock(&mutex);

		count++;
		end = clock();
		t = (double)(end-start);
		outfile<<count<<"："<<t/CLOCKS_PER_SEC*1000<<std::endl;
		std::cout<<count<<"："<<t/CLOCKS_PER_SEC*1000<<std::endl;
		start = end;
	}
	std::cout<<"退出子线程"<<std::endl;
}
