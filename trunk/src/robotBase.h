#include "stage.h"
#include "Simulation.h" //文件格式，暂时不用

#ifndef __ROBOTBASE_H__
#define __ROBOTBASE_H__

using namespace Stg;
using namespace std;

enum LogType {DebugLog, ErrorLog, NormalLog}

class RobotBase
{
	public:

		//ID of robot, got from wifi id
		unsigned int id;
		unsigned long communication;

		//Representation of devices
		ModelPosition *position;  //360 degree detection
		ModelLaser *laser;
		ModelWifi *wifi;

		//Movement control
		int avoidcount, randcount;

		bool verbose;
		bool processMsg;
		int movementType;

	public:
		//Constructor
		RobotBase(Model* model);
		//Destructor
		~RobotBase(){};
		//What the robot dose in every loop
		void DoLoop();

		//Return the state of the robot
		int IsActive();

		//Robot wander and avoid obstacles in the environment
		int Wander();
		void MoveStraight();
        
		//Transmit mode: Broadcast
		void Broadcast(WifiMessageBase* msg);

		//Porcess message
		virtual void MessageReceive(WifiMessageBase* mesg) {};
	
	protected:
		unsigned int GetTime();
		void Log(string Msg, LogType type=NormalLog);
		void Sleep(unsigned int sec);
		int startx;
		
}; //end Robot

const unsigned int VerboseList[] = {4, 8, 12};
const int VerboseNumber = 0;
const int MaxMove = 25;

#endif
