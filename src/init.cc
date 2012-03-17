#include "robotBase.h"
#include "stage.hh" //stage.h

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

using namespace Stg;
using namespace std;

int PositionUpdate(Model* mod, RobotBase* robot);
void ProcessMessage( WifiMessage * mesg);

RobotBase * RobotRef[200];

//Stage call this when the model starts up
extern "C" int Init( Model* mod, CtrlArgs* args )
{
	RobotBase* robot;

	RobotRef[robot->id] = robot; //每个robot都有一个id，直接访问就行
   
	robot->position->AddUpdateCallback((stg_model_callback_t)PositionUpdate, robot);

	robot->laser->Subscribe(); //starts the laser updates
	robot->position->Subscribe();
	robot->wifi->Subscribe();
	robot->wifi->comm.SetReceiveMsgFn(ProcessMessage);  //Set function of receiving message.告诉仿真器这是处理信息的函数
	return 0;
}

void ProcessMessage( WifiMessageBase * mesg)
{
	int RecipientId = mesg->GetRecipientId();

	RobotRef[RecipientId/4 - 1]->MessageReceive(mesg);  //接收信息
}

//call the robot's loop function
int PositionUpdate( Model* mod, RobotBase* robot)
{
	robot->DoLoop();
	return 0; //Run again
}
