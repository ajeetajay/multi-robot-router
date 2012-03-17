#include "robotBase.h"
#include <sstream>

using namespace std;

RobotBase::RobotBase(Model* model)
{
	if(model == NULL)
	{
		cout << "Initalizing failure!!!\n"<<endl;
		return;
	}

	//Initialize movement control parameter
	avoidcount = 0;
	randcount = 0;

	verbose = false;
	processMsg = false;
	communication = 0;

	//Initialize devices
	position = (ModelPosition*) model;
	laser = (ModelLaser*) model->GetChild("laser:0");
	wifi = (ModelWifi*) model->GetChild("wifi:0");

	id = wifi->GetId()/4 -1; 
	
}

void RobotBase::DoLoop()
{
	switch(this->movementType)
	{
		case 0:
			Broadcast();
			break;
		case 1:
			Wander();
			break;
		case 2:
			MoveStraight();
			break;
		default:
			printf("MOVEMENT TYPE FAILURE!\n");
			break;
	}
}

void RobotBase::MessageReceive(WifiMessageBase* mesg)
{
	printf("Robot: %d Receive a message!", id);
}


void RobotBase::Broadcast(WifiMessageBase* msg)
{
	wifi->comm.SendBroadcastMessage(msg);
}

unsigned int RobotBase::GetTime()
{
	return (unsigned int)position->GetWorld()->SimTimeNow();
}

void RobotBase::MoveStraight()
{
	const double cruisespeed = 0.2;

	if(this->position->GetGloablPose().x - this->startx < MaxMove)
	{
		position->SetXSpeed(cruisespeed);
	    position->SetTurnSpeed(0);
	}
	else
	{
		position->SetXSpeed(0);
		position->SetTurnSpeed(0);
	}
}

int RobotBase::Wander()
{
	const double cruisespeed = 0.2;
	const double avoidspeed = 0.1;
	const double avoidturn = 1;
	const double minfrontdistance = 2.0;
	const double stopdist = 0.7;
	const int avoidduration = 10;

	//get the data
	uint32_t sample_count = 0;
	ModelLaser::Sample* scan = this->laser->GetSamples(&sample_count);
	if( !scan)
		return 0;
	
	bool obstruction = false;
	bool stop = false;

	//find the closet distance to the left and right and check if there is anything in front
	double minleft = 1e6;
	double minright = 1e6;

	for(uint32_t i = 0; i < sample_count; i++)
	{
		if( i > (sample_count/3) && (i < (sample_count - (sample_count/3))) && scan[i].range < minfrontdistance)
		{
			obstruction = true;
		}

		if( scan[i].range < stopdist )
		{
			stop = true;
		}

		if( i > sample_count/2)
			minleft = std::min(minleft, scan[i].range);
		else
			minright = std::min(minright, scan[i].range);
	}

	if( obstruction || stop || ( avoidcount > 0))
	{
		position->SetXSpeed( stop ? 0.0 : avoidspeed );

		//once we start avoiding, select a turn direction and stick with it 
		if( avoidcount < 1)
		{
			avoidcount = random() % avoidduration + avoidduration;
			
			if( minleft < minright )
			{
				position->SetTurnSpeed( -avoidturn );
			}
			else
			{
				position->SetTurnSpeed( +avoidturn );
			}
		}

		avoidcount--;
	}
    //else go straight
	else
	{
		avoidcount = 0;
		position->SetXSpeed( cruisespeed );
		position->SetTurnSpeed( 0 );
	}

	return 0;

}
