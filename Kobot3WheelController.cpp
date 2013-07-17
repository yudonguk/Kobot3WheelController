#include "Kobot3WheelController.h"

#include <boost/chrono.hpp>

#include <device/OprosPrintMessage.h>
#include <device/OprosTimer.h>

#include "debug_macro.h"
#include "Kobot3MotionBoard.h"

//Define Required Profile Name List
#define WHEEL_DIAMETER "WheelDiameter"
#define AXLE_DISTANCE "AxleDistance"
#define MAXIMUM_VELOCITY "MaximumVelocity"
#define ACCELERATION "Acceleration"
#define ENCODER_PULSE_PER_ROTATION "EncoderPulsePerRotation"
#define CONTROL_POSITION_ERROR_LIMIT "ControlPositionErrorLimit"
#define CONTROL_P_GAIN "ControlPGain"
#define CONTROL_I_GAIN "ControlIGain"
#define CONTROL_D_GAIN "ControlDGain"
//End of Define Required Profile Name List

class SimpleTimer
{
public:
	SimpleTimer()
	{
		Reset();
	}

public:
	unsigned long GetTimeTick()
	{
		return (unsigned long)boost::chrono::duration_cast<boost::chrono::milliseconds>(boost::chrono::high_resolution_clock::now() - startTime).count();
	}

	void Reset()
	{
		startTime = boost::chrono::high_resolution_clock::now();
	}

private:
	boost::chrono::high_resolution_clock::time_point startTime;
};

Kobot3WheelController::Kobot3WheelController()
	: mLeftMotorId(1), mRightMotorId(0), mpMotionBoard(new Kobot3MotionBoard)
	, mpTimer(new SimpleTimer)
{
}

Kobot3WheelController::~Kobot3WheelController()
{
	Finalize();
}

int Kobot3WheelController::Initialize( Property parameter )
{
	if (_status != DEVICE_CREATED)
	{
		PrintMessage(DEBUG_MESSAGE("Already initialized").c_str());
		return API_ERROR;
	}

	//////////////////////////////////////////////////////////////////////////
	
	mpTimer->Reset();

	if (SetProperty(parameter) == false)
	{
		PrintMessage(DEBUG_MESSAGE("Can't set property").c_str());
		return API_ERROR;
	}

	vector<wstring> motionBoardIdList = Kobot3MotionBoard::GetMotionBoadIdList();
	if (motionBoardIdList.size() == 0)
	{
		PrintMessage(DEBUG_MESSAGE("Can't find Kobot3 Motionboard").c_str());		
		return API_ERROR;
	}

	if (mpMotionBoard->SetMotionBoardId(motionBoardIdList[0]) == false)
	{
		PrintMessage(DEBUG_MESSAGE("").c_str());
		return API_ERROR;
	}

	//////////////////////////////////////////////////////////////////////////
	_status = DEVICE_READY;
	return API_SUCCESS;
}

int Kobot3WheelController::Finalize()
{
	if (_status == DEVICE_ACTIVE || _status == DEVICE_ERROR)
		Disable();

	//////////////////////////////////////////////////////////////////////////


	//////////////////////////////////////////////////////////////////////////

	_status = DEVICE_CREATED;
	return API_SUCCESS;
}

int Kobot3WheelController::Enable()
{
	if (_status == DEVICE_ACTIVE)
	{
		PrintMessage(DEBUG_MESSAGE("Already enabled").c_str());
		return API_SUCCESS;
	}
	else if(_status != DEVICE_READY)
	{
		PrintMessage(DEBUG_MESSAGE("Precondition not met").c_str());
		return API_ERROR;
	}

	//////////////////////////////////////////////////////////////////////////

	if(mpMotionBoard->Enable() == false)
	{
		PrintMessage(DEBUG_MESSAGE("Can't enable Kobot3 motion board").c_str());
		return API_ERROR;
	}

	if (mpMotionBoard->Stop(mLeftMotorId, FreeWheeling, mRightMotorId, FreeWheeling) == false)
	{
		PrintMessage(DEBUG_MESSAGE("Can't stop motor").c_str());
		return API_ERROR;
	}

	Kobot3WheelControllerProfile profile = mProfile;

	OprosSleep(10);
	if(mpMotionBoard->SetPulsePerRotation(mLeftMotorId, (float)profile.encoderPulsePerRotation
		, mRightMotorId, (float)profile.encoderPulsePerRotation) == false)
	{
		PrintMessage(DEBUG_MESSAGE("Can't set pulse per rotation").c_str());
		return API_ERROR;
	}

	OprosSleep(10);
	if(mpMotionBoard->SetPID(mLeftMotorId, (float)profile.controlPGain
		, (float)profile.controlIGain, (float)profile.controlDGain) == false)
	{
		PrintMessage(DEBUG_MESSAGE("Can't set PID of left motor").c_str());
		return API_ERROR;
	}

	OprosSleep(10);
	if(mpMotionBoard->SetPID(mRightMotorId, (float)profile.controlPGain
		, (float)profile.controlIGain, (float)profile.controlDGain) == false)
	{
		PrintMessage(DEBUG_MESSAGE("Can't set PID of right motor").c_str());
		return API_ERROR;
	}

	OprosSleep(10);
	int32_t milliRotatePositionErrorLimit
		= INTEGER(profile.controlPositionErrorLimit * 1000.0  / 360.0);
	if(mpMotionBoard->SetPositionErrorLimit(mLeftMotorId, milliRotatePositionErrorLimit
		, mRightMotorId, milliRotatePositionErrorLimit) == false)
	{
		PrintMessage(DEBUG_MESSAGE("Can't set position error limit").c_str());
		return API_ERROR;
	}

	OprosSleep(10);
	int32_t milliRotateAcceleration = INTEGER(profile.acceleration * 1000.0 / (profile.wheelDiameter * M_PI));
	if(mpMotionBoard->SetAcceleration(mLeftMotorId, milliRotateAcceleration
		, mRightMotorId, milliRotateAcceleration) == false)
	{
		PrintMessage(DEBUG_MESSAGE("Can't set acceleration").c_str());
		return API_ERROR;
	}

	mWheelPosition.x = 0.0;
	mWheelPosition.y = 0.0;
	mWheelPosition.theta = 0.0;

	mPreviousMotorPosition.leftPosition = 0;
	mPreviousMotorPosition.rightPosition = 0;
	mPreviousMotorPosition.time = 0;

	//////////////////////////////////////////////////////////////////////////

	_status = DEVICE_ACTIVE;
	return API_SUCCESS;
}

int Kobot3WheelController::Disable()
{
	SimpleReaderWriterLock::ScopedWriterLock scopedWriterLock(mRwLockStatus);
	if (_status == DEVICE_READY)
	{
		PrintMessage(DEBUG_MESSAGE("Already disabled").c_str());
		return API_SUCCESS;
	}
	else if(_status != DEVICE_ACTIVE && _status != DEVICE_ERROR)
	{
		PrintMessage(DEBUG_MESSAGE("Precondition not met").c_str());
		return API_ERROR;
	}

	//////////////////////////////////////////////////////////////////////////

	if (mpMotionBoard->Stop(mLeftMotorId, FreeWheeling
		, mRightMotorId, FreeWheeling) == false)
	{
		PrintMessage(DEBUG_MESSAGE("Can't stop wheel").c_str());
		return API_ERROR;
	}

	if (mpMotionBoard->Disable() == false)
	{
		PrintMessage(DEBUG_MESSAGE("Can't disable Kobot3 motion board").c_str());
		return API_ERROR;
	}

	//////////////////////////////////////////////////////////////////////////
	_status = DEVICE_READY;
	return API_SUCCESS;
}

int Kobot3WheelController::SetParameter( Property parameter )
{
	if (_status == DEVICE_CREATED)
	{
		PrintMessage(DEBUG_MESSAGE("Precondition not met").c_str());
		return API_ERROR;
	}

	if (Disable() != API_SUCCESS)
	{
		PrintMessage(DEBUG_MESSAGE("Can not disable").c_str());
		_status = DEVICE_ERROR;
		return API_ERROR;
	}

	//////////////////////////////////////////////////////////////////////////

	if (SetProperty(parameter) == false)
	{
		PrintMessage(DEBUG_MESSAGE("Can't set property").c_str());
		return API_ERROR;
	}

	//////////////////////////////////////////////////////////////////////////

	if (Enable() != API_SUCCESS)
	{
		PrintMessage(DEBUG_MESSAGE("Can not enable").c_str());
		_status = DEVICE_ERROR;
		return API_ERROR;
	}

	return API_SUCCESS;
}

int Kobot3WheelController::GetParameter( Property &parameter )
{
	if (_status == DEVICE_CREATED)
	{
		PrintMessage(DEBUG_MESSAGE("Precondition not met").c_str());
		return API_ERROR;
	}

	//////////////////////////////////////////////////////////////////////////

	Kobot3WheelControllerProfile profile = mProfile;
	std::ostringstream stringConverter;

	stringConverter << profile.wheelDiameter;
	parameter.SetValue(WHEEL_DIAMETER, stringConverter.str());
	stringConverter.str("");

	stringConverter << profile.axleDistance;
	parameter.SetValue(AXLE_DISTANCE, stringConverter.str());
	stringConverter.str("");

	stringConverter << profile.maximumVelocity;
	parameter.SetValue(MAXIMUM_VELOCITY, stringConverter.str());
	stringConverter.str("");

	stringConverter << profile.acceleration;
	parameter.SetValue(ACCELERATION, stringConverter.str());
	stringConverter.str("");

	stringConverter << profile.encoderPulsePerRotation;
	parameter.SetValue(ENCODER_PULSE_PER_ROTATION, stringConverter.str());
	stringConverter.str("");

	stringConverter << profile.controlPositionErrorLimit;
	parameter.SetValue(CONTROL_POSITION_ERROR_LIMIT, stringConverter.str());
	stringConverter.str("");

	stringConverter << profile.controlPGain;
	parameter.SetValue(CONTROL_P_GAIN, stringConverter.str());
	stringConverter.str("");

	stringConverter << profile.controlIGain;
	parameter.SetValue(CONTROL_I_GAIN, stringConverter.str());
	stringConverter.str("");

	stringConverter << profile.controlDGain;
	parameter.SetValue(CONTROL_D_GAIN, stringConverter.str());
	stringConverter.str("");

	//////////////////////////////////////////////////////////////////////////

	return API_SUCCESS;
}

int Kobot3WheelController::OnExecute()
{
	SimpleReaderWriterLock::ScopedReaderLock sopedReaderLock(mRwLockStatus);
	if (_status != DEVICE_ACTIVE)
	{
		PrintMessage(DEBUG_MESSAGE("Precondition not met").c_str());
		return API_ERROR;
	}

	//////////////////////////////////////////////////////////////////////////

	if (ProcessOdometricLocalization() == false)
	{
		return API_ERROR;
	}

	//////////////////////////////////////////////////////////////////////////
	return API_SUCCESS;
}

int Kobot3WheelController::SetPosition( ObjectLocation position )
{
	SimpleReaderWriterLock::ScopedReaderLock sopedReaderLock(mRwLockStatus);
	if (_status != DEVICE_ACTIVE)
	{
		PrintMessage(DEBUG_MESSAGE("Precondition not met").c_str());
		return API_ERROR;
	}

	//////////////////////////////////////////////////////////////////////////

	mMutexWheelPosition.Lock();
	mWheelPosition = position;
	mWheelPosition.theta = DEG2RAD(position.theta);
	mMutexWheelPosition.Unlock();

	//////////////////////////////////////////////////////////////////////////
	return API_SUCCESS;
}

int Kobot3WheelController::GetPosition( ObjectLocation &position )
{
	SimpleReaderWriterLock::ScopedReaderLock sopedReaderLock(mRwLockStatus);
	if (_status != DEVICE_ACTIVE)
	{
		PrintMessage(DEBUG_MESSAGE("Precondition not met").c_str());
		return API_ERROR;
	}

	//////////////////////////////////////////////////////////////////////////

	mMutexWheelPosition.Lock();
	position = mWheelPosition;
	position.theta = RAD2DEG(mWheelPosition.theta);
	mMutexWheelPosition.Unlock();
	/*
	std::cout << "--------------------------------" << std::endl;
	std::cout << "WheelPosition.x : " << position.x << std::endl;
	std::cout << "WheelPosition.y : " << position.y << std::endl;
	std::cout << "WheelPosition.theta : " << position.theta << std::endl;
	std::cout << "--------------------------------" << std::endl;
	*/

	//////////////////////////////////////////////////////////////////////////
	return API_SUCCESS;
}

int Kobot3WheelController::GetOdometery( vector<long> &odometery )
{
	SimpleReaderWriterLock::ScopedReaderLock sopedReaderLock(mRwLockStatus);
	if (_status != DEVICE_ACTIVE)
	{
		PrintMessage(DEBUG_MESSAGE("Precondition not met").c_str());
		return API_ERROR;
	}

	//////////////////////////////////////////////////////////////////////////

	int32_t leftMotorEncoder;
	int32_t rightMotorEncoder;

	if(mpMotionBoard->GetEncoder(mLeftMotorId, leftMotorEncoder
		, mRightMotorId, rightMotorEncoder) == false)
	{
		PrintMessage(DEBUG_MESSAGE("Can't get encoder").c_str());
		return API_ERROR;
	}

	odometery.resize(2);
	odometery[0] = leftMotorEncoder;
	odometery[1] = -rightMotorEncoder;

	//////////////////////////////////////////////////////////////////////////
	return API_SUCCESS;
}

int Kobot3WheelController::DriveWheel( double linearVelocity, double angularVelocity )
{
	SimpleReaderWriterLock::ScopedReaderLock sopedReaderLock(mRwLockStatus);
	if (_status != DEVICE_ACTIVE)
	{
		PrintMessage(DEBUG_MESSAGE("Precondition not met").c_str());
		return API_ERROR;
	}

	//////////////////////////////////////////////////////////////////////////

	linearVelocity = bound(linearVelocity, -mProfile.maximumVelocity, mProfile.maximumVelocity);

	const double differenceWheelVelocity = mProfile.axleDistance * DEG2RAD(angularVelocity) / 2.0;

	//선속도 및 각속도에 의한 양쪽 바퀴의 선속도
	double rightWheelSpeed = linearVelocity + differenceWheelVelocity;
	double leftWheelSpeed = linearVelocity - differenceWheelVelocity;

	const double wheelCircumference = (mProfile.wheelDiameter * M_PI);

	//m/s 에서 mRotate/s 로 변경
	int32_t leftMilliRoatePerSec = INTEGER(leftWheelSpeed * 1000.0 / wheelCircumference);
	int32_t rightMilliRotatePerSec = INTEGER(rightWheelSpeed * 1000.0 / wheelCircumference);

	if (mpMotionBoard->ControlVelocity(mLeftMotorId, leftMilliRoatePerSec
		, mRightMotorId, -rightMilliRotatePerSec) == false)
	{
		PrintMessage(DEBUG_MESSAGE("Can't control velocity").c_str());
		return API_ERROR;
	}	

	//////////////////////////////////////////////////////////////////////////
	return API_SUCCESS;
}

int Kobot3WheelController::MoveWheel( double distance, double linearVelocity )
{
	SimpleReaderWriterLock::ScopedReaderLock sopedReaderLock(mRwLockStatus);
	if (_status != DEVICE_ACTIVE)
	{
		PrintMessage(DEBUG_MESSAGE("Precondition not met").c_str());
		return API_ERROR;
	}

	//////////////////////////////////////////////////////////////////////////

	linearVelocity = bound(linearVelocity, -mProfile.maximumVelocity, mProfile.maximumVelocity);

	const double wheelCircumference = (mProfile.wheelDiameter * M_PI);

	//m 에서 mRotate 로 변환
	int32_t milliRotate = INTEGER(distance * 1000.0 / wheelCircumference);

	//m/s 에서 mRotate/s 로 변환
	int32_t milliRotatePerSec = abs(INTEGER(linearVelocity * 1000.0 / wheelCircumference));

	if (linearVelocity < 0.0)
		milliRotate = -milliRotate;

	if (mpMotionBoard->SetMaxVelocity(mLeftMotorId, milliRotatePerSec
		, mRightMotorId, milliRotatePerSec) == false)
	{
		PrintMessage(DEBUG_MESSAGE("Can't set max velocity for control position").c_str());
		return API_ERROR;
	}

	if(mpMotionBoard->ControlPosition(mLeftMotorId, milliRotate
		, mRightMotorId, -milliRotate) == false)
	{
		PrintMessage(DEBUG_MESSAGE("Can't control position").c_str());
		return API_ERROR;
	}

	//////////////////////////////////////////////////////////////////////////
	return API_SUCCESS;
}

int Kobot3WheelController::RotateWheel( double angle, double angularVelocity )
{
	SimpleReaderWriterLock::ScopedReaderLock sopedReaderLock(mRwLockStatus);
	if (_status != DEVICE_ACTIVE)
	{
		PrintMessage(DEBUG_MESSAGE("Precondition not met").c_str());
		return API_ERROR;
	}

	//////////////////////////////////////////////////////////////////////////

	const double degreeToRotation = mProfile.axleDistance / (360.0 * mProfile.wheelDiameter);

	// degree 에서 mRotate로 변환
	int32_t milliRotate = INTEGER(angle * 1000 * degreeToRotation);

	// degree/s 에서 mRotate/s로 변환
	int32_t milliRotatePerSec = abs(INTEGER(angularVelocity * 1000 * degreeToRotation));

	if(angularVelocity < 0.0)
		milliRotate = -milliRotate;

	if (mpMotionBoard->SetMaxVelocity(mLeftMotorId, milliRotatePerSec
		, mRightMotorId, milliRotatePerSec) == false)
	{
		PrintMessage(DEBUG_MESSAGE("Can't set max velocity for control position").c_str());
		return API_ERROR;
	}

	if(mpMotionBoard->ControlPosition(mLeftMotorId, -milliRotate
		, mRightMotorId, -milliRotate) == false)
	{
		PrintMessage(DEBUG_MESSAGE("Can't control position").c_str());
		return API_ERROR;
	}

	//////////////////////////////////////////////////////////////////////////	
	return API_SUCCESS;
}

int Kobot3WheelController::StopWheel()
{
	SimpleReaderWriterLock::ScopedReaderLock sopedReaderLock(mRwLockStatus);
	if (_status != DEVICE_ACTIVE)
	{
		PrintMessage(DEBUG_MESSAGE("Precondition not met").c_str());
		return API_ERROR;
	}

	//////////////////////////////////////////////////////////////////////////

	if (mpMotionBoard->Stop(mLeftMotorId, DeaccelerationStop
		, mRightMotorId, DeaccelerationStop) == false)
	{
		PrintMessage(DEBUG_MESSAGE("Can't stop motor").c_str());
		return API_ERROR;
	}

	/////////////////////////////////////////////////////////////////////////
	return API_SUCCESS;
}

int Kobot3WheelController::IsWheelRunning( bool &isWheelRunning )
{
	SimpleReaderWriterLock::ScopedReaderLock sopedReaderLock(mRwLockStatus);
	if (_status != DEVICE_ACTIVE)
	{
		PrintMessage(DEBUG_MESSAGE("Precondition not met").c_str());
		return API_ERROR;
	}

	//////////////////////////////////////////////////////////////////////////

	int32_t leftWheelSpeed;
	int32_t rightWheelSpeed;

	if (mpMotionBoard->GetVelocity(mLeftMotorId, leftWheelSpeed
		, mRightMotorId, rightWheelSpeed) == false)
	{
		PrintMessage(DEBUG_MESSAGE("Can't get motor velocity").c_str());
		return API_ERROR;
	}

	if (abs(leftWheelSpeed) < 10 && abs(rightWheelSpeed) < 10)
	{
		isWheelRunning = false;
	}
	else
	{
		isWheelRunning = true;
	}

	//////////////////////////////////////////////////////////////////////////
	return API_SUCCESS;
}

bool Kobot3WheelController::SetProperty( Property& propery )
{
	Kobot3WheelControllerProfile profile = mProfile;

	PrintMessage("--------------------------------\r\n");
	PrintMessage("Kobot3WheelController Profiles\r\n");
	PrintMessage("--------------------------------\r\n");

	if (propery.FindName(WHEEL_DIAMETER) == true)
		profile.wheelDiameter = atof(propery.GetValue(WHEEL_DIAMETER).c_str());
	PrintMessage("%s : %f\r\n", WHEEL_DIAMETER, profile.wheelDiameter);

	if (propery.FindName(AXLE_DISTANCE) == true)
		profile.axleDistance = atof(propery.GetValue(AXLE_DISTANCE).c_str());
	PrintMessage("%s : %f\r\n", AXLE_DISTANCE, profile.axleDistance);

	if (propery.FindName(MAXIMUM_VELOCITY) == true)
		profile.maximumVelocity = atof(propery.GetValue(MAXIMUM_VELOCITY).c_str());
	PrintMessage("%s : %f\r\n", MAXIMUM_VELOCITY, profile.maximumVelocity);

	if (propery.FindName(ACCELERATION) == true)
		profile.acceleration = atof(propery.GetValue(ACCELERATION).c_str());
	PrintMessage("%s : %f\r\n", ACCELERATION, profile.acceleration);

	if (propery.FindName(ENCODER_PULSE_PER_ROTATION) == true)
		profile.encoderPulsePerRotation = atof(propery.GetValue(ENCODER_PULSE_PER_ROTATION).c_str());
	PrintMessage("%s : %f\r\n", ENCODER_PULSE_PER_ROTATION, profile.encoderPulsePerRotation);

	if (propery.FindName(CONTROL_POSITION_ERROR_LIMIT) == true)
		profile.controlPositionErrorLimit = atof(propery.GetValue(CONTROL_POSITION_ERROR_LIMIT).c_str());
	PrintMessage("%s : %f\r\n", CONTROL_POSITION_ERROR_LIMIT, profile.controlPositionErrorLimit);

	if (propery.FindName(CONTROL_P_GAIN) == true)
		profile.controlPGain = atof(propery.GetValue(CONTROL_P_GAIN).c_str());
	PrintMessage("%s : %f\r\n", CONTROL_P_GAIN, profile.controlPGain);

	if (propery.FindName(CONTROL_I_GAIN) == true)
		profile.controlIGain = atof(propery.GetValue(CONTROL_I_GAIN).c_str());
	PrintMessage("%s : %f\r\n", CONTROL_I_GAIN, profile.controlIGain);

	if (propery.FindName(CONTROL_D_GAIN) == true)
		profile.controlDGain = atof(propery.GetValue(CONTROL_D_GAIN).c_str());
	PrintMessage("%s : %f\r\n", CONTROL_D_GAIN, profile.controlDGain);

	PrintMessage("--------------------------------\r\n");

	mProfile = profile;

	return true;
}

bool Kobot3WheelController::ProcessOdometricLocalization()
{
	MotorPosition motorPosition;
	{
		int32_t leftWheelPosition;
		int32_t rightWheelPosition;
		if (mpMotionBoard->GetPosition(mLeftMotorId, leftWheelPosition
			, mRightMotorId, rightWheelPosition) == false)
		{
			PrintMessage(DEBUG_MESSAGE("Can't get motor position").c_str());
			return false;
		}
		motorPosition.time = mpTimer->GetTimeTick();
		motorPosition.leftPosition = (leftWheelPosition / 1000.0) * 2 * M_PI;
		motorPosition.rightPosition = (-rightWheelPosition / 1000.0) * 2 * M_PI;
	}

	if (mPreviousMotorPosition.time != 0)
	{
		const double deltaLeftPosition = motorPosition.leftPosition - mPreviousMotorPosition.leftPosition;
		const double deltaRightPosition = motorPosition.rightPosition - mPreviousMotorPosition.rightPosition;

		const double dt = (motorPosition.time - mPreviousMotorPosition.time) / 1000.0;
		const double linearVelocity = mProfile.wheelDiameter * (deltaRightPosition + deltaLeftPosition) / (4 * dt);
		const double angularVelocity = mProfile.wheelDiameter * (deltaRightPosition - deltaLeftPosition) / (2 * mProfile.axleDistance * dt);

		if (!_finite(angularVelocity))
		{
			//__asm int 3;
		}

		mMutexWheelPosition.Lock();
		if (abs(angularVelocity) < 0.0001)
		{
			//2nd order Runge-Kutta integration
			// x(t+1) = x(t) + v(t) * Periode * cos(theta(t) + w(t) * Periode / 2)
			// y(t+1) = y(t) + v(t) * Periode * sin(theta(t) + w(t) * Periode / 2)
			// theta(t+1) = theta(t) + w(t) * Periode
			const double theta = mWheelPosition.theta + angularVelocity * dt / 2.0;
			const double moveDistance = linearVelocity * dt;

			mWheelPosition.x += moveDistance * cos(theta);
			mWheelPosition.y += moveDistance * sin(theta);
			mWheelPosition.theta += angularVelocity * dt;
		}
		else
		{
			// exact integration
			// x(t+1) = x(t) + v(t) / w(t) * (sin(theta(t+1)) - sin(theta(t))))
			// y(t+1) = y(t) - v(t) / w(t) * (cos(theta(t+1)) - cos(theta(t))))
			// theta(t+1) = theta(t) + w(t) * Periode
			double theta = mWheelPosition.theta;
			double theta2 = mWheelPosition.theta + angularVelocity * dt;
			double linearVelocityPerAngularVelocity = linearVelocity / angularVelocity;

			mWheelPosition.x += linearVelocityPerAngularVelocity * (sin(theta2) - sin(theta));
			mWheelPosition.y -= linearVelocityPerAngularVelocity * (cos(theta2) - cos(theta));
			mWheelPosition.theta = theta2;
		}
		mMutexWheelPosition.Unlock();
	}

	mPreviousMotorPosition = motorPosition;

	return true;
}


#ifdef WIN32
extern "C"
{
	__declspec(dllexport) OprosApi* GetAPI();
	__declspec(dllexport) void ReleaseAPI(OprosApi* pOprosApi);
}

OprosApi* GetAPI()
{
	return new Kobot3WheelController();
}

void ReleaseAPI(OprosApi* pOprosApi)
{
	delete pOprosApi;
}

#else
extern "C"
{
	OprosApi* GetAPI();
	void ReleaseAPI(OprosApi* pOprosApi);
}

OprosApi* GetAPI()
{
	return new Kobot3WheelController();
}

void ReleaseAPI(OprosApi* pOprosApi)
{
	delete pOprosApi;
}

#endif

//Undefine Required Profile Name List
#undef WHEEL_DIAMETER 
#undef AXLE_DISTANCE 
#undef MAXIMUM_VELOCITY 
#undef ACCELERATION  
#undef ENCODER_PULSE_PER_ROTATION  
#undef CONTROL_POSITION_ERROR_LIMIT 
#undef CONTROL_P_GAIN  
#undef CONTROL_I_GAIN  
#undef CONTROL_D_GAIN  
//End of Undefine Required Profile Name List
