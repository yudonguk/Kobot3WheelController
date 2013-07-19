#ifndef __KOBOT_3_WHEEL_CONTROLLER_H__
#define __KOBOT_3_WHEEL_CONTROLLER_H__

#include <memory>
#include <functional>
#include <queue>
#include <cstdint>

#include <device/WheelController.h>
#include <device/ServoActuator.h>

#include "SimpleLock.h"

class SimpleTimer;
class Notifier;
class Kobot3MotionBoard;

class Kobot3WheelController : public WheelController
{
public:
	class Kobot3WheelControllerProfile : public WheelControllerProfile
	{
	public:
		Kobot3WheelControllerProfile()
		{
			wheelDiameter = 0.12;
			axleDistance = 0.32; 
			acceleration = 0.4;
			maximumVelocity = 0.2;

			encoderPulsePerRotation = 1482;
			controlPositionErrorLimit = 5.0;
			controlPGain = 0.033;
			controlIGain = 6.60;
			controlDGain = 0.00011;
		}

		double encoderPulsePerRotation;
		double controlPositionErrorLimit;
		double controlPGain;
		double controlIGain;
		double controlDGain;
	};

	class MotorPosition 
	{
	public:
		MotorPosition(unsigned time_ = 0, double rightPosition_ = 0, double leftPosition_ = 0)
			: time(time_), rightPosition(rightPosition_), leftPosition(leftPosition_)
		{}

		unsigned long time;
		double rightPosition;
		double leftPosition;
	};

public:
	Kobot3WheelController();
	~Kobot3WheelController();

private:
	// 복사 생성자 및 연산자 제거
	Kobot3WheelController(const Kobot3WheelController&);
	const Kobot3WheelController& operator=(const Kobot3WheelController&);

public:
	virtual int Initialize(Property parameter);
	virtual int Finalize();
	virtual int Enable();
	virtual int Disable();
	virtual int SetParameter(Property parameter);
	virtual int GetParameter(Property &parameter);
	virtual int OnExecute();

public:
	virtual int SetPosition(ObjectLocation position);
	virtual int GetPosition(ObjectLocation &position);
	virtual int GetOdometery(vector<long> &odometery);
	virtual int DriveWheel(double linearVelocity, double angularVelocity);
	virtual int MoveWheel(double distance, double linearVelocity);
	virtual int RotateWheel(double angle, double angularVelocity);
	virtual int StopWheel();
	virtual int IsWheelRunning(bool &isWheelRunning);

private:
	bool SetProperty(Property& propery);
	bool ProcessOdometricLocalization();

private:
	const unsigned char mLeftMotorId;
	const unsigned char mRightMotorId;

	const uint32_t mTimeout;

	Kobot3WheelControllerProfile mProfile;

	std::unique_ptr<Kobot3MotionBoard> mpMotionBoard;

	SimpleReaderWriterLock mRwLockStatus;

	std::unique_ptr<SimpleTimer> mpTimer;
	MotorPosition mPreviousMotorPosition;
	ObjectLocation mWheelPosition;
	SimpleMutex mMutexWheelPosition;

	std::queue<std::pair<std::function<void()>, std::shared_ptr<Notifier>>> mCommandQueue;
	SimpleMutex mCommandQueueMutex;
};

#endif