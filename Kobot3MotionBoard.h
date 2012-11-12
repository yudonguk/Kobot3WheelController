#ifndef __KOBOT_3_MOTION_BOARD_H__
#define __KOBOT_3_MOTION_BOARD_H__

#include <vector>
#include <string>
#include <cstdint>

enum MotionControllerErrorCode
{
	None = 0,
	UnknownError = 1,
	NotSupportedCommand = 2,
	NotHasProcessedPreviousCommand = 3,
	InvalidMotorId = 10,
	EnteringRebootingProcess = 11
};

enum MotionControllerStopMode
{
	EmergencyStop = 0, DeaccelerationStop = 1, FreeWheeling = 2
};

enum CommandType
{
	RESET_MOTION_BOARD = 0	
	, PID_TUNNING_MODE = 1, VELOCITY_CONTROL_MODE = 2, POSITION_CONTROL_MODE = 3, STOP_MODE = 4
	, GET_STATUS = 9, GET_ENCODER = 10, GET_VELOCITY = 11, GET_POSITION = 12
	, GET_PULSE_PER_ROTAITION = 20, GET_ACCELERATION = 21, GET_MAX_VELOCITY_FOR_CONTROL_POSITION = 22
	, GET_POSITION_ERROR_LIMIT = 23, GET_P_GAIN = 24, GET_I_GAIN = 25, GET_D_GAIN = 26
	, SET_PULSE_PER_ROTAITION = 148, SET_ACCELERATION = 149, SET_MAX_VELOCITY_FOR_CONTROL_POSITION = 150
	, SET_POSITION_ERROR_LIMIT = 151, SET_P_GAIN = 152, SET_I_GAIN = 153, SET_D_GAIN = 154
};

struct MotionControllerCommand
{
	uint8_t command;
	uint8_t option;

	union
	{
		uint8_t rawData[4];
		int32_t int32Data;
		uint32_t uint32Data;
		float float32Data;
		uint32_t errorCode;
	} data;
};

typedef struct hid_device_ hid_device;
class SimpleMutex;

class Kobot3MotionBoard
{
public:
	static const uint16_t VID = 0x5975;
	static const uint16_t PID = 0x4B4D;
		
public:
	Kobot3MotionBoard(const std::wstring& motionBoardId = L"");
	~Kobot3MotionBoard();

private:
	// no copyable
	Kobot3MotionBoard(const Kobot3MotionBoard&);
	const Kobot3MotionBoard& operator =(const Kobot3MotionBoard&);

public:
	static std::vector<std::wstring> GetMotionBoadIdList();

public:
	bool Enable();
	bool Disable();
	void EnableDebugMessage(bool isEnable);

	bool SetMotionBoardId(const std::wstring& motionBoardId);
	inline std::wstring GetMotionBoardId();

	template<size_t requestCount, size_t responseCount>
	inline bool Request(const MotionControllerCommand (&requests)[requestCount], MotionControllerCommand (&responses)[responseCount])
	{
		static_assert(requestCount == responseCount, "requestCount must be equal responseCount.");
		static_assert(requestCount <= 4, "requestCount must be less than or equal to 4.");
		return Request(requests, responses, requestCount);
	}
	inline bool Request(const MotionControllerCommand& request, MotionControllerCommand& response)
	{
		return Request(&request, &response, 1);
	}
	bool Request(const MotionControllerCommand* requests, MotionControllerCommand* responses, size_t requestCount);
	
	bool GetStatus(MotionControllerErrorCode& errorCode);
	bool Reset();

	bool ControlPidTunning(uint8_t motorId1, int32_t milliRotatePerSec1);
	bool ControlPidTunning(uint8_t motorId1, int32_t milliRotatePerSec1, uint8_t motorId2, int32_t milliRotatePerSec2);
	bool ControlPidTunning(uint8_t motorId1, int32_t milliRotatePerSec1, uint8_t motorId2, int32_t milliRotatePerSec2, uint8_t motorId3, int32_t milliRotatePerSec3);

	bool ControlVelocity(uint8_t motorId1, int32_t milliRotatePerSec1);
	bool ControlVelocity(uint8_t motorId1, int32_t milliRotatePerSec1, uint8_t motorId2, int32_t milliRotatePerSec2);
	bool ControlVelocity(uint8_t motorId1, int32_t milliRotatePerSec1, uint8_t motorId2, int32_t milliRotatePerSec2, uint8_t motorId3, int32_t milliRotatePerSec3);

	bool ControlPosition(uint8_t motorId1, int32_t milliRotate1);
	bool ControlPosition(uint8_t motorId1, int32_t milliRotate1, uint8_t motorId2, int32_t milliRotate2);
	bool ControlPosition(uint8_t motorId1, int32_t milliRotate1, uint8_t motorId2, int32_t milliRotate2, uint8_t motorId3, int32_t milliRotate3);

	bool Stop(uint8_t motorId1, MotionControllerStopMode stopMode1);
	bool Stop(uint8_t motorId1, MotionControllerStopMode stopMode1, uint8_t motorId2, MotionControllerStopMode stopMode2);
	bool Stop(uint8_t motorId1, MotionControllerStopMode stopMode1, uint8_t motorId2, MotionControllerStopMode stopMode2, uint8_t motorId3, MotionControllerStopMode stopMode3);

	bool GetEncoder(uint8_t motorId1, int32_t& encoder1);
	bool GetEncoder(uint8_t motorId1, int32_t& encoder1, uint8_t motorId2, int32_t& encoder2);
	bool GetEncoder(uint8_t motorId1, int32_t& encoder1, uint8_t motorId2, int32_t& encoder2, uint8_t motorId3, int32_t& encoder3);

	bool GetVelocity(uint8_t motorId1, int32_t& milliRotatePerSec1);
	bool GetVelocity(uint8_t motorId1, int32_t& milliRotatePerSec1, uint8_t motorId2, int32_t& milliRotatePerSec2);
	bool GetVelocity(uint8_t motorId1, int32_t& milliRotatePerSec1, uint8_t motorId2, int32_t& milliRotatePerSec2, uint8_t motorId3, int32_t& milliRotatePerSec3);

	bool GetPosition(uint8_t motorId1, int32_t& milliRotate1);
	bool GetPosition(uint8_t motorId1, int32_t& milliRotate1, uint8_t motorId2, int32_t& milliRotate2);
	bool GetPosition(uint8_t motorId1, int32_t& milliRotate1, uint8_t motorId2, int32_t& milliRotate2, uint8_t motorId3, int32_t& milliRotate3);
	
	bool GetPulsePerRotation(uint8_t motorId1, float& pulsePerRotate1);
	bool GetPulsePerRotation(uint8_t motorId1, float& pulsePerRotate1, uint8_t motorId2, float& pulsePerRotate2);
	bool GetPulsePerRotation(uint8_t motorId1, float& pulsePerRotate1, uint8_t motorId2, float& pulsePerRotate2, uint8_t motorId3, float& pulsePerRotate3);

	bool SetPulsePerRotation(uint8_t motorId1, float pulsePerRotate1);
	bool SetPulsePerRotation(uint8_t motorId1, float pulsePerRotate1, uint8_t motorId2, float pulsePerRotate2);
	bool SetPulsePerRotation(uint8_t motorId1, float pulsePerRotate1, uint8_t motorId2, float pulsePerRotate2, uint8_t motorId3, float pulsePerRotate3);

	bool GetAcceleration(uint8_t motorId1, int32_t& miliRotatePerSecSec1);
	bool GetAcceleration(uint8_t motorId1, int32_t& miliRotatePerSecSec1, uint8_t motorId2, int32_t& miliRotatePerSecSec2);
	bool GetAcceleration(uint8_t motorId1, int32_t& miliRotatePerSecSec1, uint8_t motorId2, int32_t& miliRotatePerSecSec2, uint8_t motorId3, int32_t& miliRotatePerSecSec3);

	bool SetAcceleration(uint8_t motorId1, int32_t miliRotatePerSecSec1);
	bool SetAcceleration(uint8_t motorId1, int32_t miliRotatePerSecSec1, uint8_t motorId2, int32_t miliRotatePerSecSec2);
	bool SetAcceleration(uint8_t motorId1, int32_t miliRotatePerSecSec1, uint8_t motorId2, int32_t miliRotatePerSecSec2, uint8_t motorId3, int32_t miliRotatePerSecSec3);

	bool GetMaxVelocity(uint8_t motorId1, int32_t& miliRotatePerSec1);
	bool GetMaxVelocity(uint8_t motorId1, int32_t& miliRotatePerSec1, uint8_t motorId2, int32_t& miliRotatePerSec2);
	bool GetMaxVelocity(uint8_t motorId1, int32_t& miliRotatePerSec1, uint8_t motorId2, int32_t& miliRotatePerSec2, uint8_t motorId3, int32_t& miliRotatePerSec3);

	bool SetMaxVelocity(uint8_t motorId1, int32_t miliRotatePerSec1);
	bool SetMaxVelocity(uint8_t motorId1, int32_t miliRotatePerSec1, uint8_t motorId2, int32_t miliRotatePerSec2);
	bool SetMaxVelocity(uint8_t motorId1, int32_t miliRotatePerSec1, uint8_t motorId2, int32_t miliRotatePerSec2, uint8_t motorId3, int32_t miliRotatePerSec3);

	bool GetPositionErrorLimit(uint8_t motorId1, int32_t& miliRotatePerSec1);
	bool GetPositionErrorLimit(uint8_t motorId1, int32_t& miliRotatePerSec1, uint8_t motorId2, int32_t& miliRotatePerSec2);
	bool GetPositionErrorLimit(uint8_t motorId1, int32_t& miliRotatePerSec1, uint8_t motorId2, int32_t& miliRotatePerSec2, uint8_t motorId3, int32_t& miliRotatePerSec3);
	
	bool SetPositionErrorLimit(uint8_t motorId1, int32_t miliRotatePerSec1);
	bool SetPositionErrorLimit(uint8_t motorId1, int32_t miliRotatePerSec1, uint8_t motorId2, int32_t miliRotatePerSec2);
	bool SetPositionErrorLimit(uint8_t motorId1, int32_t miliRotatePerSec1, uint8_t motorId2, int32_t miliRotatePerSec2, uint8_t motorId3, int32_t miliRotatePerSec3);
	
	bool GetPID(uint8_t motorId, float& gainP, float& gainI, float& gainD);
	bool SetPID(uint8_t motorId, float gainP, float gainI, float gainD);
	
private:
	bool mIsEnableDebugMessage;
	std::wstring mMotionBoardId;
	
	hid_device* mpDevice;
	SimpleMutex* const mpMutexDevice;
};

#endif