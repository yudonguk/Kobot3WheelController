#ifdef TEST_BUILD

#if defined(WIN32)
#include <conio.h>
#include <windows.h>
#else
#include <pthread.h>
#include <term.h>
#include <termios.h>
#include <unistd.h>
#endif

#include <opencv/highgui.h>
#include <opencv/cv.h>

#include <device/OprosPrintMessage.h>
#include <device/OprosTimer.h>


#include "debug_macro.h"
#include "Kobot3WheelController.h"

#ifndef WIN32
int getch()
{
	int ch;
	struct termios buf;
	struct termios save;
	tcgetattr(0, &save);
	buf = save;
	buf.c_lflag &= ~(ICANON|ECHO);
	buf.c_cc[VMIN] = 1;
	buf.c_cc[VTIME] = 0;
	tcsetattr(0, TCSAFLUSH, &buf);
	ch = getchar();
	tcsetattr(0, TCSAFLUSH, &save);
	return ch;
}
#endif

Property GetProperty()
{
	Property result;
	
	return result;
}

#if defined(WIN32)
DWORD WINAPI ThreadHandler(LPVOID lParam)
#else
void* ThreadHandler(void* lParam)
#endif
{
	WheelController* wheel = (WheelController*)lParam;
	ObjectLocation position;

	IplImage* image = cvCreateImage(cvSize(1000, 1000), 8, 3);
	IplImage* backGroundImage = cvCreateImage(cvSize(image->width, image->height), image->depth, image->nChannels);

	const double SCALE = 100.0;

	cvSet(backGroundImage,  CV_RGB(255, 255, 255));

	//X line
	double centerX = image->width / 2.0;
	for (int i = 0, max = image->width / SCALE; i < max; i++)
	{
		int x = centerX - (max / 2 - i) * SCALE;
		cvDrawLine(backGroundImage, cvPoint(x, 0), cvPoint(x, image->height), CV_RGB(0, 0, 0));
	}
	//Y line
	double centerY = image->height / 2.0;
	for (int i = 0, max = image->height / SCALE; i < max; i++)
	{
		int y = centerY - (max / 2 - i) * SCALE;
		cvDrawLine(backGroundImage, cvPoint(0, y), cvPoint(image->width, y), CV_RGB(0, 0, 0));
	}
	
	auto DrawRobot = 
	[&SCALE](IplImage* image, const ObjectLocation& robotPosition)
	{
		const CvScalar robotColor = CV_RGB(255, 0, 0);
		const int width = image->width;
		const int height = image->height;
		const double x = robotPosition.x;
		const double y = robotPosition.y;
		const double theta = DEG2RAD(-robotPosition.theta);
		const double cosValue = cos(theta);
		const double sinValue = sin(theta);
		
		static CvMat* rotationMatrix = cvCreateMat(3, 3, CV_32FC1);
		cvmSet(rotationMatrix, 0, 0, cosValue);
		cvmSet(rotationMatrix, 0, 1, -sinValue);
		cvmSet(rotationMatrix, 1, 0, sinValue);
		cvmSet(rotationMatrix, 1, 1, cosValue);

		//¸öÃ¼
		{
			static CvMat* direction = cvCreateMat(3, 1, CV_32FC1);
			const double centerX = width / 2.0 + x * SCALE;
			const double centerY = height / 2.0 - y * SCALE;

			const double radius = 0.3 * SCALE;
			cvmSet(direction, 0, 0, radius);
			cvmSet(direction, 1, 0, 0);

			cvmMul(rotationMatrix, direction, direction);

			cvCircle(image, cvPoint(centerX, centerY), radius, CV_RGB(255, 255, 255), -1);
			cvCircle(image, cvPoint(centerX, centerY), radius, robotColor);
			cvLine(image, cvPoint(centerX, centerY)
				, cvPoint(centerX + cvmGet(direction, 0, 0), centerY + cvmGet(direction, 1, 0)), robotColor);
		}
	};

	IplImage* resizedImage = cvCreateImage(cvSize(500, 500), image->depth, image->nChannels);
	for (size_t i = 0;; i++)
	{
		wheel->OnExecute();
		wheel->GetPosition(position);
		
		cvCopyImage(backGroundImage, image);

		//if (i % 10 == 0)
		{
			//cvSet(image,  CV_RGB(255, 255, 255));
			DrawRobot(image, position);
		}
		
		cvResize(image, resizedImage);

		cvShowImage("Localization", resizedImage);
		cvWaitKey(10);	
	}

	return 0;
}

bool TestWheelController(WheelController& wheel)
{
	PrintMessage("WheelController Test\r\n");

	Property& porperty = GetProperty();

	if(wheel.Initialize(porperty) != API_SUCCESS)
	{
		PrintMessage("Can not initialize WheelController\r\n");
		return false;
	}
	if(wheel.Enable() != API_SUCCESS)
	{
		PrintMessage("Can not initialize WheelController\r\n");
		return false;
	}

#if defined(WIN32)
	CreateThread(NULL, 0, &ThreadHandler, &wheel, 0, NULL);
#else
	pthread_t receiveThread;
	pthread_create(&receiveThread, NULL, &ThreadHandler, &wheel);
#endif

	const double MAX_LINEAR_SPEED = 0.2;
	const double MAX_ANGULAR_SPEED = 20;

	const double ROTATION_DEGREE = 90;
	const double MOVE_DISTANCE = 1;
	
	for (;;)
	{
		PrintMessage("Please input command\r\n");
		char command = getch();

		switch(command)
		{
		case 'w':
			PrintMessage("DriveWheel(%lf, 0.0)\r\n", MAX_LINEAR_SPEED);
			wheel.DriveWheel(MAX_LINEAR_SPEED, 0.0);
			break;
		case 'q':
			PrintMessage("DriveWheel(%lf, %lf)\r\n", MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED);
			wheel.DriveWheel(MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED);
			break;
		case 'x':
			PrintMessage("DriveWheel(%lf, 0.0)\r\n", -MAX_LINEAR_SPEED);
			wheel.DriveWheel(-MAX_LINEAR_SPEED, 0.0);
			break;
		case 'e':
			PrintMessage("DriveWheel(%lf, %lf)\r\n", MAX_LINEAR_SPEED, -MAX_ANGULAR_SPEED);
			wheel.DriveWheel(MAX_LINEAR_SPEED, -MAX_ANGULAR_SPEED);
			break;
		case 's':
			PrintMessage("StopWheel()\r\n");
			wheel.StopWheel();
			break;
		case 'a':
			PrintMessage("RotateWheel(%lf, %lf)\r\n", ROTATION_DEGREE, MAX_ANGULAR_SPEED);
			wheel.RotateWheel(ROTATION_DEGREE, MAX_ANGULAR_SPEED);
			break;
		case 'd':
			PrintMessage("RotateWheel(%lf, %lf)\r\n", -ROTATION_DEGREE, MAX_ANGULAR_SPEED);
			wheel.RotateWheel(-ROTATION_DEGREE, MAX_ANGULAR_SPEED);
			break;
		case 'z':
			PrintMessage("DriveWheel(%lf, %lf)\r\n", -MAX_LINEAR_SPEED, -MAX_ANGULAR_SPEED);
			wheel.DriveWheel(-MAX_LINEAR_SPEED, -MAX_ANGULAR_SPEED);
			break;
		case 'c':
			PrintMessage("DriveWheel(%lf, %lf)\r\n", -MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED);
			wheel.DriveWheel(-MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED);
			break;

			//MoveWheel Thest
		case 'r':
			PrintMessage("MoveWheel(%lf, %lf)\r\n", MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);
			wheel.MoveWheel(MOVE_DISTANCE, MAX_LINEAR_SPEED);
			break;
		case 'v':
			PrintMessage("MoveWheel(%lf, %lf)\r\n", -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);
			wheel.MoveWheel(-MOVE_DISTANCE, MAX_LINEAR_SPEED);
			break;

		case 'f':
			{
				bool isWheelRunning = false;
				wheel.IsWheelRunning(isWheelRunning);
				PrintMessage("Is WheelRunning : %s \r\n", isWheelRunning ? "True" : "False");
			}
			break;
		case 27:
			PrintMessage("Exit\r\n");
			wheel.Disable();
			wheel.Finalize();
			return true;
		default:
			break;
		}
	}

	return true;
}

int main()
{
	Kobot3WheelController wheel;

	TestWheelController(wheel);

	return 0;
}

#endif