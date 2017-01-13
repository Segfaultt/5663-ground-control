#include <iostream>
#include <math.h>
#include <memory>
#include <string>
#include "WPILib.h"
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <Joystick.h>
#include <GenericHID.h>
#include <CANTalon.h>
#include "GripPipeline.h"

//calculate distance from lift in meters assuming you're right on
//k is a constant found by measuring the distance and area
double find_distance(int area, const int k)
{
	return k*sqrt(area);
}

//find the centreline where the pin is from a greyscale matrix
int average_x_coordinate(cv::Mat& matrix)
{
	unsigned long cummulative_value = 0, n_of_values = 0;

	for(int i = 0; i < matrix.rows; ++i) {
		const unsigned char* row = matrix.ptr<unsigned char>(i);
		for(int j = 0; j < matrix.cols; j++) {
			if (row[j] == 255) {
				cummulative_value += j;
				n_of_values++;
			}
		}
	}

	return cummulative_value/n_of_values;
}

//find how many white pixels there are right or left of midpoint
int sum_of_side(cv::Mat& matrix, int midpoint, bool lhs)
{
	unsigned long cummulative_value = 0;

	if (lhs) {
		for(int i = 0; i < matrix.rows; ++i) {
			const unsigned char* row = matrix.ptr<unsigned char>(i); 
			for(int j = 0; j < midpoint; j++) {
				if (row[j] == 255) {
					cummulative_value++;
				}
			}
		}
	} else {
		
		for(int i = 0; i < matrix.rows; ++i) {
			const unsigned char* row = matrix.ptr<unsigned char>(i); 
			for(int j = midpoint; j < matrix.cols; j++) {
				if (row[j] == 255) {
					cummulative_value++;
				}
			}
		}
	}

	return cummulative_value;
}

void vision(const double thresh, const double max_val)
{
	//set up camera input (may need to be changed)
	cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture();
	camera.SetResolution(640, 480);
	cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo();

	cv::Mat image;
	grip::GripPipeline grip;

	while(true) {
		cvSink.GrabFrame(image);

		//put image processing here
		grip.process(image);
	}
}
class Robot: public frc::IterativeRobot {
	frc::XboxController *xbox;
	frc::RobotDrive *drive;
	frc::LiveWindow* lw = LiveWindow::GetInstance();
	frc::SendableChooser<std::string> chooser;
	CANTalon *r1;
	CANTalon *r2;
	CANTalon *l1;
	CANTalon *l2;
	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "My Auto";
	std::string autoSelected;

	public:
	int A,B,X,Y,back,start,LB,RB,LS,RS,Arrow,ThrottlePressS,ThrottlePressB; double LT,RT,RX,RY,LX,LY,Throttle;
	void RobotInit() {
		chooser.AddDefault(autoNameDefault, autoNameDefault);
		chooser.AddObject(autoNameCustom, autoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &chooser);
		xbox = new XboxController(0);
		l1 = new CANTalon(1);
		l2 = new CANTalon(2);
		r1 = new CANTalon(3);
		r2 = new CANTalon(4);
		l1->Set(0);
		l2->Set(0);
		r1->Set(0);
		r2->Set(0);
		drive = new RobotDrive(l1, l2, r1, r2);
		drive->SetExpiration(0.5);
		drive->SetSafetyEnabled(true);
		Throttle = 0.7; //Maximum robot speed as a %

		//create a thread for vision
		std::thread visionThread(vision, 200, 255);//thresh = 200, max value = 255
		visionThread.detach();
	}

	void getControllerValues() {
		A = xbox->GetRawButton(1);
		B = xbox->GetRawButton(2);
		X = xbox->GetRawButton(3);
		Y = xbox->GetRawButton(4);
		LB = xbox->GetRawButton(5);
		RB = xbox->GetRawButton(6);
		back = xbox->GetRawButton(7);
		start = xbox->GetRawButton(8);
		LS = xbox->GetRawButton(9);
		RS = xbox->GetRawButton(10);
		LX = xbox->GetRawAxis(0);
		LY = xbox->GetRawAxis(1);
		LT = xbox->GetRawAxis(2);
		RT = xbox->GetRawAxis(3);
		RX = xbox->GetRawAxis(4);
		RY = xbox->GetRawAxis(5);
		Arrow = xbox->GetPOV();
	}

	void updateControllerValues() {
		SmartDashboard::PutNumber("A",A);
		SmartDashboard::PutNumber("B",B);
		SmartDashboard::PutNumber("X",X);
		SmartDashboard::PutNumber("Y",Y);
		SmartDashboard::PutNumber("Left Bumper",LB);
		SmartDashboard::PutNumber("Right Bumper",RB);
		SmartDashboard::PutNumber("Start",start);
		SmartDashboard::PutNumber("Back",back);
		SmartDashboard::PutNumber("Right Stick",RS);
		SmartDashboard::PutNumber("Left Stick",LS);
		SmartDashboard::PutNumber("Right Stick X",RX);
		SmartDashboard::PutNumber("Right Stick Y",RY);
		SmartDashboard::PutNumber("Left Stick X",LX);
		SmartDashboard::PutNumber("Left Stick Y",LY);
		SmartDashboard::PutNumber("Right Trigger",RT);
		SmartDashboard::PutNumber("Left Trigger",LT);
		SmartDashboard::PutNumber("Arrow Pad",Arrow);
		SmartDashboard::PutNumber("Throttle",Throttle);
	}

	void updateThrottle() {
		if (back==0)
			ThrottlePressB = 0;
		if (start==0)
			ThrottlePressS = 0;
		if (back == 1 && Throttle > 0 && ThrottlePressB == 0 && ThrottlePressS == 0) {
			Throttle-=0.05;
			ThrottlePressB=1;
		}
		if (start==1 && Throttle > 0 && ThrottlePressB == 0 && ThrottlePressS == 0) {
			Throttle+=0.05;
			ThrottlePressS=1;
		}
	}

	void AutonomousInit() override {
		autoSelected = chooser.GetSelected();
		//std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
		std::cout << "Auto selected: " << autoSelected << std::endl;

		if (autoSelected == autoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	}

	void AutonomousPeriodic() {
		if (autoSelected == autoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	}

	void TeleopInit() {
		drive->SetSafetyEnabled(true);
		Throttle = 0.7;
		ThrottlePressS = 0; ThrottlePressB = 0;

	}

	void TeleopPeriodic() {
		getControllerValues();
		updateThrottle();
		updateControllerValues();
		drive->TankDrive(-LY*Throttle*abs(Throttle),-RY*Throttle*abs(Throttle));
	}

	void TestPeriodic(){
	 	drive->SetSafetyEnabled(false);
		getControllerValues();
		updateControllerValues();
	}
};

START_ROBOT_CLASS(Robot)
