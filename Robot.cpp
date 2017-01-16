#include <iostream>
#include <memory>
#include <string>
#include "math.h"
#include "WPILib.h"
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <Joystick.h>
#include <GenericHID.h>
#include <CANTalon.h>
#include <CameraServer.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <thread>
#include <Commands/Command.h>
#include <Commands/Scheduler.h>
#include "GripPipeline.h"
using namespace std;
class Robot: public frc::IterativeRobot {
	XboxController *xbox;
	RobotDrive *drive;
	CameraServer *camera;
	CANTalon *r1;
	CANTalon *r2;
	CANTalon *l1;
	CANTalon *l2;
	SendableChooser<int> chooser;
public:
	int A,B,X,Y,back,start,LB,RB,LS,RS,Arrow,ThrottlePressS,ThrottlePressB,choice; double LT,RT,RX,RY,LX,LY,Throttle;
	std::shared_ptr<NetworkTable> table;
	Robot(){table = NetworkTable::GetTable("GRIP/myContoursReport");}

 static void imageProcess() {
	cs::UsbCamera cameraF = CameraServer::GetInstance()->StartAutomaticCapture();
	cameraF.SetResolution(640, 480);
	cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo();
	cs::CvSource outputStream;
	cv::Mat image;
	grip::GripPipeline *grip;
	cvSink.GrabFrame(image);
	outputStream = CameraServer::GetInstance()->PutVideo("Back Camera: ", 640, 480);
	grip->process(image);

	outputStream = CameraServer::GetInstance()->PutVideo("Processed Image: ", 640, 480);
	outputStream = CameraServer::GetInstance()->PutVideo("Front Camera: ", 640, 480);
	outputStream.PutFrame(image);
}

void RobotInit() {
	    chooser.AddDefault("Front Camera: ", 1);
	   	chooser.AddObject("Processed Image: ", 2);
	   	chooser.AddObject("Back Camera: ", 3);
	   	SmartDashboard::PutData("Auto Modes", &chooser);
		xbox = new XboxController(0);
		l1 = new CANTalon(1); l2 = new CANTalon(2); r1 = new CANTalon(3); r2 = new CANTalon(4);
        l1->Set(0); l2->Set(0); r1->Set(0); r2->Set(0);
		drive = new RobotDrive(l1, l2, r1, r2); drive->SetExpiration(0.5); drive->SetSafetyEnabled(true);
		Throttle = 0.7; //Maximum robot speed as a %
		std::thread VisionThread(imageProcess);
		VisionThread.detach();
		getValues();
		updateDash();
	}

void getValues() { //Get Xbox controller values
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
    	LX = xbox->GetRawAxis(0); if(((LX<0)&&(LX>-0.01))||((LX>0)&&(LX<0.01))){LX = 0;} //Makes sure that value is == 0 when stick
    	LY = xbox->GetRawAxis(1); if(((LY<0)&&(LY>-0.01))||((LY>0)&&(LY<0.01))){LY = 0;} //is not being moved
    	LT = xbox->GetRawAxis(2);
    	RT = xbox->GetRawAxis(3);
    	RX = xbox->GetRawAxis(4); if(((RX<0)&&(RX>-0.01))||((RX>0)&&(RX<0.01))){RX = 0;}
    	RY = xbox->GetRawAxis(5); if(((RY<0)&&(RY>-0.01))||((RY>0)&&(RY<0.01))){RY = 0;}
    	Arrow = xbox->GetPOV();
    }

void updateDash() { //Put new controller values to the dashboard
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
	SmartDashboard::PutNumber("Throttle",Throttle*100);
}

void updateThrottle() {
	if(back==0){ThrottlePressB = 0;}
	if(start==0){ThrottlePressS = 0;}
	if((back==1)&&(Throttle>0.07)&&(ThrottlePressB==0)&&(ThrottlePressS==0)){
		Throttle-=0.05;
		ThrottlePressB=1;
	}
	if((start==1)&&(Throttle<0.97)&&(ThrottlePressB==0)&&(ThrottlePressS==0)){
		Throttle+=0.05;
		ThrottlePressS=1;
	}
}

void processContours() { //Get contour values and do the math
	std::vector<double> arr = table->GetNumberArray("width",llvm::ArrayRef<double>());
	std::vector<double> centerX = table->GetNumberArray("centerX",llvm::ArrayRef<double>());
	double CX,TW,J,BL,R,CL,LL,C;
	//CX/CY = camera resolution
	//L = distance of wall camera can see
	//C = angle of view of camera
	//TW = width of the retro-reflective tape
	//J = distance of camera to wall
	//BL = angle of wall to camera
	//R = ratio of tape sizes
	CX = 640;
	C = 90;
	TW = 5.8;
//	L = CX/((arr[0]+arr[1])/2)*TW;
	R = arr[0]/(arr[0]+arr[1]);
	BL = 180*R; CL = C*R;
	LL = ((CX-((centerX[0] + centerX[1])/2))/((arr[0]+arr[1])/2))*TW;
	J = sin(360-BL-CL)/(sin(CL)/LL);
	// angle from pin: BL-90
	//distance from pin: J
	SmartDashboard::PutNumber("Angle from pin",BL-90);
	SmartDashboard::PutNumber("Distance from pin",J);
}
void AutonomousInit() override {
	}

void AutonomousPeriodic() {
	}

void TeleopInit() {
		drive->SetSafetyEnabled(false);
		Throttle = 0.7;
		ThrottlePressS = 0; ThrottlePressB = 0;
	}

void TeleopPeriodic() {
	    getValues();
	    updateThrottle();
	    updateDash();
	    if(chooser.GetSelected()==2)processContours();
	}

void TestPeriodic(){
	    drive->SetSafetyEnabled(true);
	    getValues();
	    updateThrottle();
	    updateDash();
	 	drive->TankDrive(-LY*Throttle,-RY*Throttle);
	}
};
START_ROBOT_CLASS(Robot)
