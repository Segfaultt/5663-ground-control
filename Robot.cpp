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
using namespace std;
class Robot: public frc::IterativeRobot {
	XboxController *xbox;
	RobotDrive *drive;
	CameraServer *camera;
	CANTalon *r1;
	CANTalon *r2;
	CANTalon *r3;
	CANTalon *l1;
	CANTalon *l2;
	CANTalon *l3;
	SendableChooser<int> *chooser;
	DoubleSolenoid *rightGear;
	DoubleSolenoid *leftGear;
	Compressor *c = new Compressor(1);

public:
	int LS, RS, Arrow, choice, gearToggle;
	double LT, RT, RX, RY, LX, LY, speedL, speedR;bool enabled, down, up, left,
			right, LB, RB, back, start, A, B, X, Y;
	void RobotInit() {
		c->SetClosedLoopControl(true);
		chooser->AddDefault("Mode 1", 1);
		chooser->AddObject("Drive", 2);
		SmartDashboard::PutData("Modes", chooser);
		xbox = new XboxController(0);
		l1 = new CANTalon(2);
		l2 = new CANTalon(3);
		r1 = new CANTalon(4);
		r2 = new CANTalon(5);
		l1->Set(0);
		l2->Set(0);
		r1->Set(0);
		r2->Set(0);
		drive = new RobotDrive(l1, l2, r1, r2);
		drive->SetExpiration(0.5);
		drive->SetSafetyEnabled(false);
		rightGear = new DoubleSolenoid(0, 1);
		leftGear = new DoubleSolenoid(2, 3);
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
		LX = xbox->GetRawAxis(0);
		if (((LX < 0) && (LX > -0.01)) || ((LX > 0) && (LX < 0.01))) {
			LX = 0;
		} //Makes sure that value is == 0 when stick is not being moved
		LY = xbox->GetRawAxis(1);
		if (((LY < 0) && (LY > -0.01)) || ((LY > 0) && (LY < 0.01))) {
			LY = 0;
		}
		LT = xbox->GetRawAxis(2);
		RT = xbox->GetRawAxis(3);
		RX = xbox->GetRawAxis(4);
		if (((RX < 0) && (RX > -0.01)) || ((RX > 0) && (RX < 0.01))) {
			RX = 0;
		}
		RY = xbox->GetRawAxis(5);
		if (((RY < 0) && (RY > -0.01)) || ((RY > 0) && (RY < 0.01))) {
			RY = 0;
		}
		Arrow = xbox->GetPOV();
		up = Arrow == 0;
		right = Arrow == 90;
		down = Arrow == 180;
		left = Arrow == 270;
		enabled = c->Enabled();
		choice = chooser->GetSelected();
		speedL = LT * 0.5 + LY;
		if (speedL > 1) {
			speedL = 1;
		}
		speedR = RT * 0.5 + RY;
		if (speedR > 1) {
			speedR = 1;
		}
	}

	void updateDash() { //Put new controller values to the dashboard
		SmartDashboard::PutNumber("A", A);
		SmartDashboard::PutNumber("B", B);
		SmartDashboard::PutNumber("X", X);
		SmartDashboard::PutNumber("Y", Y);
		SmartDashboard::PutNumber("Left Bumper", LB);
		SmartDashboard::PutNumber("Right Bumper", RB);
		SmartDashboard::PutNumber("Start", start);
		SmartDashboard::PutNumber("Back", back);
		SmartDashboard::PutNumber("Right Stick", RS);
		SmartDashboard::PutNumber("Left Stick", LS);
		SmartDashboard::PutNumber("Right Stick X", RX);
		SmartDashboard::PutNumber("Right Speed", speedR);
		SmartDashboard::PutNumber("Left Stick X", LX);
		SmartDashboard::PutNumber("Left Speed", speedL);
		SmartDashboard::PutNumber("Right Trigger", RT);
		SmartDashboard::PutNumber("Left Trigger", LT);
		SmartDashboard::PutNumber("Arrow Pad", Arrow);
		SmartDashboard::PutNumber("Compressor:", enabled);
	}

	void changeGear(bool i) {
		if (i == 1 && gearToggle == 0) {
			gearToggle = 1;
		}
		if (i == 0 && gearToggle == 1) {
			rightGear->Set(rightGear->kReverse);
			leftGear->Set(leftGear->kReverse);
			gearToggle = 2;
		}
		if (i == 1 && gearToggle == 2) {
			gearToggle = 3;
		}
		if (i == 0 && gearToggle == 3) {
			rightGear->Set(rightGear->kForward);
			leftGear->Set(leftGear->kForward);
			gearToggle = 0;
		}
	}
//void processContours() { //Get contour values and do the math
//	std::vector<double> arr = table->GetNumberArray("width",llvm::ArrayRef<double>());
//	std::vector<double> centerX = table->GetNumberArray("centerX",llvm::ArrayRef<double>());
//	double CX,TW,J,BL,R,CL,LL,C,XX;
//	//CX/CY = camera resolution
//	//L = distance of wall camera can see
//	//C = angle of view of camera
//	//TW = width of the retro-reflective tape
//	//J = distance of camera to wall
//	//BL = angle of wall to camera
//	//R = ratio of tape sizes
//	CX = 640;
//	C = 90;
//	TW = 5.8;
////	L = CX/((arr[0]+arr[1])/2)*TW;
//	R = arr[0]/(arr[0]+arr[1]);
//	BL = 180*R; CL = C*R;
//	XX = (CX-((centerX[0] + centerX[1])/2));
//	LL = (XX / ((arr[0]+arr[1])/2))*TW;
//	J = sin(360-BL-CL)/(sin(CL)/LL);
//	SmartDashboard::PutNumber("Angle from pin",BL-90);
//	SmartDashboard::PutNumber("Distance from pin",J);
//	SmartDashboard::PutNumber("Width, Contour 1",arr[0]);
//	SmartDashboard::PutNumber("Width, Contour 2",arr[1]);
//	SmartDashboard::PutNumber("Center X",XX);
//}

	void AutonomousInit() override {
	}

	void AutonomousPeriodic() {
	}

	void TeleopInit() {
		drive->SetSafetyEnabled(true);
		rightGear->Set(rightGear->kReverse);
		leftGear->Set(leftGear->kReverse);
		gearToggle = 0;
	}

	void TeleopPeriodic() {
		getValues();
		updateDash();
		changeGear(Y);
		if (right || left) {
			//rotate gear
		}
		if (down || up) {
			// climber
		}
		if (RB || LB) {
			// move gear loader
		}
		if (choice == 1) {
			drive->TankDrive(0.0, 0.0);
		}
		if (choice == 2) {
			drive->TankDrive(speedL, speedR);
		}

	}

	void TestPeriodic() {
	}
};
START_ROBOT_CLASS(Robot)
