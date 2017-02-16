#include <iostream>
#include <string>
#include <memory>
#include <string>
#include "math.h"
#include "WPILib.h"
#include <DigitalInput.h>
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
#include <I2C.h>
#include "C:/Users/jakel/workspace/5663.0.0/src/AHRS.h"
#include "ADIS16448_IMU.h"
#include <time.h>
using namespace std;
class Robot: public frc::IterativeRobot {
	clock_t time; ADIS16448_IMU *imu = new ADIS16448_IMU;
	XboxController *xbox = new XboxController(0), *xbox2 = new XboxController(1);
    Talon *left_motors = new Talon(0), *right_motors = new Talon(1), *climber = new Talon(2), *loader = new Talon(3);
	DoubleSolenoid *rightGear = new DoubleSolenoid(1, 0, 1), *leftGear = new DoubleSolenoid(1, 2, 3);
	Compressor *c = new Compressor(1);
	Servo *servo = new Servo(9);
	DigitalInput *IRsensor = new DigitalInput(0), *IRsensor2 = new DigitalInput(1);
	SendableChooser<int*> *AutoChooser = new SendableChooser<int*>, *ControllerMode = new SendableChooser<int*>;
public:
	std::shared_ptr<NetworkTable> table;
	int LS, RS, gearToggle, servoAngle, Auto, Control, defaultGear, fastGear, slowGear, angle;
	double LT, RT, RX, RY, LX, LY, speedL, speedR, rate, dist, offset, accel, velocity;
	bool LB, RB, back, start, A, B, X, Y, up, down, left, right, IR, IR2;

	void RobotInit() {
		NetworkTable::SetTeam(5663); NetworkTable::SetIPAddress("10.56.63.2"); table = NetworkTable::GetTable("LiftTracker");

		AutoChooser->AddDefault("Do nothing",(int*)1);
		AutoChooser->AddObject("Drive forward 1 meter",(int*)2);
		AutoChooser->AddObject("Drive forward 4 meters",(int*)3);
		SmartDashboard::PutData("Choose Mode", AutoChooser);

		ControllerMode->AddDefault("One Controller", (int*)1);
		ControllerMode->AddObject("Two Controller", (int*)2);
		SmartDashboard::PutData("Control Mode", ControllerMode);

		SmartDashboard::PutBoolean("Squared Input?",true);
		SmartDashboard::PutBoolean("Dead Zones?",true);

		left_motors->Set(0); right_motors->Set(0); climber->Set(0); loader->Set(0);
		fastGear = rightGear->kForward; slowGear = rightGear->kReverse;

		getValues();
		updateDash();
	}

	void getValues() { //Get values
		table->GetNumber("distanceFromTarget",0); table->GetNumber("angleFromGoal",0); table->GetNumberArray("centerX",0.0);
		Auto = (int)AutoChooser->GetSelected(); Control = (int)ControllerMode->GetSelected();
		IR = IRsensor->Get(); 					IR2 = IRsensor2->Get();
		A = xbox->GetRawButton(1); 				B = xbox->GetRawButton(2);
		X = xbox->GetRawButton(3); 				Y = xbox->GetRawButton(4);
		LB = xbox->GetRawButton(5);			    RB = xbox->GetRawButton(6);
		back = xbox->GetRawButton(7); 			start = xbox->GetRawButton(8);
		LS = xbox->GetRawButton(9); 			RS = xbox->GetRawButton(10);
		LX = xbox->GetRawAxis(0); 				if (LX > -0.05 && LX < 0.05) LX = 0;
		LY = xbox->GetRawAxis(1); 				if (LY > -0.05 && LY < 0.05) LY = 0;
		LT = xbox->GetRawAxis(2);				RT = xbox->GetRawAxis(3);
		RX = xbox->GetRawAxis(4);				if (RX > -0.05 && RX < 0.05) RX = 0;
		RY = xbox->GetRawAxis(5);				if (RY > -0.05 && RY < 0.05) RY = 0;
		switch (xbox->GetPOV()) {
		case 0: up = true; break;
		case 45: up = true; right = true; break;
		case 315: up = true; left = true; break;
		case 90: right = true; break;
		case 180: down = true; break;
		case 225: left = true; down = true; break;
		case 135: right = true; down = true; break;
		case 270: left = true; break;
		default: up = false; left = false; right = false; down = false; break;
		}
		speedR = (-RT * 0.5) + RY; if (speedR > 1) speedR = 1;
		speedL = (-LT * 0.5) + LY; if (speedL > 1) speedL = 1;
	}

	void updateDash() { //Put new controller values to the dashboard
		SmartDashboard::PutBoolean("A", A);                   SmartDashboard::PutBoolean("B", B);
		SmartDashboard::PutBoolean("X", X);                   SmartDashboard::PutBoolean("Y", Y);
		SmartDashboard::PutBoolean("Left Bumper", LB);        SmartDashboard::PutBoolean("Right Bumper", RB);
		SmartDashboard::PutBoolean("Back", back);             SmartDashboard::PutBoolean("Start", start);
		SmartDashboard::PutBoolean("Left Stick", LS);         SmartDashboard::PutBoolean("Right Stick", RS);
		SmartDashboard::PutNumber("Left Stick X", LX);        SmartDashboard::PutNumber("Right Stick X", RX);
		SmartDashboard::PutNumber("Left Speed", speedL);      SmartDashboard::PutNumber("Right Speed", speedR);
		SmartDashboard::PutNumber("Left Trigger", LT);        SmartDashboard::PutNumber("Right Trigger", RT);
		SmartDashboard::PutBoolean("IR2",IR2);                SmartDashboard::PutBoolean("IR1",IR);
		SmartDashboard::PutNumber("Servo Angle", servoAngle);
		SmartDashboard::PutNumber("distance",table->GetNumber("distanceFromTarget",0));
		SmartDashboard::PutNumber("angle", table->GetNumber("angleFromGoal",0));
		SmartDashboard::PutNumberArray("center x",table->GetNumberArray("centerX",0.0));
		SmartDashboard::PutNumber("Acceleration Y", imu->GetAccelY());
		if(rightGear->Get() == slowGear) {SmartDashboard::PutBoolean("", true); SmartDashboard::PutString(" ","SLOW");}
		else {SmartDashboard::PutBoolean("", false); SmartDashboard::PutString(" ","FAST");}
	}

	void setGear(int gear) {
		if(gear == fastGear) {
			rightGear->Set(rightGear->kForward); leftGear->Set(leftGear->kForward);
		}
		if(gear == slowGear) {
			rightGear->Set(rightGear->kReverse); leftGear->Set(leftGear->kReverse);
		}
	}

    void changeGear(bool i) {
		if (i && gearToggle == 1) {
			if (rightGear->Get() == slowGear) { setGear(fastGear); gearToggle = 0; }
			else { setGear(slowGear); gearToggle = 0; }
		}
		if(!i) gearToggle = 1;
	}

	void moveServo(bool min, bool add) {
		if (!add && servoAngle < 170) servoAngle += 10;
		if (!min && servoAngle > 9) servoAngle -= 10;
		servo->SetAngle(servoAngle);
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

	void tankDrive(double l,double r) {
		if(!SmartDashboard::GetBoolean("Dead Zones?",true)) { l = xbox->GetRawAxis(1); r = xbox->GetRawAxis(5);}
		if(SmartDashboard::GetBoolean("Squared Input?", true)) { l*=abs(l); r*=abs(r); }
		left_motors->Set(l); right_motors->Set(-r);
	}

	void moveClimber(bool u) {
		if(u) rate += 0.01;
		else rate = 0;
		climber->Set(rate);
	}

	void moveLoader(double l, double r) {
		if(l > r) loader->Set(-l);
		else loader->Set(r);
	}

	void turnDrive(int a) {

	}

	void sendLED(char send) {
		//Wire->Write(4,send);
	}

	void AutonomousInit() override {
		c->SetClosedLoopControl(false);
		setGear(slowGear);
		dist = 0; angle = 0;
		imu->Calibrate();
		offset = imu->GetAccelY();
		time = clock();
	}

	void AutonomousPeriodic() {
		if(dist<4) {
			time = clock()-time;
			accel = imu->GetAccelY()-offset;
			velocity = accel * ((float)time)/CLOCKS_PER_SEC;
			dist += velocity * ((float)time)/CLOCKS_PER_SEC;
			tankDrive(0.5,0.5);
		} else tankDrive(0,0);
	}

	void TeleopInit() {
		setGear(fastGear);
		gearToggle = 0; servoAngle = 0;
		c->SetClosedLoopControl(false);
		rate = 0; dist = 0; angle = 0;
		imu->Calibrate(); offset = imu->GetAccelY();
		time = clock();
	}

	void TeleopPeriodic() {
		getValues();
		tankDrive(speedL,speedR);
		changeGear(RS);
		if(Control == 1) { //One controller
			moveServo(IR,IR2);
			moveClimber(A || up);
			moveLoader(left, right);
		}
		if(Control == 2) { //Two controllers
			moveServo((IR || xbox2->GetBumper(xbox2->kLeftHand)),(IR2 || xbox2->GetBumper(xbox2->kRightHand)));
			moveClimber(xbox2->GetAButton() || up);
			moveLoader(xbox2->GetTriggerAxis(xbox2->kLeftHand), xbox2->GetTriggerAxis(xbox2->kRightHand));
		}
		updateDash();
	}

	void TestPeriodic() {
		c->SetClosedLoopControl(true);
		tankDrive(0,0);
		climber->Set(0);
		loader->Set(0);
	}
};
START_ROBOT_CLASS(Robot)
