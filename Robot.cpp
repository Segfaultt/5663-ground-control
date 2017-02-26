#include <iostream>
#include <string>
#include <memory>
#include <string>
#include "math.h"
#include "WPILib.h"
#include <DigitalInput.h>
#include <IterativeRobot.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <Joystick.h>
#include <GenericHID.h>
#include <CANTalon.h>
#include <CameraServer.h>
#include <Commands/Command.h>
#include <Commands/Scheduler.h>
#include <I2C.h>
#include <time.h>
using namespace std;
class Robot: public frc::IterativeRobot {
	clock_t time;
	frc::PowerDistributionPanel *pdp = new PowerDistributionPanel(0);
	XboxController *xbox = new XboxController(0), *xbox2 = new XboxController(1);
	Talon *left_motors = new Talon(0), *right_motors = new Talon(1), *climber = new Talon(2), *loader = new Talon(3),
			*gear_motor = new Talon(4);
	DoubleSolenoid *rightGear = new DoubleSolenoid(1, 0, 1), *leftGear = new DoubleSolenoid(1, 2, 3);
	Compressor *c = new Compressor(1);
	DigitalInput *IRsensor = new DigitalInput(0), *IRsensor2 = new DigitalInput(1);
	frc::DigitalOutput *one = new DigitalOutput(2), *two = new DigitalOutput(3), *three = new DigitalOutput(4);
	SendableChooser<int*> *AutoChooser = new SendableChooser<int*>,
			*ControllerMode = new SendableChooser<int*>;

public:
	std::shared_ptr<NetworkTable> table;
	int LS, RS, gearToggle, Auto, Control, fastGear,
			slowGear, mult, state;
	string auto_state;
	double LT, RT, RX, RY, LX, LY, speedL, speedR, rate, dist, center; bool LB, RB, back, start, A, B, X, Y, up, down,
			left, right, IR, IR2;

	void RobotInit() {
		NetworkTable::SetTeam(5663);
		NetworkTable::SetIPAddress("10.56.63.2");
		table = NetworkTable::GetTable("LiftTracker");

		AutoChooser->AddObject("Nothing", (int*) 0);
		AutoChooser->AddDefault("Forward", (int*) 1);
		AutoChooser->AddObject("Left (drive forward test)", (int*) 2);
		AutoChooser->AddObject("Right", (int*) 3);
		SmartDashboard::PutData("Choose Mode", AutoChooser);

		ControllerMode->AddDefault("Two Controller", (int*) 2);
		ControllerMode->AddObject("One Controller", (int*) 1);
		SmartDashboard::PutData("Control Mode", ControllerMode);

		SmartDashboard::PutBoolean("IR sensors?", true);

		left_motors->Set(0);
		right_motors->Set(0);
		climber->Set(0);
		loader->Set(0);
		gear_motor->Set(0);
		slowGear = rightGear->kReverse;
		fastGear = rightGear->kForward;

		getValues();
		updateDash();
	}

	void getValues() { //Get values
		dist = table->GetNumber("distanceFromTarget", 1234567890);
		center = table->GetNumber("center", 1234567890);
		Auto = (int) AutoChooser->GetSelected();
		Control = (int) ControllerMode->GetSelected();
		double zone = 0.06;
		IR = !IRsensor->Get();
		IR2 = !IRsensor2->Get();
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
		if (LX > -zone && LX < zone)
			LX = 0;
		LY = xbox->GetRawAxis(1);
		if (LY > -zone && LY < zone)
			LY = 0;
		LT = xbox->GetRawAxis(2);
		RT = xbox->GetRawAxis(3);
		RX = xbox->GetRawAxis(4);
		if (RX > -zone && RX < zone)
			RX = 0;
		RY = xbox->GetRawAxis(5);
		if (RY > -zone && RY < zone)
			RY = 0;
		switch (xbox->GetPOV()) {
		case 0:
			up = true;
			break;
		case 45:
			up = true;
			right = true;
			break;
		case 315:
			up = true;
			left = true;
			break;
		case 90:
			right = true;
			break;
		case 180:
			down = true;
			break;
		case 225:
			left = true;
			down = true;
			break;
		case 135:
			right = true;
			down = true;
			break;
		case 270:
			left = true;
			break;
		default:
			up = false;
			left = false;
			right = false;
			down = false;
			break;
		}
		if (Control == 2)
			speedR = (-RT * 0.5) + RY;
		else
			speedR = RY;
		if (speedR > 1)
			speedR = 1;
		if (Control == 2)
			speedL = (-LT * 0.5) + LY;
		else
			speedL = LY;
		if (speedL > 1)
			speedL = 1;
	}

	void updateDash() { //Put new controller values to the dashboard
		SmartDashboard::PutNumber("arrows 2", xbox2->GetPOV());
		SmartDashboard::PutNumber("Amp", pdp->GetTotalCurrent());
		SmartDashboard::PutBoolean("A", A);
		SmartDashboard::PutBoolean("B", B);
		SmartDashboard::PutBoolean("X", X);
		SmartDashboard::PutBoolean("Y", Y);
		SmartDashboard::PutBoolean("Left Bumper", LB);
		SmartDashboard::PutBoolean("Right Bumper", RB);
		SmartDashboard::PutBoolean("Back", back);
		SmartDashboard::PutBoolean("Start", start);
		SmartDashboard::PutBoolean("Left Stick", LS);
		SmartDashboard::PutBoolean("Right Stick", RS);
		SmartDashboard::PutNumber("Left Stick X", LX);
		SmartDashboard::PutNumber("Right Stick X", RX);
		SmartDashboard::PutNumber("Left Speed", speedL);
		SmartDashboard::PutNumber("Right Speed", speedR);
		SmartDashboard::PutNumber("Left Trigger", LT);
		SmartDashboard::PutNumber("Right Trigger", RT);
		SmartDashboard::PutBoolean("IR2", IR2);
		SmartDashboard::PutBoolean("IR1", IR);
		SmartDashboard::PutNumber("distance", dist);
		SmartDashboard::PutNumber("center", center);
		SmartDashboard::PutString("Auto state", auto_state);
		if (rightGear->Get() == slowGear) {
			SmartDashboard::PutBoolean("", true);
			SmartDashboard::PutString(" ", "SLOW");
		} else {
			SmartDashboard::PutBoolean("", false);
			SmartDashboard::PutString(" ", "FAST");
		}
	}

	void sendLED(bool L, bool E, bool D) {
		one->Set(L); two->Set(E); three->Set(D);
	}
	/*
	 * Codes:
	 * 0,0,0 = off
	 * 1,1,1 = rocket mode
	 * 1,1,0 = launch rocket
	 * 1,0,0 = red
	 * 0,1,0 = green
	 * 0,0,1 = blue
	 * 0,1,1 = yellow
	 *
	 */
	void setGear(int gear) {
		if (gear == rightGear->kForward) {
			rightGear->Set(rightGear->kForward);
			leftGear->Set(leftGear->kForward);
		}
		if (gear == rightGear->kReverse) {
			rightGear->Set(rightGear->kReverse);
			leftGear->Set(leftGear->kReverse);
		}
	}

	void changeGear(bool i) {
		if (i && gearToggle == 1) {
			if (rightGear->Get() == fastGear) {
				setGear(slowGear);
				gearToggle = 0;
			} else {
				setGear(fastGear);
				gearToggle = 0;
			}
		}
		if (!i) {
			gearToggle = 1;
		}
	}

	void moveGear(bool min, bool add) {
		if (add)
			gear_motor->Set(0.6);
		else if (min)
			gear_motor->Set(-0.6);
		else
			gear_motor->Set(0);
	}

	void tankDrive(double l, double r) {
		l *= abs(l);
		r *= abs(r);
		left_motors->Set(-l);
		right_motors->Set(r);
	}

	void moveClimber(bool u) {
		if (u)
			rate += 0.015;
		else
			rate = 0;
		climber->Set(rate);
	}

	void moveLoader(double l, double r) {
		if (l > r)
			loader->Set(-l);
		else
			loader->Set(r);
	}

	void alignLoader(int lower, int upper, double speed) {
		if (center > upper && center != 1234567890)
			moveLoader(0, speed);
		else if (center < lower)
			moveLoader(speed, 0);
		else
			loader->Set(0);
	}

	void alignRobot(int lower, int upper) {
		if (center > upper) {
			auto_state = "Left";
			tankDrive(0.3, -0.3);
			mult = -1;
		}
		if (center < lower) {
			auto_state = "Right";
			tankDrive(-0.3, 0.3);
			mult = 1;
		}
		if (center == 1234567890) {
			if (mult == -1)
				auto_state = "Error - Going Right";
			if (mult == 1)
				auto_state = "Error - Going Left";
			tankDrive(mult * 0.5, -mult * 0.5);
		}
	}

	void AutonomousInit() override {
		c->SetClosedLoopControl(false);
		setGear(slowGear);
		tankDrive(0, 0);
		climber->Set(0);
		loader->Set(0);
		getValues();
		updateDash();
		mult = 1;
		time = clock();
		state = 0;
	}

	void AutonomousPeriodic() {
		getValues();
		updateDash();
		SmartDashboard::PutNumber("state:",state);
		cout << state << endl;
		if (Auto == 0) state = 0;
		//#############
		if(Auto == 1) {
			if (state == 0) {
				if ((dist > 60) && (center > 280 && center < 360)) {
					auto_state = "forward";
					tankDrive(0.5, 0.53);
				}
				if ((dist < 60) && (center > 280 && center < 360)) {
					auto_state = "stop";
					tankDrive(0, 0);
					state = 1;
					time = clock();
				}
				alignRobot(280,360);
			}
			if (state == 1) {
				alignRobot(290,350);
				if (center < 350 && center > 290) {
					state = 2;
					tankDrive(0,0);
				}
			}
			if (state == 2) {
				tankDrive(0, 0);
				alignLoader(310,330,0.6);
				if(center < 330 && center > 310) {
					loader->Set(0);
					state = 3; time = clock();
				}
			}
			if (state == 3) {
				if (pdp->GetTotalCurrent() < 4.2) {
					if (((((float) (clock() - time))) / CLOCKS_PER_SEC) < 1.5) {
						tankDrive(0.35, 0.38);
					} else {
						tankDrive(0, 0);
						auto_state = "Gear on peg";
						state = -1;
						time = clock();
					}
				} else { tankDrive(0,0); state = 4; time = clock();}
			}
			if (state == 4) {
				if (((((float) (clock() - time))) / CLOCKS_PER_SEC) < 3) {
					tankDrive(-0.45, -0.45);
				} else {
					tankDrive(0,0); state = 0;
				}
			}
		}
		//#################
		if(Auto == 2) {
			//Drive forward test
			tankDrive(0.5,0.5);
		}
	}

	void TeleopInit() {
		setGear(fastGear);
		gearToggle = 0;
		c->SetClosedLoopControl(false);
	}

	void TeleopPeriodic() {
		getValues();
		if(LB) tankDrive(-speedR,-speedR);
		else tankDrive(-speedL, -speedR);
		changeGear(RB || Y);

		if (Control == 2) { //Two controllers

			if(SmartDashboard::GetBoolean("IR sensors?", true)) {
			moveGear((xbox2->GetBumper(xbox2->kLeftHand) || IR), (xbox2->GetBumper(xbox2->kRightHand) || IR2));
			} else {
				moveGear(xbox2->GetBumper(xbox2->kLeftHand), xbox2->GetBumper(xbox2->kRightHand));
			}

			moveClimber( xbox2->GetAButton() || xbox2->GetPOV() == 0 || xbox2->GetPOV() == 315 || xbox2->GetPOV() == 45);

			if(xbox2->GetXButton() && dist > 40)
				alignLoader(280,360,0.6);
			else
				moveLoader(xbox2->GetTriggerAxis(xbox2->kLeftHand), xbox2->GetTriggerAxis(xbox2->kRightHand));
		}

		if (Control == 1) { //One controller
			moveGear(left || IR, right || IR2);
			moveClimber(A);
			if (LT != 0 || RT != 0) moveLoader(LT, RT);
			else if (X && dist > 40) {
				alignLoader(280,360,0.6);
			} else
				loader->Set(0);
		}
		updateDash();
	}

	void TestInit() {
		c->SetClosedLoopControl(true);
		tankDrive(0, 0);
		climber->Set(0);
		loader->Set(0);
	}
	void TestPeriodic() {
//		c->SetClosedLoopControl(true);
//		tankDrive(0, 0);
//		climber->Set(0);
//		loader->Set(0);
	}
};
START_ROBOT_CLASS(Robot)
