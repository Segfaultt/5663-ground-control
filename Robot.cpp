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
	clock_t t;
	frc::PowerDistributionPanel *pdp = new PowerDistributionPanel(0);
	XboxController *xbox = new XboxController(0), *xbox2 = new XboxController(1);
	Talon *left_motors = new Talon(0), *right_motors = new Talon(1), *climber = new Talon(2), *loader = new Talon(3),
			*gear_motor = new Talon(4);
	DoubleSolenoid *rightGear = new DoubleSolenoid(1, 0, 1),
			*eject = new DoubleSolenoid(1, 2, 3);
	Compressor *c = new Compressor(1);
	DigitalInput *IRsensor = new DigitalInput(0), *IRsensor2 = new DigitalInput(1);
	frc::DigitalOutput *one = new DigitalOutput(2), *two = new DigitalOutput(3), *three = new DigitalOutput(4);
	SendableChooser<int*> *AutoChooser = new SendableChooser<int*>,
			*ControllerMode = new SendableChooser<int*>;

public:
	std::shared_ptr<NetworkTable> table;
	int LS, RS, gearToggle, ejectToggle = 0, Auto, Control, fastGear,
			slowGear, mult, state;
	double LT, RT, RX, RY, LX, LY, speedL, speedR, rate, dist, center, throttle; bool LB, RB, back, start, A, B, X, Y, up, down,
	left, right;

	void RobotInit() {
		NetworkTable::SetTeam(5663);
		NetworkTable::SetIPAddress("10.56.63.2");
		table = NetworkTable::GetTable("LiftTracker");
		CameraServer::GetInstance()->StartAutomaticCapture();
		AutoChooser->AddObject("Reach Base Line", (int*) 0);
		AutoChooser->AddDefault("Forward", (int*) 1);
		AutoChooser->AddObject("Left Peg", (int*) 2);
		AutoChooser->AddObject("Do Nothing", (int*) 4);
		AutoChooser->AddObject("Do Nothing", (int*) 4);
		SmartDashboard::PutData("Choose Mode", AutoChooser);

		ControllerMode->AddDefault("Two Controller", (int*) 2);
		ControllerMode->AddObject("One Controller", (int*) 1);
		SmartDashboard::PutData("Control Mode", ControllerMode);

		left_motors->Set(0);
		right_motors->Set(0);
		climber->Set(0);
		loader->Set(0);
		gear_motor->Set(0);
		slowGear = rightGear->kReverse;
		fastGear = rightGear->kForward;
		eject->Set(eject->kReverse);
		getValues();
		updateDash();
	}

	void getValues() { //Get values
		dist = table->GetNumber("distanceFromTarget", 1234567890);
		center = table->GetNumber("center", 1234567890);
		Auto = (int) AutoChooser->GetSelected();
		Control = (int) ControllerMode->GetSelected();
		double zone = 0.06;
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
		switch (xbox2->GetPOV()) {
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
		SmartDashboard::PutNumber("distance", dist);
		SmartDashboard::PutNumber("center", center);
		if (rightGear->Get() == slowGear) {
			SmartDashboard::PutBoolean("", true);
			SmartDashboard::PutString(" ", "SLOW");
		} else {
			SmartDashboard::PutBoolean("", false);
			SmartDashboard::PutString(" ", "FAST");
		}
		if(290 < center && center < 350) {
			SmartDashboard::PutString("Loader Is Aligned?", "In center");
		} else if(center < 290){
			SmartDashboard::PutString("Loader Is Aligned?", "Too far left");
		} else {
			SmartDashboard::PutString("Loader Is Aligned?", "Too far right");
		}
	}

	void setGear(int gear) {
		if (gear == rightGear->kForward) {
			rightGear->Set(rightGear->kForward);
		}
		if (gear == rightGear->kReverse) {
			rightGear->Set(rightGear->kReverse);
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

	void changeEject(bool i) {
		if (i && ejectToggle == 1) {
			if (eject->Get() == eject->kForward) {
				eject->Set(eject->kReverse);
				ejectToggle = 0;
			} else {
				eject->Set(eject->kForward);
				ejectToggle = 0;
			}
		}
		if (!i) {
			ejectToggle = 1;
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
		if(pdp->GetVoltage() < 9.5) {
			throttle = (pdp->GetVoltage() / 10) - 0.15;
		}
		else if(pdp->GetVoltage() < 6) {
			throttle = 0;
		} else {
			throttle = 1;
		}
		l *= abs(l);
		r *= abs(r);
		left_motors->Set(-l*throttle);
		right_motors->Set(r*throttle);
	}

	void moveClimber(bool u) {
		if (u) {
			rate = 1;
			SmartDashboard::PutBoolean("Climbing:", true);
		} else {
			rate = 0;
			SmartDashboard::PutBoolean("Climbing:", false);
		}
		climber->Set(rate);
	}

	void moveLoader(double l, double r) {
		if (l > r)
			loader->Set(-l);
		else
			loader->Set(r);
	}

	bool time(double aa) {
		if (((((float) (clock() - t))) / CLOCKS_PER_SEC) < aa) {
			return true;
		} else {
			return false;
		}
	}

	bool alignLoader(int lower, int upper, double speed) {
		if (center > upper && center != 1234567890) {
			SmartDashboard::PutString("Auto State", "Aligning Loader - Moving Right");
			moveLoader(speed, 0);
			return false;
		}
		else if (center < lower) {
			SmartDashboard::PutString("Auto State", "Aligning Loader - Moving Left");
			moveLoader(0, speed);
			return false;
		} else if (center == 1234567890) {
			return false;
		}
		else {
			SmartDashboard::PutString("Auto State", "Aligning Loader - In Center");
			loader->Set(0);
			return true;
		}
	}

	bool alignRobot(int lower, int upper, int distance) {
		if (center > upper && center != 1234567890) {
			SmartDashboard::PutString("Auto State", "Aligning - Turning Right");
			tankDrive(0.35, -0.35);
			mult = 1;
			return false;
		}
		else if (center < lower) {
			SmartDashboard::PutString("Auto State", "Aligning - Turning Left");
			tankDrive(-0.35, 0.35);
			mult = -1;
			return false;
		}
		else if (center == 1234567890) {
			if (mult == -1)
				SmartDashboard::PutString("Auto State", "Lost Target - Turning Left");
			if (mult == 1)
				SmartDashboard::PutString("Auto State", "Lost Target - Turning Right");
			tankDrive(mult * 0.4, -mult * 0.4);
			return false;
		}
		else if ((center > lower && center < upper) && dist > distance){
			SmartDashboard::PutString("Auto State", "In Center - Driving Forward");
			tankDrive(0.507, 0.5);
			return false;
		}
		else {
			SmartDashboard::PutString("Auto State", "Close To Target - Stopping");
			tankDrive(0,0);
			return true;
		}
	}

	void AutonomousInit() override {
		c->SetClosedLoopControl(false);
		setGear(slowGear);
		tankDrive(0, 0);
		climber->Set(0);
		loader->Set(0);
		gear_motor->Set(0);
		eject->Set(eject->kReverse);
		getValues();
		updateDash();
		mult = 1;
		t = clock();
		state = 0;
	}

	void AutonomousPeriodic() {
		getValues();
		updateDash();
		SmartDashboard::PutNumber("Auto", Auto);
		SmartDashboard::PutNumber("state:",state);
		std::cout << state << endl;
		SmartDashboard::PutNumber("timer", ((((float) (clock() - t))) / CLOCKS_PER_SEC));
		//#############
		if (Auto == 0) { // baseline
			if(state == 0) {
				if(time(4)){
					SmartDashboard::PutString("Auto State", "Driving Forward");
					tankDrive(0.5,0.5);
				} else {
					SmartDashboard::PutString("Auto State", "Reached Baseline - Stopping");
					tankDrive(0,0);
				}
			}
		}
		//#############
		if(Auto == 1) { // forward
			if (state == 0) {
				if (time(0.75)) {
					tankDrive(0.5575,0.55);
				} else {
					state = 3;
					tankDrive(0,0);
				}
			}
			if (state == 1) {
				if (alignRobot(280,360,60)) {
					state = 3;
				}
			}
			if (state == 3) {
				if (alignLoader(310,330,0.65)) {
					state = 4; t = clock();
				}
			}
			if (state == 4) {
				if (time(1)) {
					SmartDashboard::PutString("Auto State", "Forward Slowly");
					tankDrive(0.4085, 0.4);
				} else {
					tankDrive(0, 0);
					eject->Set(eject->kForward);
					SmartDashboard::PutString("Auto State", "Done");
				}
			}
		}
		if(Auto == 2) { // left
			if (state == 0) {
				if(time(0.97)){
					tankDrive(0.5,0.5);
				} else {
					tankDrive(0,0);
					state = 1;
					mult = 1;
				}
			}
			if (state == 1) {
				if(alignRobot(280,360,60)) {
					state = 3;
					t = clock();
				}
			}
			if (state == 3) {
				if (alignLoader(310,330,0.6)) {
					state = 4; t = clock();
				}
			}
			if (state == 4) {
					if (time(0.5)) {
						SmartDashboard::PutString("Auto State", "Going Forward");
						tankDrive(0.35, 0.38);
					} else {
						tankDrive(0, 0);
						SmartDashboard::PutString("Auto State", "Done");
						state = -1;
						t = clock();
					}
			}
			if (state == 5) {
				if (time(3)) {
					SmartDashboard::PutString("Auto State", "Hit Wall - Driving Backwards");
					tankDrive(-0.45, -0.45);
				} else {
					SmartDashboard::PutString("Auto State", "Restarting Auto");
					tankDrive(0,0); state = 1;
				}
			}
		}
	}

	void TeleopInit() {
		setGear(fastGear);
		gearToggle = 0;
		gear_motor->Set(0);
		c->SetClosedLoopControl(false);
	}
	/*
	 * Controls: Controller 1 -
	 * Right Stick = Right side speed
	 * Left Stick = Left side speed
	 * Hold Left Bumper = Both sides controlled by right stick
	 * Y button or Right bumper = Change gear
	 * Right Trigger = right side slow
	 * Left Trigger = left side slow
	 *
	 * Controller 2 -
	 * A button = climber
	 * B button = climber slow
	 * Left Bumper and Right Bumper = move gear
	 * Hold X button = auto align loader
	 * left trigger and right trigger = move loader
	 */
	void TeleopPeriodic() {
		getValues();
		if(LB){tankDrive(-speedR,-speedR);}
		else {tankDrive(-speedL, -speedR);}
		changeGear(RB || Y);
		if (Control == 2) { //Two controllers
			moveGear(xbox2->GetBumper(xbox2->kLeftHand), xbox2->GetBumper(xbox2->kRightHand));
			moveClimber(xbox2->GetAButton());

			if(xbox2->GetXButton() && dist > 40) {
				alignLoader(290,350,0.6);
			}
			else {
				moveLoader(xbox2->GetTriggerAxis(xbox2->kLeftHand), xbox2->GetTriggerAxis(xbox2->kRightHand));
			}

			changeEject(xbox2->GetYButton());
		}

		if (Control == 1) { //One controller
			moveGear(left, right);
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
		gear_motor->Set(0);
		setGear(slowGear);
	}
	void TestPeriodic() {
	}
};
START_ROBOT_CLASS(Robot)
