#include <iostream>
#include <memory>
#include <string>
#include "AHRS.h"
#include "WPILib.h"
#include "math.h"

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>

class Robot: public frc::IterativeRobot {

	//From NAVX mxp data monitor example

	//end of data monitor example

public:
	Robot() :
			Adrive(DriveLeft0, DriveLeft1, DriveRight0, DriveRight1), Bdrive(
					DriveLeft2, DriveRight2), chooser(), Drivestick(0), DriveLeft0(
					0), DriveLeft1(1), DriveLeft2(2), DriveRight0(3), DriveRight1(
					4), DriveRight2(5), EncoderLeft(0, 1), EncoderRight(2, 3),
			//from data monitor example
			table(NULL), ahrs(NULL), state(0), AutonOverride(), AutoSw1(), AutoSw2(), AutoSw3(), DiIn9(
					9), DiIn8(8), DiIn7(7), AutoVal(), AutoVal0(), AutoVal1(), AutoVal2(), OutputX(), OutputY()
	//end of monitor example
	{

	}

	void RobotInit() {
		AutonOverride = false;
		if (AutonOverride) {
			chooser.AddDefault(autonNameOFF, autonNameOFF);
			chooser.AddObject(autonNameRed1, autonNameRed1);
			chooser.AddObject(autonNameRed2, autonNameRed2);
			chooser.AddObject(autonNameRed3, autonNameRed3);
			chooser.AddObject(autonNameBlue1, autonNameBlue1);
			chooser.AddObject(autonNameBlue2, autonNameBlue2);
			chooser.AddObject(autonNameBlue3, autonNameBlue3);
			frc::SmartDashboard::PutData("Auto Modes2", &chooser);
		}
		EncoderLeft.SetDistancePerPulse(0.0243228675 * 4);
		EncoderRight.SetDistancePerPulse(-0.0243228675 * 4);
		OutputX = 0, OutputY = 0;
	}

	/*
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * GetString line to get the auto name from the text box below the Gyro.
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */

	void AutonomousInit() override {
		state = 1;

		AutonTimer.Reset();
		AutonTimer.Start();
		// Encoder based auton
		EncoderLeft.Reset();
		EncoderRight.Reset();
		// Turn off drive motors
		DriveLeft0.Set(0);
		DriveLeft1.Set(0);
		DriveLeft2.Set(0);
		DriveRight0.Set(0);
		DriveRight1.Set(0);
		DriveRight2.Set(0);
		//Read Auton Switch
		AutoSw1 = DiIn7.Get();
		AutoSw2 = DiIn8.Get();
		AutoSw3 = DiIn9.Get();
		//Set auto value based on auton switch
		if (!AutoSw1)
			AutoVal0 = 1;
		else
			AutoVal0 = 0;
		if (!AutoSw2)
			AutoVal1 = 2;
		else
			AutoVal1 = 0;
		if (!AutoSw3)
			AutoVal2 = 4;
		else
			AutoVal2 = 0;
		AutoVal = AutoVal0 + AutoVal1 + AutoVal2;

		if (AutonOverride) {
			autoSelected = chooser.GetSelected();
			std::cout << "Auto selected: " << autoSelected << std::endl;
		} else {
			switch (AutoVal) {
			case 1:
				autoSelected = autonNameBlue1;
				break;
			case 2:
				autoSelected = autonNameBlue2;
				break;
			case 3:
				autoSelected = autonNameBlue3;
				break;
			case 4:
				autoSelected = autonNameRed1;
				break;
			case 5:
				autoSelected = autonNameRed2;
				break;
			case 6:
				autoSelected = autonNameRed3;
				break;
			default:
				autoSelected = autonNameOFF;
			}
		}
		//from NAVX mxp data monitor example
		table = NetworkTable::GetTable("datatable");
		lw = LiveWindow::GetInstance();
		try {
			/* Communicate w/navX MXP via the MXP SPI Bus.                                       */
			/* Alternatively:  I2C::Port::kMXP, SerialPort::Port::kMXP or SerialPort::Port::kUSB */
			/* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details.   */
			ahrs = new AHRS(SPI::Port::kMXP, 200);
		} catch (std::exception ex) {
			std::string err_string = "Error instantiating navX MXP:  ";
			err_string += ex.what();
			DriverStation::ReportError(err_string.c_str());
		}
		if (ahrs) {
			LiveWindow::GetInstance()->AddSensor("IMU", "Gyro", ahrs);
			//line made by us, it's here because it will get to 90 degrees easier later
			ahrs->Reset();
		}

		//end of data monitor example

		SmartDashboard::PutNumber("current yaw", 0);
		SmartDashboard::PutNumber("yaw error", 0);
		SmartDashboard::PutNumber("error gain", 0);
//		if (autoSelected == autoNameCustom) {
//			// Custom Auto goes here
//		} else {
//			// Default Auto goes here
//		}
	}

	void AutonomousPeriodic() {
		if (autoSelected == autonNameRed1) {
			//Red boiler side code
			SmartDashboard::PutString("autonMode", "Red  1");
			autoRed1();
		} else if (autoSelected == autonNameRed2) {
			//Red center position code
			SmartDashboard::PutString("autonMode", "Red 2");
			autoRed2();
		} else if (autoSelected == autonNameRed3) {
			//Red retreival zone
			SmartDashboard::PutString("autonMode", "Red 3");
			autoRed3();
		} else if (autoSelected == autonNameBlue1) {
			//Blue boiler side auto selection code
			SmartDashboard::PutString("autonMode", "Blue 1");
			autoBlue1();
		} else if (autoSelected == autonNameBlue2) {
			//Blue center code
			SmartDashboard::PutString("autonMode", "Blue 2");
			autoBlue2();
		} else if (autoSelected == autonNameBlue3) {
			//Blue retreival zone
			SmartDashboard::PutString("autonMode", "Blue 3");
			autoBlue3();
		} else {

			//Default Auto goes here
			SmartDashboard::PutString("autonMode", "Off");
			DriveLeft0.Set(0);
			DriveLeft1.Set(0);
			DriveLeft2.Set(0);
			DriveRight0.Set(0);
			DriveRight1.Set(0);
			DriveRight2.Set(0);

		}

		SmartDashboard::PutNumber("DistanceLeft(raw)", EncoderLeft.GetRaw());
		SmartDashboard::PutNumber("DistanceRight(raw)", EncoderRight.GetRaw());
		SmartDashboard::PutNumber("DistanceLeft(Inch)",
				EncoderLeft.GetDistance());
		SmartDashboard::PutNumber("DistanceRight(Inch)",
				EncoderRight.GetDistance());

//		if (autoSelected == autoNameCustom) {
//			// Custom Auto goes here
//		} else {
//			// Default Auto goes here1
//		}
	}

	int forward7ft(double speed, double target) {
		double DistanceLeft = EncoderLeft.GetDistance();
		double SpeedLeft = speed;
		double SpeedRight = speed;
		int done = 0;
		if (abs(DistanceLeft) < abs(target)) {
			// GetDistance is already calibrated to inches.
			DriveLeft0.Set(SpeedLeft * -1);
			DriveLeft1.Set(SpeedLeft * -1);
			DriveRight0.Set(SpeedRight);
			DriveRight1.Set(SpeedRight);

			SmartDashboard::PutNumber("Left Encoder Distance", DistanceLeft);
			done = 0;
		} else {
			// All done with going straight
			DriveLeft0.Set(0);
			DriveLeft1.Set(0);
			DriveRight0.Set(0);
			DriveRight1.Set(0);
			// done going forwards 7ft

			EncoderLeft.Reset();
			EncoderRight.Reset();

			done = 1;
		}
		return done;
	}
	int autonTurn(float targetYaw, float tolerance, float errorGain) {

		// targetYaw is the angle want to go to
		//tolerance is the acceptable error band of destination
		//GetYaw() returns a value between +180 and -180 degrees

		float currentYaw = ahrs->GetAngle();
		float yawError = currentYaw - targetYaw;

		int done = 0;

		SmartDashboard::PutNumber("current yaw", currentYaw);
		SmartDashboard::PutNumber("yaw error", yawError);
		SmartDashboard::PutNumber("error gain", errorGain);

		if (abs(yawError) > tolerance) {
			DriveLeft0.Set(yawError * errorGain);
			DriveLeft1.Set(yawError * errorGain);
			DriveRight0.Set(yawError * errorGain);
			DriveRight1.Set(yawError * errorGain);

			SmartDashboard::PutNumber("motor speed", yawError * errorGain);

			//will continue to turn
			done = 0;

		} else {
			//ahrs->Reset();
			//stops turning and goes to next state
			done = 1;
		}
		return done;
	}

	void autoBlue1A(void) {
		//drives turns then drives again
		//Blue boiler side code
		//blue 1
		//std::string autoSelected = *((std::string*) chooser.GetSelected());
		if (state == 1) {
			// go forward 7 ft
			//rover on carpet:forward7ft(-0.8, 6.5 * 12.0)
			//rover on tile: forward7ft(-0.8, 6.5 *12.0)
			if (forward7ft(-0.8, 6.5 * 12.0) == 1) {
				state = 2;
				ahrs->Reset();
			}
		} else if (state == 2) {
			// turn 90 degrees clockwise
			//rover on carpet: autonTurn(85, 5, -0.012)
			//rover on tile: autonTurn(75, 5, -0.012)
			if (autonTurn(85, 5, -0.012) == 1) {
				state = 3;
				EncoderLeft.Reset();
				EncoderRight.Reset();
			}
		} else if (state == 3) {
			// go forward 7 ft to hit hopper
			//rover on carpet: forward7ft(0.8, 7 * -1 * 12.0)
			//rover on tile: forward7ft(0.8, 7 * -1 * 12.0)
			if (forward7ft(0.8, 7 * -1 * 12.0) == 1) {
				state = 4;
				AutonTimer.Reset();
			}
		} else if (state == 4) {
			//waits a couple of seconds for balls
			//rover on carpet: pause(1, 0.1)
			//rover on tile: pause(1, 0.1)
			if (timedDrive(1, 0.1, 0.1) == 1) {
				state = 5;
			}
		} else if (state == 5) {
			//go backward 4-ish feet
			//rover on carpet: forward(-0.8, 4 * 12.0)
			//rover on tile: forward7ft(-0.8, 4 * 12.0)
			if (forward7ft(-0.8, 4 * 12.0) == 1) {
				state = 6;
				ahrs->Reset();
			}
		} else if (state == 6) {
			// turn enough degrees to face boiler
			//rover on carpet: autonTurn(120, 5, -0.012)
			//rover on tile: autonTurn(105, 5, -0.012)
			if (autonTurn(120, 5, -0.012) == 1) {
				state = 7;
				EncoderLeft.Reset();
				EncoderRight.Reset();
			}
		} else if (state == 7) {
			//go forward 7-ish feet to run into boiler
			//rover on carpet:forward7ft(-0.8, 8.5 * 12.0)
			//rover on tile: forward7ft(-0.8, 8.5 * 12.0)
			if (forward7ft(-0.8, 8.5 * 12.0) == 1) {
				state = 8;
				ahrs->Reset();
			}
		} else if (state == 8) {
			//launch
			state = 9;
		} else if (state == 9) {
			//stop
			stopx();
		} else {
			state = 1;
		}

		SmartDashboard::PutNumber("DistanceLeft(raw)", EncoderLeft.GetRaw());
		SmartDashboard::PutNumber("DistanceRight(raw)", EncoderRight.GetRaw());
		SmartDashboard::PutNumber("DistanceLeft(Inch)",
				EncoderLeft.GetDistance());
		SmartDashboard::PutNumber("DistanceRight(Inch)",
				EncoderRight.GetDistance());
		SmartDashboard::PutNumber("State", state);
		SmartDashboard::PutNumber("Timer", AutonTimer.Get());

	}

	void autoBlue1(void) {
			//drives diagonally toward hopper
			//Blue boiler side code
			//blue 1
			//std::string autoSelected = *((std::string*) chooser.GetSelected());
			if (state == 1) {
				// go forward 8 ft diagonally towards hopper
				//rover on carpet:forward7ft(-0.8, 6.5 * 12.0)
				//rover on tile: forward7ft(-0.8, 6.5 *12.0)
				if (forward7ft(-0.8, 8 * 12.0) == 1) {
					state = 4;
					ahrs->Reset();
				}
			} else if (state == 4) {
				//turns into hopper
				//rover on carpet: pause(1, 0.1)
				//rover on tile: pause(1, 0.1)
				if (timedDrive(1, 0.2, 0.1) == 1) {
					state = 5;
				}
			} else if (state == 5) {
				//go backward 4-ish feet
				//rover on carpet: forward(-0.8, 4 * 12.0)
				//rover on tile: forward7ft(-0.8, 4 * 12.0)
				if (forward7ft(-0.8, 4 * 12.0) == 1) {
					state = 6;
					ahrs->Reset();
				}
			} else if (state == 6) {
				// turn enough degrees to face boiler
				//rover on carpet: autonTurn(120, 5, -0.012)
				//rover on tile: autonTurn(105, 5, -0.012)
				if (autonTurn(120, 5, -0.012) == 1) {
					state = 7;
					EncoderLeft.Reset();
					EncoderRight.Reset();
				}
			} else if (state == 7) {
				//go forward 7-ish feet to run into boiler
				//rover on carpet:forward7ft(-0.8, 8.5 * 12.0)
				//rover on tile: forward7ft(-0.8, 8.5 * 12.0)
				if (forward7ft(-0.8, 8.5 * 12.0) == 1) {
					state = 8;
					ahrs->Reset();
				}
			} else if (state == 8) {
				//launch
				state = 9;
			} else if (state == 9) {
				//stop
				stopx();
			} else {
				state = 1;
			}

			SmartDashboard::PutNumber("DistanceLeft(raw)", EncoderLeft.GetRaw());
			SmartDashboard::PutNumber("DistanceRight(raw)", EncoderRight.GetRaw());
			SmartDashboard::PutNumber("DistanceLeft(Inch)",
					EncoderLeft.GetDistance());
			SmartDashboard::PutNumber("DistanceRight(Inch)",
					EncoderRight.GetDistance());
			SmartDashboard::PutNumber("State", state);
			SmartDashboard::PutNumber("Timer", AutonTimer.Get());

		}

	void autoBlue2(void) {
		if (state == 1) {
			//rover on carpet: forward7ft(-0.8, 7 * 12.0)
			//rover on tile: forward7ft(-0.8, 7 * 12.0)
			if (forward7ft(-0.8, 7 * 12.0) == 1) {
				state = 9;
			}
		} else if (state == 9) {
			stopx();
		} else {
			state = 9;
		}
		return;

	}

	void autoBlue3(void) {
		//blue three
		if (state == 1) {
			// go forward 7 ft
			//rover on carpet: forward 7ft(-0.8, 7 * 12.0)
			//rover on tile: forward7ft(-0.8, 7 * 12.0)
			if (forward7ft(-0.8, 7 * 12.0) == 1) {
				state = 2;
				ahrs->Reset();
			}
		} else if (state == 2) {
			// turn 90 degrees counterclockwise
			//rover on carpet: autonTurn(-60, 5, -0.012)
			//rover on tile: autonTurn(-60, 5, -0.012)
			if (autonTurn(-60, 5, -0.012) == 1) {
				state = 3;
				EncoderLeft.Reset();
				EncoderRight.Reset();
			}
		} else if (state == 3) {
			// go forward
			//rover on carpet: forward7ft(-0.6, 2 * 12.0)
			//rover on tile: forward7ft(-0.5, 2 * 12)
			if (forward7ft(-0.6, 2 * 12) == 1) {
				state = 9;
				EncoderLeft.Reset();
				EncoderRight.Reset();
			}
		} else if (state == 9) {
			stopx();
		} else {
			state = 9;
		}
		return;

	}

	void autoRed1A(void) {
		//Red center position code
		//this version turns the robot in a right angle
		SmartDashboard::PutString("autonMode", "Red 1");
		//***red 1
		//std::string autoSelected = *((std::string*) chooser.GetSelected());

		if (state == 1) {
			// go forward 7 ft
			//rover on carpet: forward7ft(-0.8, 6.5 * 12.0)
			//rover on tile: forward7ft(-0.8, 6.5 * 12.0)
			if (forward7ft(-0.8, 6.5 * 12.0) == 1) {
				state = 2;
				ahrs->Reset();
			}
		} else if (state == 2) {
			// turn 90 degrees counterclockwise
			//rover on carpet: autonTurn(-95, 5, -0.012)
			//rover on tile: autonTurn(-82, 5, -0.012)
			if (autonTurn(-95, 5, -0.012) == 1) {
				state = 3;
				EncoderLeft.Reset();
				EncoderRight.Reset();
			}
		} else if (state == 3) {
			// go forward 7 ft to hit hopper
			//rover on carpet: forward7ft(0.8, 7 * -1 * 12.0)
			//rover on tile: forward7ft(0.8, 7 * 12.0)
			if (forward7ft(0.8, 7 * -1 * 12.0) == 1) {
				state = 4;
				AutonTimer.Reset();
			}
		} else if (state == 4) {
			//waits in front of hopper a couple of seconds for balls
			//rover on carpet: pause(1, 0)
			//rover on tile: pause(1, 0)
			if (timedDrive(1, 0.1, 0.1) == 1) {
				state = 5;
			}
		} else if (state == 5) {
			//go backward 3-ish feet
			//rover on carpet: forward7ft(-0.8, 3 * 12.0)
			//rover on tile:forward7ft(-0.8, 3 * 12.0)
			if (forward7ft(-0.8, 3 * 12.0) == 1) {
				state = 6;
				ahrs->Reset();
			}
		} else if (state == 6) {
			// turns counterclockwise enough degrees to face boiler
			//rover on carpet: autonTurn(-125, 5, -0.012)
			//rover on tile: autonTurn(-115, 5, -0.012)
			if (autonTurn(-125, 5, -0.012) == 1) {
				state = 7;
				EncoderLeft.Reset();
				EncoderRight.Reset();
			}
		} else if (state == 7) {
			//go forward 7-ish feet to run into boiler
			//rover on carpet: forward7ft(-0.8, 8.5 * 12.0)
			//rover on tile: forward7ft(-0.8, 8.5*12.0)
			if (forward7ft(-0.8, 8.5 * 12.0) == 1) {
				state = 8;
				ahrs->Reset();
			}
		} else if (state == 8) {
			//launch
			state = 9;
		} else if (state == 9) {
			//stop
			stopx();
		} else {
			state = 1;
		}

		SmartDashboard::PutNumber("DistanceLeft(raw)", EncoderLeft.GetRaw());
		SmartDashboard::PutNumber("DistanceRight(raw)", EncoderRight.GetRaw());
		SmartDashboard::PutNumber("DistanceLeft(Inch)",
				EncoderLeft.GetDistance());
		SmartDashboard::PutNumber("DistanceRight(Inch)",
				EncoderRight.GetDistance());
		SmartDashboard::PutNumber("State", state);
		return;

	}

	void autoRed1(void) {
		//Red center position code
		//this version drive diagonally to the hopper
		SmartDashboard::PutString("autonMode", "Red 1");
		//***red 1
		//std::string autoSelected = *((std::string*) chooser.GetSelected());

		if (state == 1) {
			// go forward 7 ft
			//rover on carpet: forward7ft(-0.8, 6.5 * 12.0)
			//rover on tile: forward7ft(-0.8, 6.5 * 12.0)
			if (forward7ft(-0.8, 8 * 12.0) == 1) {
				//goes directly to state 4 for consistency with autoRed1A
				state = 4;
				ahrs->Reset();
			}
		} else if (state == 4) {
			//robot drives counterclockwise into hopper
			//rover on carpet: pause(1, 0) 		// changed from pause -> timedDrive()
			//rover on tile: pause(1, 0)
			if (timedDrive(1, 0.1, 0.2) == 1) {
				state = 5;
			}
		} else if (state == 5) {
			//go backward 3-ish feet
			//rover on carpet: forward7ft(-0.8, 3 * 12.0)
			//rover on tile:forward7ft(-0.8, 3 * 12.0)
			if (forward7ft(-0.8, 3 * 12.0) == 1) {
				state = 6;
				ahrs->Reset();
			}
		} else if (state == 6) {
			// turns counterclockwise enough degrees to face boiler
			//rover on carpet: autonTurn(-125, 5, -0.012)
			//rover on tile: autonTurn(-115, 5, -0.012)
			if (autonTurn(-125, 5, -0.012) == 1) {
				state = 7;
				EncoderLeft.Reset();
				EncoderRight.Reset();
			}
		} else if (state == 7) {
			//go forward 7-ish feet to run into boiler
			//rover on carpet: forward7ft(-0.8, 8.5 * 12.0)
			//rover on tile: forward7ft(-0.8, 8.5*12.0)
			if (forward7ft(-0.8, 8.5 * 12.0) == 1) {
				state = 8;
				ahrs->Reset();
			}
		} else if (state == 8) {
			//launch
			state = 9;
		} else if (state == 9) {
			//stop
			stopx();
		} else {
			state = 1;
		}

		SmartDashboard::PutNumber("DistanceLeft(raw)", EncoderLeft.GetRaw());
		SmartDashboard::PutNumber("DistanceRight(raw)", EncoderRight.GetRaw());
		SmartDashboard::PutNumber("DistanceLeft(Inch)",
				EncoderLeft.GetDistance());
		SmartDashboard::PutNumber("DistanceRight(Inch)",
				EncoderRight.GetDistance());
		SmartDashboard::PutNumber("State", state);
		return;

	}

	void autoRed2(void) {

		if (state == 1) {
			//rover on carpet: forward7ft(-0.8, 7 * 12.0)
			//rover on tile: forward7ft(-0.8, 7 * 12.0)
			if (forward7ft(-0.8, 7 * 12.0) == 1) {
				state = 9;
			}
		} else if (state == 9) {
			stopx();
		} else {
			state = 9;
		}

		return;
	}

	void autoRed3(void) {
		//red three
		if (state == 1) {
			// go forward 7 ft
			//rover on carpet: forward7ft(-0.8, 9 * 12.0)
			//rover on tile: forward 7ft(-0.8, 9 * 12.0)
			if (forward7ft(-0.8, 9 * 12.0) == 1) {
				state = 2;
				ahrs->Reset();
			}
		} else if (state == 2) {
			// turn 60 degrees clockwise
			//rover on carpet: autonTurn(60, 5, -0.019)
			//rover on tile:autonTurn(60, 5, -0.012)
			if (autonTurn(60, 5, -0.019) == 1) {
				state = 3;
				EncoderLeft.Reset();
				EncoderRight.Reset();
			}
		} else if (state == 3) {
			// go forward
			//rover on carpet: forward7ft(-0.6, 2 * 12)
			//rover on tile: forward7ft(-0.5, 2 * 12.0)
			if (forward7ft(-0.6, 2 * 12) == 1) {
				state = 9;
				EncoderLeft.Reset();
				EncoderRight.Reset();
			}
		} else if (state == 9) {
			stopx();
		} else {
			state = 9;
		}

		return;

	}

	int timedDrive(double driveTime, double leftMotorSpeed,
			double rightMotorSpeed) {
		int done;
		float currentTime = AutonTimer.Get();
		if (currentTime < driveTime) {
			DriveLeft0.Set(-leftMotorSpeed);
			DriveLeft1.Set(-leftMotorSpeed);
			DriveRight0.Set(rightMotorSpeed);
			DriveRight0.Set(rightMotorSpeed);

			done = 0;
		} else {
			done = 1;
		}
		return done;
	}

//	void autonTurn(float targetYaw, float tolerance, float errorGain) {
//
//		// targetYaw is the angle want to go to
//		//tolerance is the acceptable error band of destination
//		//GetYaw() returns a value between +180 and -180 degrees
//
//		float currentYaw = ahrs->GetAngle();
//		float yawError = currentYaw - targetYaw;
//		float errorGain = 0.005;
//
//		SmartDashboard::PutNumber("current yaw", currentYaw);
//		SmartDashboard::PutNumber("yaw error", yawError);
//		SmartDashboard::PutNumber("error gain", errorGain);
//
//		if (abs(yawError) > tolerance) {
//			DriveLeft0.Set(yawError * errorGain);
//			DriveLeft1.Set(yawError * errorGain);
//			DriveRight0.Set(yawError * errorGain);
//			DriveRight1.Set(yawError * errorGain);
//
//			SmartDashboard::PutNumber("motor speed", yawError * errorGain);
//		} else {
//			state = 3;
//			ahrs->Reset();
//		}
//	}
	int stopx() {
		DriveLeft0.Set(0);
		DriveLeft1.Set(0);
		DriveRight0.Set(0);
		DriveRight1.Set(0);
		//Go to next state turn left 90 degrees
		return 0;
	}

	void TeleopInit() {
		Adrive.SetSafetyEnabled(true);
		Bdrive.SetSafetyEnabled(true);
		OutputX = 0, OutputY = 0;
	}

	void TeleopPeriodic() {

		double SpeedLeft = Drivestick.GetRawAxis(1) * -1; // get Yaxis value (forward)
		double SpeedRight = Drivestick.GetRawAxis(4); // get Xaxis value (turn)
		// Set dead band for X and Y axis
		float Deadband = 0.11;
		if (SpeedLeft < Deadband and SpeedLeft > -Deadband)
			SpeedLeft = 0;
		if (SpeedRight < Deadband and SpeedRight > -Deadband)
			SpeedRight = 0;
		//Reduce turn speed left trigger is pressed
		float LdTrig = Drivestick.GetRawAxis(2); //Read left drive trigger
		float MaxSpeed = 0.5;				 // Set reduced speed
		if (LdTrig > 0.1) {
			SpeedLeft = SpeedLeft * MaxSpeed;  // Reduce turn speed
			SpeedRight = SpeedRight * MaxSpeed;  // Reduce drive speed
		}
		//slow down direction changes from 1 cycle to 5

		OutputY = (0.8 * OutputY) + (0.2 * SpeedLeft);
		OutputX = (0.8 * OutputX) + (0.2 * SpeedRight);
		// Drive Robot Arcade style
		Adrive.ArcadeDrive(OutputY, OutputX, true);
		Bdrive.ArcadeDrive(OutputY, OutputX, true);

		double DistanceLeft = EncoderLeft.GetRaw();
		double DistanceRight = EncoderRight.GetRaw();
		SmartDashboard::PutNumber("DistanceLeft(raw)", DistanceLeft);
		SmartDashboard::PutNumber("DistanceRight(raw)", DistanceRight);
		SmartDashboard::PutNumber("DistanceLeft(Inch)",
				EncoderLeft.GetDistance());
		SmartDashboard::PutNumber("DistanceRight(Inch)",
				EncoderRight.GetDistance());

	}

	void TestPeriodic() {
		lw->Run();
	}

private:
	RobotDrive Adrive, Bdrive;
	frc::LiveWindow* lw = LiveWindow::GetInstance();
	frc::SendableChooser<std::string> chooser;
	const std::string autonNameOFF = "0 OFF";
	const std::string autonNameBlue1 = "Blue 1";
	const std::string autonNameBlue2 = "Blue 2";
	const std::string autonNameBlue3 = "Blue 3";
	const std::string autonNameRed1 = "Red 1";
	const std::string autonNameRed2 = "Red 2";
	const std::string autonNameRed3 = "Red 3";
	std::string autoSelected;
	Joystick Drivestick;
	VictorSP DriveLeft0;
	VictorSP DriveLeft1;
	VictorSP DriveLeft2;
	VictorSP DriveRight0;
	VictorSP DriveRight1;
	VictorSP DriveRight2;
	Timer AutonTimer;
	Encoder EncoderLeft;
	Encoder EncoderRight;
	std::shared_ptr<NetworkTable> table;
	AHRS *ahrs;
	int state;bool AutonOverride, AutoSw1, AutoSw2, AutoSw3;
	DigitalInput DiIn9, DiIn8, DiIn7;
	int AutoVal, AutoVal0, AutoVal1, AutoVal2;
	float OutputX, OutputY;

};

START_ROBOT_CLASS(Robot)
