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
		GRIPTable = NetworkTable::GetTable("GRIP/myContuorsReport");
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
			frc::SmartDashboard::PutData("Auto Modes", &chooser);
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
		isWaiting = 0;

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

		SmartDashboard::PutString("autonMode", "Off");

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
			case 14:
				autoSelected = autonNameBlue1;
				break;
			case 13:
				autoSelected = autonNameBlue2;
				break;
			case 12:
				autoSelected = autonNameBlue3;
				break;
			case 11:
				autoSelected = autonNameRed1;
				break;
			case 10:
				autoSelected = autonNameRed2;
				break;
			case 9:
				autoSelected = autonNameRed3;
				break;
			case 8:
				autoSelected = autonNameBlue2;
				break;
			case 7:
				autoSelected = autonNameRed2;
				break;
			default:
				autoSelected = autonNameOFF;
			}
		}
		//from NAVX mxp data monitor example
		ahrs->Reset();
		ahrs->ZeroYaw();
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
	}

	void TeleopInit() {
		Adrive.SetSafetyEnabled(true);
		Bdrive.SetSafetyEnabled(true);
		OutputX = 0, OutputY = 0;
	}


	void DisabledPeriodic() {
		SmartDashboardSenser();
	}
	void AutonomousPeriodic() {
		if (autoSelected == autonNameRed1)
			autoRed1();
		else if (autoSelected == autonNameRed2)
			autoRed2();
		else if (autoSelected == autonNameRed3)
			autoRed3();
		else if (autoSelected == autonNameBlue1)
			autoBlue1();
		else if (autoSelected == autonNameBlue2)
			autoBlue2();
		else if (autoSelected == autonNameBlue3)
			autoBlue3();
		else
			stopx();

		SmartDashboardSenser();
	}

	void TeleopPeriodic() {
		float Deadband = 0.11;
		float MaxSpeed = 0.5;

		double SpeedLinear = Drivestick.GetRawAxis(1) * 1; // get Yaxis value (forward)
		double SpeedRotate = Drivestick.GetRawAxis(4) * -1; // get Xaxis value (turn)

		// Set dead band for X and Y axis
		if (SpeedLinear < Deadband and SpeedLinear > -Deadband)
			SpeedLinear = 0;
		if (SpeedRotate < Deadband and SpeedRotate > -Deadband)
			SpeedRotate = 0;

		//Reduce turn speed left trigger is pressed
		float LdTrig = Drivestick.GetRawAxis(2);	//Read left drive trigger
		// Set reduced speed
		if (LdTrig > 0.1) {
			SpeedLinear = SpeedLinear * MaxSpeed;  // Reduce turn speed
			SpeedRotate = SpeedRotate * MaxSpeed;  // Reduce drive speed
		}
		//slow down direction changes from 1 cycle to 5

		OutputY = (0.8 * OutputY) + (0.2 * SpeedLinear);
		OutputX = (0.8 * OutputX) + (0.2 * SpeedRotate);
		// Drive Robot Arcade style
		Adrive.ArcadeDrive(OutputY, OutputX, true);
		Bdrive.ArcadeDrive(OutputY, OutputX, true);

		SmartDashboardSenser();

	}

	void autoBlue1(void) {

		//Blue boiler side code
		//drives turns then drives again
		switch (state) {
		case 1:
			// go forward 7 ft
			if (forward(6.5 * 12.0)) {
				//rover on carpet:forward7ft(-0.8, 6.5 * 12.0)
				//rover on tile: forward7ft(-0.8, 6.5 *12.0)
				//bob on tile:forward(6.5 * 12.0, 0.2, 0.3, 0.16)
				state = 2;
				ahrs->ZeroYaw();
			}
			break;
		case 2:
			// turn 90 degrees clockwise
			if (autonTurn(85)) {
				//rover on carpet: autonTurn(85, 5, -0.012)
				//rover on tile: autonTurn(75, 5, -0.012)
				//bob on tile: autonTurn(85, 10, -0.009)
				state = 3;
				EncoderLeft.Reset();
				EncoderRight.Reset();
				ahrs->ZeroYaw();
			}
			break;
		case 3:
			// go forward 7 ft to hit hopper
			if (forward(7.6 * -1 * 12.0)) {
				//rover on carpet: forward7ft(0.8, 7 * -1 * 12.0)
				//rover on tile: forward7ft(0.8, 7 * -1 * 12.0)
				//bob on tile: forward7ft(0.4, 7 * -1 * 12.0)
				//bob on tile:forward(7.6 * -1 * 12.0, 0.2, 0.3, 0.16)
				state = 4;
				AutonTimer.Reset();
			}
			break;
		case 4:
			//waits a couple of seconds for balls
			if (timedDrive(2, 0.1, 0.1)) {
				//rover on carpet: pause(1, 0.1)
				//rover on tile: pause(1, 0.1)
				//bob on tile: timedDrive(2, 0.1, 0.1)
				state = 5;
				EncoderLeft.Reset();
				EncoderRight.Reset();
				ahrs->ZeroYaw();
			}
			break;
		case 5:
			//go backward 4-ish feet
			if (forward(2.5 * 12.0)) {
				//rover on carpet: forward(-0.8, 4 * 12.0)
				//rover on tile: forward7ft(-0.8, 4 * 12.0)
				//bob on tile: forward7ft(-0.4, 4*12.0)
				//bob on tile: forward(2.5 * 12.0, 0.2, 0.3, 0.16)
				state = 6;
				//ahrs->Reset();
				ahrs->ZeroYaw();
			}
			break;
		case 6:
			// turn enough degrees to face boiler
			if (autonTurn(120)) {
				//rover on carpet: autonTurn(120, 5, -0.012)
				//rover on tile: autonTurn(105, 5, -0.012)
				//bob on tile: autonTurn(120, 5, -0.009)
				state = 7;
				EncoderLeft.Reset();
				EncoderRight.Reset();
				ahrs->ZeroYaw();
			}
			break;
		case 7:
			//go forward 7-ish feet to run into boiler
			if (forward(8.5 * 12.0)) {
				//rover on carpet:forward7ft(-0.8, 8.5 * 12.0)
				//rover on tile: forward7ft(-0.8, 8.5 * 12.0)
				//bob on tile: forward7ft(-0.4, 8.5 * 12.0)
				//bob on tile: forward(8.5 * 12.0, 0.2, 0.3, 0.16)
				state = 8;
				//ahrs->Reset();
				ahrs->ZeroYaw();
			}
			break;
		case 8:
			//launch
			state = 9;
			break;
		default:
			stopx();
		}
		return;
	}

	void autoBlue1A(void) {

		//Blue boiler side code
		//drives diagonally toward hopper
		switch (state) {
		case 1:
			// go forward 8 ft diagonally towards hopper
			if (forward(7 * 12.0)) {
				//rover on carpet:forward7ft(-0.8, 6.5 * 12.0)
				//rover on tile: forward7ft(-0.8, 6.5 *12.0)
				state = 2;
				//ahrs->Reset();
				ahrs->ZeroYaw();
			}
			break;
		case 2:
			//turns into hopper
			if (timedDrive(3, 0.2, -0.8)) {
				//rover on carpet: pause(1, 0.1)
				//rover on tile: pause(1, 0.1)
				state = 3;
			}
			break;
		case 3:
			//go backward 4-ish feet
			if (forward(4 * 12.0)) {
				//rover on carpet: forward(-0.8, 4 * 12.0)
				//rover on tile: forward7ft(-0.8, 4 * 12.0)
				state = 4;
				//ahrs->Reset();
				ahrs->ZeroYaw();
			}
			break;
		case 4:
			// turn enough degrees to face boiler
			if (autonTurn(120)) {
				//rover on carpet: autonTurn(120, 5, -0.012)
				//rover on tile: autonTurn(105, 5, -0.012)
				state = 5;
				EncoderLeft.Reset();
				EncoderRight.Reset();
			}
			break;
		case 5:
			//go forward 7-ish feet to run into boiler
			if (forward(8.5 * 12.0)) {
				//rover on carpet:forward7ft(-0.8, 8.5 * 12.0)
				//rover on tile: forward7ft(-0.8, 8.5 * 12.0)
				state = 8;
				//ahrs->Reset();
				ahrs->ZeroYaw();
			}
			break;
		case 6:
			//launch
			state = 9;
			break;
		default:
			stopx();
		}

		return;

	}

	void autoBlue2(void) {
		//blue side code
		//goes forward to put gear on pin

		switch (state) {
		case 1:
			//ahrs->Reset();
			ahrs->ZeroYaw();
			state = 2;
			break;

		case 2:
			if (forward(4 * 12)) {
				//gain = 0.05 is too small
				//rover on carpet: forward7ft(-0.8, 7 * 12.0)
				//rover on tile: forward7ft(-0.8, 7 * 12.0)
				//if (forward7ft(-0.4, 7 * 12.0)) {
				state = 3;
			}
			break;

		default:
			stopx();

		}
		return;
	}

	void autoBlue3(void) {
		//blue side code
		//puts gear on pin on side of airship

		switch (state) {
		case 1:
			// go forward 7 ft
			if (forward(7 * 12.0)) {
				//rover on carpet: forward 7ft(-0.8, 7 * 12.0)
				//rover on tile: forward7ft(-0.8, 7 * 12.0)
				state = 2;
				//ahrs->Reset();
				ahrs->ZeroYaw();
			}
			break;

		case 2:
			// turn 90 degrees counterclockwise
			if (autonTurn(-60)) {
				//rover on carpet: autonTurn(-60, 5, -0.012)
				//rover on tile: autonTurn(-60, 5, -0.012)
				state = 3;
				EncoderLeft.Reset();
				EncoderRight.Reset();
			}
			break;
		case 3:
			// go forward
			if (forward(2 * 12)) {
				//rover on carpet: forward7ft(-0.6, 2 * 12.0)
				//rover on tile: forward7ft(-0.5, 2 * 12)
				state = 9;
				EncoderLeft.Reset();
				EncoderRight.Reset();
			}
			break;
		default:
			stopx();
		}
		return;
	}

	void autoRed1(void) {
		//Red center position code
		//this version turns the robot in a right angle

		switch (state) {
		case 1:
			// go forward 7 ft
			if (forward(6.5 * 12.0)) {
				//rover on carpet: forward7ft(-0.8, 6.5 * 12.0)
				//rover on tile: forward7ft(-0.8, 6.5 * 12.0)
				state = 2;
				//ahrs->Reset();
				ahrs->ZeroYaw();
			}
			break;
		case 2:
			// turn 90 degrees counterclockwise
			if (autonTurn(-95) ) {
				//rover on carpet: autonTurn(-95, 5, -0.012)
				//rover on tile: autonTurn(-82, 5, -0.012)
				state = 3;
				EncoderLeft.Reset();
				EncoderRight.Reset();
			}
			break;
		case 3:
			// go forward 7 ft to hit hopper
			if (forward(7 * -1 * 12.0) ) {
				//rover on carpet: forward7ft(0.8, 7 * -1 * 12.0)
				//rover on tile: forward7ft(0.8, 7 * 12.0)
				state = 4;
				AutonTimer.Reset();
			}
			break;
		case 4:
			//waits in front of hopper a couple of seconds for balls
			if (timedDrive(1, 0.05, 0.05) ) {
				//rover on carpet: pause(1, 0)
				//rover on tile: pause(1, 0)
				state = 5;
			}
			break;
		case 5:
			//go backward 3-ish feet
			if (forward(3 * 12.0) ) {
				//rover on carpet: forward7ft(-0.8, 3 * 12.0)
				//rover on tile:forward7ft(-0.8, 3 * 12.0)
				state = 6;
				//ahrs->Reset();
				ahrs->ZeroYaw();
			}
			break;
		case 6:
			// turns counterclockwise enough degrees to face boiler
			if (autonTurn(-125) ) {
				//rover on carpet: autonTurn(-125, 5, -0.012)
				//rover on tile: autonTurn(-115, 5, -0.012)
				state = 7;
				EncoderLeft.Reset();
				EncoderRight.Reset();
			}
			break;
		case 7:
			//go forward 7-ish feet to run into boiler
			if (forward(8.5 * 12.0) ) {
				//rover on carpet: forward7ft(-0.8, 8.5 * 12.0)
				//rover on tile: forward7ft(-0.8, 8.5*12.0)
				state = 8;
				//ahrs->Reset();
				ahrs->ZeroYaw();
			}
			break;
		case 8:
			//launch
			state = 9;
			break;
		default:
			stopx();
		}
		return;
	}

	void autoRed1A(void) {
		//Red center position code
		//this version drive diagonally to the hopper

		switch (state) {
		case 1:
			// go forward 7 ft
			if (forward(8 * 12.0) ) {
				//rover on carpet: forward7ft(-0.8, 6.5 * 12.0)
				//rover on tile: forward7ft(-0.8, 6.5 * 12.0)
				state = 2;
				//ahrs->Reset();
				ahrs->ZeroYaw();
			}
			break;
		case 2:
			//robot drives counterclockwise into hopper
			if (timedDrive(1, 0.2, 0.1) ) {
				//rover on carpet: pause(1, 0) 		// changed from pause -> timedDrive()
				//rover on tile: pause(1, 0)
				state = 3;
			}
			break;
		case 3:
			//go backward 3-ish feet
			if (forward(3 * 12.0) ) {
				//rover on carpet: forward7ft(-0.8, 3 * 12.0)
				//rover on tile:forward7ft(-0.8, 3 * 12.0)
				state = 4;
				//ahrs->Reset();
				ahrs->ZeroYaw();
			}
			break;
		case 4:
			// turns counterclockwise enough degrees to face boiler
			if (autonTurn(-125) ) {
				//rover on carpet: autonTurn(-125, 5, -0.012)
				//rover on tile: autonTurn(-115, 5, -0.012)
				state = 5;
				EncoderLeft.Reset();
				EncoderRight.Reset();
			}
			break;
		case 5:
			//go forward 7-ish feet to run into boiler
			if (forward(8.5 * 12.0) ) {
				//rover on carpet: forward7ft(-0.8, 8.5 * 12.0)
				//rover on tile: forward7ft(-0.8, 8.5*12.0)
				state = 6;
				//ahrs->Reset();
				ahrs->ZeroYaw();
			}
			break;
		case 6:
			//launch
			state = 9;
			break;
		default:
			stopx();
		}
		return;
	}

	void autoRed2(void) {
		//puts gear on front of airship
		switch (state) {
		case 1:
			if (forward(7 * 12.0) ) {
				//rover on carpet: forward7ft(-0.8, 7 * 12.0)
				//rover on tile: forward7ft(-0.8, 7 * 12.0)
				state = 9;
			}
			break;
		default:
			stopx();
		}
		return;
	}

	void autoRed3(void) {
		//red three
		//puts gear onto side of airship

		switch (state) {
		case 1:
			// go forward 7 ft
			if (forward(9 * 12.0) ) {
				//rover on carpet: forward7ft(-0.8, 9 * 12.0)
				//rover on tile: forward 7ft(-0.8, 9 * 12.0)
				state = 2;
				//ahrs->Reset();
				ahrs->ZeroYaw();
			}
			break;
		case 2:
			// turn 60 degrees clockwise
			if (autonTurn(60) ) {
				//rover on carpet: autonTurn(60, 5, -0.019)
				//rover on tile:autonTurn(60, 5, -0.012)
				state = 3;
				EncoderLeft.Reset();
				EncoderRight.Reset();
			}
			break;
		case 3:
			// go forward
			if (forward(2 * 12) ) {
				//rover on carpet: forward7ft(-0.6, 2 * 12)
				//rover on tile: forward7ft(-0.5, 2 * 12.0)
				state = 9;
				EncoderLeft.Reset();
				EncoderRight.Reset();
			}
			break;
		default:
			stopx();
		}
		return;
	}
	void SmartDashboardSenser() {
		//Read Auton Switch
		AutoSw1 = DiIn7.Get();
		AutoSw2 = DiIn8.Get();
		AutoSw3 = DiIn9.Get();

		AutoVal = AutoSw1 * 1 + AutoSw2 * 2 + AutoSw3 * 4;
		SmartDashboard::PutNumber("Auton Switch Value", AutoVal);

		SmartDashboard::PutString("Auton Mode", autoSelected);
		SmartDashboard::PutNumber("Auton State", state);
		SmartDashboard::PutNumber("Auton Timer", AutonTimer.Get());

		double DistanceLeft = EncoderLeft.GetRaw();
		SmartDashboard::PutNumber("DistanceLeft(raw)", DistanceLeft);
		SmartDashboard::PutNumber("DistanceLeft(Inch)",
				EncoderLeft.GetDistance());

		double DistanceRight = EncoderRight.GetRaw();
		SmartDashboard::PutNumber("DistanceRight(raw)", DistanceRight);
		SmartDashboard::PutNumber("DistanceRight(Inch)",
				EncoderRight.GetDistance());

		if (ahrs) {
			double gyroAngle = ahrs->GetAngle();
			SmartDashboard::PutNumber("Gyro Angle", gyroAngle);
		} else {
			SmartDashboard::PutNumber("Gyro Angle", 999);
		}
		//std::vector<double> arr = GRIPTable->
	}

	void motorSpeed(double leftMotor, double rightMotor) {
		DriveLeft0.Set(leftMotor * -1);
		DriveLeft1.Set(leftMotor * -1);
		DriveLeft2.Set(leftMotor * -1);
		DriveRight0.Set(rightMotor);
		DriveRight1.Set(rightMotor);
		DriveRight2.Set(rightMotor);

		SmartDashboard::PutNumber("Drive Speed Left", leftMotor);
		SmartDashboard::PutNumber("Drive Speed Right", rightMotor);
	}

	int forward(double targetDistance) {
		double tolerance = 0.2;	//inches
		double kP_Linear = 0.3;
		double kP_Rotation = 0.012;
		double settlingTime = 0.16;	//seconds
		double maxDriveSpeed = 0.75; //percent

		double encoderDistance = EncoderLeft.GetDistance();
		double encoderError = encoderDistance - targetDistance;
		double driveCommandLinear = encoderError * kP_Linear;

		//limits max drive speed
		if (driveCommandLinear > maxDriveSpeed) {
			driveCommandLinear = maxDriveSpeed;
		} else if (driveCommandLinear < -1 * maxDriveSpeed) {
			driveCommandLinear = -1 * maxDriveSpeed;
		}

		double gyroAngle = ahrs->GetAngle();
		double driveCommandRotation = gyroAngle * kP_Rotation;
		//calculates and sets motor speeds
		motorSpeed(driveCommandLinear + driveCommandRotation,
				driveCommandLinear - driveCommandRotation);

		//routine helps prevent the robot from overshooting the distance
		if (isWaiting == 0) {
			if (abs(encoderError) < tolerance) {
				isWaiting = 1;
				AutonTimer.Reset();
			}
		}
		//timed wait
		else {
			float currentTime = AutonTimer.Get();
			if (abs(encoderError) > tolerance) {
				isWaiting = 0;
			} else if (currentTime > settlingTime) {
				isWaiting = 0;
				return 1;
			}
		}
		return 0;
	}

	int autonTurn(float targetYaw) {
		float tolerance = 5.0;
		float errorGain = -0.012;
		float settlingTime = 0.03;
		float currentYaw = ahrs->GetAngle();
		float yawError = currentYaw - targetYaw;

		motorSpeed(-1 * yawError * errorGain, yawError * errorGain);

		if (isWaiting == 0) {
			if (abs(yawError) < tolerance) {
				isWaiting = 1;
				AutonTimer.Reset();
			}
		}
		//timed wait
		else {
			float currentTime = AutonTimer.Get();
			if (abs(yawError) > tolerance) {
				isWaiting = 0;
			} else if (currentTime > settlingTime) {
				isWaiting = 0;
				return 1;
			}
		}
		return 0;
	}



	int timedDrive(double driveTime, double leftMotorSpeed,
			double rightMotorSpeed) {
		float currentTime = AutonTimer.Get();
		if (currentTime < driveTime) {
			motorSpeed(leftMotorSpeed, rightMotorSpeed);
		} else {
			stopx();
			return 1;
		}
		return 0;
	}

	int stopx() {
		motorSpeed(0, 0);
		//Go to next state turn left 90 degrees
		return 1;
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
	std::shared_ptr<NetworkTable> GRIPTable;
	int isWaiting = 0;
}
;

START_ROBOT_CLASS(Robot)
