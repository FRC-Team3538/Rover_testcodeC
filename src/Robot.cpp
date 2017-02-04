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

public:
	Robot() :
			Adrive(DriveLeft0, DriveLeft1, DriveRight0, DriveRight1), Bdrive(
					DriveLeft2, DriveRight2), chooser(), Drivestick(0), DriveLeft0(
					0), DriveLeft1(1), DriveLeft2(2), DriveRight0(3), DriveRight1(
					4), DriveRight2(5), EncoderLeft(0, 1), EncoderRight(2, 3),

			table(NULL), ahrs(NULL), state(0), AutonOverride(), AutoSw1(), AutoSw2(), AutoSw3(), DiIn9(
					9), DiIn8(8), DiIn7(7), AutoVal(), AutoVal0(), AutoVal1(), AutoVal2(), OutputX(), OutputY()
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
		driveSolenoid->Set(false);      //turn off all solenoids

		EncoderLeft.SetDistancePerPulse(0.0243228675 * 4);
		EncoderRight.SetDistancePerPulse(0.0243228675 * 4);
		OutputX = 0, OutputY = 0;

		//from NAVX mxp data monitor example
		try {/////***** Let's do this differently.  We want Auton to fail gracefully, not just abort. Remember Ariane 5
			/* Communicate w/navX MXP via the MXP SPI Bus.                                       */
			/* Alternatively:  I2C::Port::kMXP, SerialPort::Port::kMXP or SerialPort::Port::kUSB */
			/* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details.   */
			ahrs = new AHRS(SPI::Port::kMXP, 200);
			ahrs->Reset();
		} catch (std::exception ex) {
			std::string err_string = "Error instantiating navX MXP:  ";
			err_string += ex.what();
			DriverStation::ReportError(err_string.c_str());
		}
	}

	void AutonomousInit() override {
		state = 1; /////***** Rename this.  It is ambiguous with the global one.
		isWaiting = 0;						/////***** Rename this.

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
		//zeros the navX
		if (ahrs) {
			ahrs->ZeroYaw();
		}

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
			stopx(); /////***** let's rename this to something more descriptive

		SmartDashboardSenser();
	}

	void TeleopPeriodic() {
		float Deadband = 0.11;
		float MaxSpeed = 0.5;

		//high gear & low gear controls
		bool LHbutton = Drivestick.GetRawButton(5); // get Left Bumper
		bool RHbutton = Drivestick.GetRawButton(6); // get Right Bumper

		if (RHbutton)
			driveSolenoid->Set(true);			// High gear press RH bumper
		if (LHbutton)
			driveSolenoid->Set(false);			// Low gear press LH bumper

		double SpeedLinear = Drivestick.GetRawAxis(1) * -1; // get Yaxis value (forward)
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
		switch (state) {  						/////***** rename this.
		case 1: /////***** Let's change the state numbers to #define's.  They really should be enum's, but I don't want to work that hard.
			// go forward 7 ft
			if (forward(6.5 * 12.0)) { /////***** Replace all "magic numbers" with #define's
				//rover on carpet:forward7ft(-0.8, 6.5 * 12.0)
				//rover on tile: forward7ft(-0.8, 6.5 *12.0)
				//bob on tile:forward(6.5 * 12.0, 0.2, 0.3, 0.16)
				state = 2; 						/////***** Use #define
				ahrs->ZeroYaw();
			}
			break;
		case 2: 								/////***** Use #define.
			// turn 90 degrees clockwise
			if (autonTurn(85)) {  		/////***** Magic number / Use #define
				//rover on carpet: autonTurn(85, 5, -0.012)
				//rover on tile: autonTurn(75, 5, -0.012)
				//bob on tile: autonTurn(85, 10, -0.009)
				state = 3; 						/////***** Use #define
				EncoderLeft.Reset();
				EncoderRight.Reset();
				ahrs->ZeroYaw();
			}
			break;
		case 3: 								/////***** Use #define
			// go forward 7 ft to hit hopper
			if (forward(7.6 * -1 * 12.0)) { /////***** Magic number / Use #define
				//rover on carpet: forward7ft(0.8, 7 * -1 * 12.0)
				//rover on tile: forward7ft(0.8, 7 * -1 * 12.0)
				//bob on tile: forward7ft(0.4, 7 * -1 * 12.0)
				//bob on tile:forward(7.6 * -1 * 12.0, 0.2, 0.3, 0.16)
				state = 9; 						/////***** Use #define
				AutonTimer.Reset();
			}
			break;
		case 4: 								/////***** Use #define
			//waits a couple of seconds for balls
			if (timedDrive(2, 0.1, 0.1)) { /////***** Magic numbers / Use #define
				//rover on carpet: pause(1, 0.1)
				//rover on tile: pause(1, 0.1)
				//bob on tile: timedDrive(2, 0.1, 0.1)
				state = 5; 						/////*****Use #define
				EncoderLeft.Reset();
				EncoderRight.Reset();
				ahrs->ZeroYaw();
			}
			break;
		case 5:  								/////***** Use #define
			//go backward 4-ish feet
			if (forward(2.5 * 12.0)) { 			/////***** Use #define
				//rover on carpet: forward(-0.8, 4 * 12.0)
				//rover on tile: forward7ft(-0.8, 4 * 12.0)
				//bob on tile: forward7ft(-0.4, 4*12.0)
				//bob on tile: forward(2.5 * 12.0, 0.2, 0.3, 0.16)
				state = 6; 						/////***** Use #define
				//ahrs->Reset();
				ahrs->ZeroYaw();
			}
			break;
		case 6: 								/////***** Use #define
			// turn enough degrees to face boiler
			if (autonTurn(120)) { 				/////***** Use #define
				//rover on carpet: autonTurn(120, 5, -0.012)
				//rover on tile: autonTurn(105, 5, -0.012)
				//bob on tile: autonTurn(120, 5, -0.009)
				state = 7; 						/////***** Use #define
				EncoderLeft.Reset();
				EncoderRight.Reset();
				ahrs->ZeroYaw();
			}
			break;
		case 7:  								/////***** Use #define
			//go forward 7-ish feet to run into boiler
			if (forward(8.5 * 12.0)) { 			/////***** Use #define
				//rover on carpet:forward7ft(-0.8, 8.5 * 12.0)
				//rover on tile: forward7ft(-0.8, 8.5 * 12.0)
				//bob on tile: forward7ft(-0.4, 8.5 * 12.0)
				//bob on tile: forward(8.5 * 12.0, 0.2, 0.3, 0.16)
				state = 8; 						/////***** Use #define
				//ahrs->Reset();
				ahrs->ZeroYaw();
			}
			break;
		case 8: 								/////***** Use #define
			//launch
			state = 9; 							/////***** Use #define
			break;
		default:
			stopx();							/////***** Rename
		}
		return;
	}

	void autoBlue1A(void) {

		//Blue boiler side code
		//drives diagonally toward hopper
		switch (state) {						/////***** Rename
		case 1:									/////***** Use #define
			// go forward 8 ft diagonally towards hopper
			if (forward(7 * 12.0)) {			/////***** Use #define
				//rover on carpet:forward7ft(-0.8, 6.5 * 12.0)
				//rover on tile: forward7ft(-0.8, 6.5 *12.0)
				state = 2;						/////***** Use #define
				//ahrs->Reset();
				ahrs->ZeroYaw();/////***** We need a better way to do initialization for the next state.
								/////***** This isn't important right now, but as the software gets more complicated, it will become more of a problem.
								/////***** I've noticed that we have too many "ahrs->ZeroYaw()"
								/////***** You can see that this initialization is already out of line.
								/////***** "State 2" used to use the gyroscope, but doesn't any more.
								/////***** The initialization for the old routine is still here.
								/////***** This isn't complicated enough to go full object-oriented,
								/////***** but whatever-we-call-case-2.init() would work.
								/////***** Can you think of a simpler way?
			}
			break;
		case 2:									/////***** Use #define
			//turns into hopper
			if (timedDrive(3, 0.2, -0.8)) {		/////***** Use #define
				//rover on carpet: pause(1, 0.1)
				//rover on tile: pause(1, 0.1)
				state = 3;						/////***** Use #define
			}
			break;
		case 3:									/////***** Use #define
			//go backward 4-ish feet
			if (forward(4 * 12.0)) {			/////***** Use #define
				//rover on carpet: forward(-0.8, 4 * 12.0)
				//rover on tile: forward7ft(-0.8, 4 * 12.0)
				state = 4;						/////***** Use #define
				//ahrs->Reset();
				ahrs->ZeroYaw();
			}
			break;
		case 4:
			// turn enough degrees to face boiler
			if (autonTurn(120)) {				/////***** Use #define
				//rover on carpet: autonTurn(120, 5, -0.012)
				//rover on tile: autonTurn(105, 5, -0.012)
				state = 5;						/////***** Use #define
				EncoderLeft.Reset();
				EncoderRight.Reset();
			}
			break;
		case 5:									/////***** Use #define
			//go forward 7-ish feet to run into boiler
			if (forward(8.5 * 12.0)) {			/////***** Use #define
				//rover on carpet:forward7ft(-0.8, 8.5 * 12.0)
				//rover on tile: forward7ft(-0.8, 8.5 * 12.0)
				state = 8;						/////***** Use #define
				//ahrs->Reset();
				ahrs->ZeroYaw();
			}
			break;
		case 6:									/////***** Use #define
			//launch
			state = 9;							/////***** rename / use #define
			break;
		default:
			stopx();							/////***** rename
		}

		return;

	}

	void autoBlue2(void) {
		//blue side code
		//goes forward to put gear on pin

		switch (state) {						/////***** rename "state"
		case 1:									/////***** use #define
			//ahrs->Reset();
			ahrs->ZeroYaw();
			state = 2;							/////***** rename / use #define
			break;

		case 2:									/////***** use #define
			if (forward(4 * 12)) {				/////***** Use #define
				//gain = 0.05 is too small
				//rover on carpet: forward7ft(-0.8, 7 * 12.0)
				//rover on tile: forward7ft(-0.8, 7 * 12.0)
				//if (forward7ft(-0.4, 7 * 12.0)) {
				state = 3;						/////***** rename / use #define
			}
			break;

		default:
			stopx();							/////***** rename

		}
		return;
	}

	void autoBlue3(void) {
		//blue side code
		//puts gear on pin on side of airship

		switch (state) {						/////***** rename
		case 1:									/////***** use #define
			// go forward 7 ft
			if (forward(7 * 12.0)) {			/////***** use #define
				//rover on carpet: forward 7ft(-0.8, 7 * 12.0)
				//rover on tile: forward7ft(-0.8, 7 * 12.0)
				state = 2;						/////***** rename / use #define
				//ahrs->Reset();
				ahrs->ZeroYaw();
			}
			break;

		case 2:									/////***** use #define
			// turn 90 degrees counterclockwise
			if (autonTurn(-60)) {				/////***** Use #define
				//rover on carpet: autonTurn(-60, 5, -0.012)
				//rover on tile: autonTurn(-60, 5, -0.012)
				state = 3;						/////***** rename / use #define
				EncoderLeft.Reset();
				EncoderRight.Reset();
			}
			break;
		case 3:									/////***** use #define
			// go forward
			if (forward(2 * 12)) {				/////***** Use #define
				//rover on carpet: forward7ft(-0.6, 2 * 12.0)
				//rover on tile: forward7ft(-0.5, 2 * 12)
				state = 9;						/////***** rename / use #define
				EncoderLeft.Reset();
				EncoderRight.Reset();
			}
			break;
		default:
			stopx();							/////***** rename
		}
		return;
	}

	void autoRed1(void) {
		//Red center position code
		//this version turns the robot in a right angle

		switch (state) {						/////***** rename
		case 1:									/////***** use #define
			// go forward 7 ft
			if (forward(6.5 * 12.0)) {			/////***** Use #define
				//rover on carpet: forward7ft(-0.8, 6.5 * 12.0)
				//rover on tile: forward7ft(-0.8, 6.5 * 12.0)
				state = 2;						/////***** rename, use #define
				//ahrs->Reset();
				ahrs->ZeroYaw();
			}
			break;
		case 2:
			// turn 90 degrees counterclockwise
			if (autonTurn(-95)) {				/////***** use #define
				//rover on carpet: autonTurn(-95, 5, -0.012)
				//rover on tile: autonTurn(-82, 5, -0.012)
				state = 3;						/////***** rename/use #define
				EncoderLeft.Reset();
				EncoderRight.Reset();
			}
			break;
		case 3:									/////***** use #define
			// go forward 7 ft to hit hopper
			if (forward(7 * -1 * 12.0)) {		/////***** Use #define
				//rover on carpet: forward7ft(0.8, 7 * -1 * 12.0)
				//rover on tile: forward7ft(0.8, 7 * 12.0)
				state = 4;						/////***** rename / use #define.
				AutonTimer.Reset();
			}
			break;
		case 4:									/////***** use #define
			//waits in front of hopper a couple of seconds for balls
			if (timedDrive(1, 0.05, 0.05)) {	/////***** Use #define
				//rover on carpet: pause(1, 0)
				//rover on tile: pause(1, 0)
				state = 5;						/////***** rename / use #define
			}
			break;
		case 5:									/////***** use #define
			//go backward 3-ish feet
			if (forward(3 * 12.0)) {			/////***** use #define
				//rover on carpet: forward7ft(-0.8, 3 * 12.0)
				//rover on tile:forward7ft(-0.8, 3 * 12.0)
				state = 6;						/////***** rename / use #define
				//ahrs->Reset();
				ahrs->ZeroYaw();
			}
			break;
		case 6:									/////***** use #define
			// turns counterclockwise enough degrees to face boiler
			if (autonTurn(-125)) {				/////***** use #define
				//rover on carpet: autonTurn(-125, 5, -0.012)
				//rover on tile: autonTurn(-115, 5, -0.012)
				state = 7;						/////***** rename / use #define
				EncoderLeft.Reset();
				EncoderRight.Reset();
			}
			break;
		case 7:									/////***** use #define
			//go forward 7-ish feet to run into boiler
			if (forward(8.5 * 12.0)) {			/////***** use #define
				//rover on carpet: forward7ft(-0.8, 8.5 * 12.0)
				//rover on tile: forward7ft(-0.8, 8.5*12.0)
				state = 8;						/////***** rename / use #define
				//ahrs->Reset();
				ahrs->ZeroYaw();
			}
			break;
		case 8:									/////***** use #define
			//launch
			state = 9;							/////***** rename / use #define
			break;
		default:
			stopx();							/////***** rename
		}
		return;
	}

	void autoRed1A(void) {
		//Red center position code
		//this version drive diagonally to the hopper

		switch (state) {						/////***** rename
		case 1:									/////***** use #define
			// go forward 7 ft
			if (forward(8 * 12.0)) {			/////***** use #define
				//rover on carpet: forward7ft(-0.8, 6.5 * 12.0)
				//rover on tile: forward7ft(-0.8, 6.5 * 12.0)
				state = 2;						/////***** rename / use #define
				//ahrs->Reset();
				ahrs->ZeroYaw();
			}
			break;
		case 2:									/////***** use #define
			//robot drives counterclockwise into hopper
			if (timedDrive(1, 0.2, 0.1)) {		/////***** use #define
				//rover on carpet: pause(1, 0) 		// changed from pause -> timedDrive()
				//rover on tile: pause(1, 0)
				state = 3;						/////***** rename / use #define
			}
			break;
		case 3:									/////***** use #define
			//go backward 3-ish feet
			if (forward(3 * 12.0)) {/////***** use #define / It is not obvious that this is correct.  Why is backing up using a positive speed?
				//rover on carpet: forward7ft(-0.8, 3 * 12.0)
				//rover on tile:forward7ft(-0.8, 3 * 12.0)
				state = 4;						/////***** rename / use #define
				//ahrs->Reset();
				ahrs->ZeroYaw();
			}
			break;
		case 4:									/////***** use #define
			// turns counterclockwise enough degrees to face boiler
			if (autonTurn(-125)) {/////***** use #define.  Remember right and left turns may need different angles.
				//rover on carpet: autonTurn(-125, 5, -0.012)
				//rover on tile: autonTurn(-115, 5, -0.012)
				state = 5;						/////***** rename / use #define
				EncoderLeft.Reset();
				EncoderRight.Reset();
			}
			break;
		case 5:									/////***** use #define
			//go forward 7-ish feet to run into boiler
			if (forward(8.5 * 12.0)) {			/////***** use #define
				//rover on carpet: forward7ft(-0.8, 8.5 * 12.0)
				//rover on tile: forward7ft(-0.8, 8.5*12.0)
				state = 6;						/////***** rename / use #define
				//ahrs->Reset();
				ahrs->ZeroYaw();
			}
			break;
		case 6:									/////***** use #define
			//launch
			state = 9;							/////***** use #define
			break;
		default:
			stopx();							/////***** rename
		}
		return;
	}

	void autoRed2(void) {
		//puts gear on front of airship
		switch (state) {						/////***** rename
		case 1:									/////***** use #define
			if (forward(7 * 12.0)) {			/////***** use #define
				//rover on carpet: forward7ft(-0.8, 7 * 12.0)
				//rover on tile: forward7ft(-0.8, 7 * 12.0)
				state = 9;						/////***** rename / use #define
			}
			break;
		default:
			stopx();							/////***** rename
		}
		return;
	}

	void autoRed3(void) {
		//red three
		//puts gear onto side of airship

		switch (state) {						/////***** rename
		case 1:									/////***** use #define
			// go forward 7 ft
			if (forward(9 * 12.0)) {			/////***** use #define
				//rover on carpet: forward7ft(-0.8, 9 * 12.0)
				//rover on tile: forward 7ft(-0.8, 9 * 12.0)
				state = 2;						/////***** rename / use #define
				//ahrs->Reset();
				ahrs->ZeroYaw();
			}
			break;
		case 2:									/////***** use #define
			// turn 60 degrees clockwise
			if (autonTurn(60)) {				/////***** use #define
				//rover on carpet: autonTurn(60, 5, -0.019)
				//rover on tile:autonTurn(60, 5, -0.012)
				state = 3;						/////***** rename / use #define
				EncoderLeft.Reset();
				EncoderRight.Reset();
			}
			break;
		case 3:									/////**** use #define
			// go forward
			if (forward(2 * 12)) {				/////***** use #define
				//rover on carpet: forward7ft(-0.6, 2 * 12)
				//rover on tile: forward7ft(-0.5, 2 * 12.0)
				state = 9;						/////***** rename / use #define
				EncoderLeft.Reset();
				EncoderRight.Reset();
			}
			break;
		default:
			stopx();							/////***** rename
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
			SmartDashboard::PutNumber("Gyro Angle", 999); /////***** don't use sentinels / use #define
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

	int forward(double targetDistance) {/////***** It is hard to find these when you are changing calibrations in a hurry.
										/////***** Putting them at the beginning makes them easier to find.
										/////***** There aren't any assignments to these variables. Use #define's instead.
		double tolerance = 0.2;	//inches		/////***** use #define
		double kP_Linear = 0.3;					/////***** use #define
		double kP_Rotation = 0.012;				/////***** use #define
		double settlingTime = 0.16;	//seconds	/////***** use #define
		double maxDriveSpeed = 0.75; //percent	/////***** use #define

		//put all encoder stuff in same place
		double encoderDistance = EncoderRight.GetDistance();
		double encoderError = encoderDistance - targetDistance;
		double driveCommandLinear = encoderError * kP_Linear;

		//limits max drive speed
		if (driveCommandLinear > maxDriveSpeed) {
			driveCommandLinear = maxDriveSpeed;
		} else if (driveCommandLinear < -1 * maxDriveSpeed) { /////***** "-1" is a "magic number." At least put a clear comment in here.
			driveCommandLinear = -1 * maxDriveSpeed;/////***** same as above.
		}

		double gyroAngle = ahrs->GetAngle();
		double driveCommandRotation = gyroAngle * kP_Rotation;
		//calculates and sets motor speeds
		motorSpeed(driveCommandLinear + driveCommandRotation,
				driveCommandLinear - driveCommandRotation);

		//routine helps prevent the robot from overshooting the distance
		if (isWaiting == 0) {/////***** Rename "isWaiting."  This isWaiting overlaps with the autonTurn() isWaiting.  There is nothing like 2 globals that are used for different things, but have the same name.
			if (abs(encoderError) < tolerance) {
				isWaiting = 1;
				AutonTimer.Reset();
			}
		}
		//timed wait
		else {
			float currentTime = AutonTimer.Get();
			if (abs(encoderError) > tolerance) {
				isWaiting = 0;					/////***** Rename
			} else if (currentTime > settlingTime) {
				isWaiting = 0;					/////***** Rename
				return 1;
			}
		}
		return 0;
	}

	int autonTurn(float targetYaw) {
		float tolerance = 5.0;/////***** These aren't assigned in this routine. Use #define's
		float errorGain = -0.012;				/////***** use #define
		float settlingTime = 0.03;				/////***** use #define
		float currentYaw = ahrs->GetAngle();
		float yawError = currentYaw - targetYaw;

		motorSpeed(-1 * yawError * errorGain, yawError * errorGain);

		if (isWaiting == 0) {/////***** Rename "isWaiting."  This isWaiting overlaps with the forward() isWaiting.  There is nothing like 2 globals that are used for different things, but have the same name.
			if (abs(yawError) < tolerance) {
				isWaiting = 1;
				AutonTimer.Reset();
			}
		}
		//timed wait
		else {
			float currentTime = AutonTimer.Get();
			if (abs(yawError) > tolerance) {
				isWaiting = 0;					/////***** Rename
			} else if (currentTime > settlingTime) {
				isWaiting = 0;					/////***** Rename
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
			stopx();							/////***** rename
			return 1;
		}
		return 0;
	}

	int stopx() {								/////***** rename
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
	int state; /////***** let's rename this.  We have a couple of different "state"'s and it isn't clear which is which
	bool AutonOverride, AutoSw1, AutoSw2, AutoSw3;
	DigitalInput DiIn9, DiIn8, DiIn7;
	int AutoVal, AutoVal0, AutoVal1, AutoVal2;
	float OutputX, OutputY;
	std::shared_ptr<NetworkTable> GRIPTable;
	int isWaiting = 0;					/////***** Divide this into 2 variables.

	Solenoid *driveSolenoid = new Solenoid(0);
}
;

START_ROBOT_CLASS(Robot)

