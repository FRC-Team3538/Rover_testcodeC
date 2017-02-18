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

//RJ-Libraries
//#include "MultiSpeedController.h"

// vision:
// Use x and y coordinates from "myContours report"
// if there are no objects use dead reckoning,
// if there is only one, use x, y coordinates in the table,
// if there are more than one, use the x, y coordinates for the blob with the largest area.
// There will be two "myContours." One will be for the boiler, one will be for the gear target.
//

//Calibrations
//------------------------Bob on tile calibrations START---------------------
// This will go to the middle of the field,
//		turn away from the hopper,
//		back up into the hopper.  (Timed so that collision with the hopper will set the robot position.)
//		hold the robot in position until all of the balls have fallen out of the hopper,
//		back up until the robot is in a position that can see the boiler,
//		point toward the boiler
//		go until collision with the boiler
//			(Timed so that collision will put the robot in position to shoot)
//			(This can be supported with vision when it is ready.)
//		launch the "fuel."
//
// go forward 6.5 ft, turn clockwise 90 degrees
//    back up 7.6 ft, go slowly forward for 3 seconds [**** This sign is backwards****]
//    turn clockwise 120 degrees, go forwards 8.5 ft

#define BLUE_1_CASE1_FWD (6.5 * 12.0)
#define BLUE_1_CASE2_TURN (90)
#define BLUE_1_CASE3_FWD (7.6 * -1 * 12.0)
#define BLUE_1_CASE4_FWD_TIME (3.5)
#define BLUE_1_CASE4_FWD_LEFT_SPD (0.2)
#define BLUE_1_CASE4_FWD_RIGHT_SPD (0.2)
#define BLUE_1_CASE5_FWD (2.5 * 12.0)
#define BLUE_1_CASE6_TURN (120)
#define BLUE_1_CASE7_FWD (8.5 * 12.0)

// This will go to the gear in front of the middle start position.
//		This is timed so that collision with the airship will put the robot in position.
//		This can be supported with vision when it is ready.
//
// go forward for .75 seconds at .9 speed
#define BLUE_2_CASE2_TIME (0.75)
#define BLUE_2_CASE2_LSPEED (-0.9)
#define BLUE_2_CASE2_RSPEED (-0.9)

// This will go to the correct position to see the gear position in front of position 3
//	Turn until the gear intake points to the gear.
//		This can be supported by vision when it is available.
//	go forward until the robot is in position for getting the gear.
//		This is timed so that collision with the airship will put the robot in the right position.
//		This can be supported by vision when it is available.
//
// go forward 7 ft, turn counter-clockwise 60 degrees
//    go forward 2 ft
#define BLUE_3_CASE1_FWD (7 * 12.0)
#define BLUE_3_CASE2_TURN (-60)
#define BLUE_3_CASE3_TIME (0.1)
#define BLUE_3_CASE3_LSPEED (-0.75)
#define BLUE_3_CASE3_RSPEED (-0.75)

// This will go to the middle of the field,
//		turn away from the hopper,
//		back up into the hopper.  (Timed so that collision with the hopper will set the robot position.)
//		hold the robot in position until all of the balls have fallen out of the hopper,
//		back up until the robot is in a position that can see the boiler,
//		point toward the boiler
//		go until collision with the boiler
//			(Timed so that collision will put the robot in position to shoot)
//			(This can be supported with vision when it is ready.)
//		launch the "fuel."
//
// go forward 6.5 ft, turn counterclockwise 90 degrees
//    back up 7 ft, go slowly forward for 1 second [**** is the sign backwards?****]
//    turn counterclockwise 125 degrees, go forwards 8.5 ft
#define RED_1_CASE1_FWD (6.5 * 12.0)
#define RED_1_CASE2_TURN (-95)
#define RED_1_CASE3_FWD (7 * -1 * 12.0)
#define RED_1_CASE4_FWD_TIME (3.5)
#define RED_1_CASE4_FWD_LEFT_SPD (0.2)
#define RED_1_CASE4_FWD_RIGHT_SPD (0.2)
#define RED_1_CASE5_FWD (3 * 12.0)
#define RED_1_CASE6_TURN (-125)
#define RED_1_CASE7_FWD (8.5 * 12.0)

// This will go to the gear in front of the middle start position.
//		This is timed so that collision with the airship will put the robot in position.
//		This can be supported with vision when it is ready.
//
// Time forward: go forward for 0.75 seconds.
// This will get us to the gear and stop.
#define RED_2_CASE2_TIME (0.75)
#define RED_2_CASE2_LSPEED (-0.9)
#define RED_2_CASE2_RSPEED (-0.9)

// Go to the correct position to see the gear position in front of position 3
//	Turn until the gear intake points to the gear.
//		This can be supported by vision when it is available.
//	go forward until the robot is in position for getting the gear.
//		This is timed so airship collision puts the robot in the
//			right position.
//		This can be supported by vision when it is available.
//
// go forward 9 ft, turn clockwise 60 degrees
//    go forward 2 ft
#define RED_3_CASE1_FWD (9 * 12.0)
#define RED_3_CASE2_TURN (60)
#define RED_3_CASE3_FWD (2 * 12)

//linear calibrations
#define LINEAR_TOLERANCE (0.2)
// This is the gain for using the encoders to set the distance
//		while going straight.
#define KP_LINEAR (0.3)
// This is the gain for using the gyroscope to go straight
#define KP_ROTATION (0.012)
#define LINEAR_SETTLING_TIME (0.1)
#define LINEAR_MAX_DRIVE_SPEED (0.75)

//turning calibrations
#define ROTATIONAL_TOLERANCE (5.0)
// This is the gain for turning using the gyroscope
#define ERROR_GAIN (-0.012)
#define ROTATIONAL_SETTLING_TIME (0.1)

//------------------------Bob on tile calibrations END---------------------

//------------------------Rover calibrations START-------------------------
//Rover calibrations are out of date, but it gives us somewhere to start,
//	if we need to.
//Blue 1
//				carpet:	forward7ft( 0.8, 7 * -1 * 12.0)tile:forward7ft( 0.8, 7 * -1 * 12.0)
//				carpet:	forward7ft(-0.8, 6.5 * 12.0)tile: 	forward7ft(-0.8, 6.5 *12.0)
//b1 case 2:	carpet:	autonTurn(85, 5, -0.012)	tile:	autonTurn(75, 5, -0.012)
//b1 case 4:	carpet:	pause(1, 0.1)				tile:	pause(1, 0.1)
//b1 case 5:	carpet:	forward(   -0.8, 4 * 12.0)	tile:	forward7ft(-0.8, 4 * 12.0)
//b1 case 5:	carpet:	autonTurn(120, 5, -0.012)	tile:	autonTurn(105, 5, -0.012)
//b1 case 7:	carpet:	forward7ft(-0.8, 8.5 * 12.0)tile: 	forward7ft(-0.8, 8.5 * 12.0)
//b2 			carpet:	forward7ft(-0.8, 7 * 12.0)	tile: 	forward7ft(-0.8, 7 * 12.0)
//b3 AB3_FWD:	carpet:	forward7ft(-0.8, 7 * 12.0)	tile: 	forward7ft(-0.8, 7 * 12.0)
//b3 AB3_TURN	carpet: autonTurn(-60, 5, -0.012)	tile: 	autonTurn(-60, 5, -0.012)
//b3 AB3_STR8	carpet:	forward7ft(-0.6, 2 * 12.0)	tile: 	forward7ft(-0.5, 2 * 12)
//R1 CASE1_FWD	carpet: forward7ft(-0.8, 6.5 * 12.0)tile: 	forward7ft(-0.8, 6.5 * 12.0)
//R1_CASE2_TURN	carpet:	autonTurn(-95, 5, -0.012)	tile: 	autonTurn(-82, 5, -0.012)
//R1_CASE3_FWD	carpet:	forward7ft(0.8, 7 * -1 * 12.0)tile:	forward7ft(0.8, 7 * 12.0)
//R1_CASE4_FWD_TIMEcarpet:pause(1, 0)				tile: 	pause(1, 0)
//R1_CASE5_FWD	carpet:	forward7ft(-0.8, 3 * 12.0)	tile:	forward7ft(-0.8, 3 * 12.0)
//R1_CASE6_TURN	carpet:	autonTurn(-125, 5, -0.012)	tile: 	autonTurn(-115, 5, -0.012)
//R1_CASE7_FWD	carpet:	forward7ft(-0.8, 8.5 * 12.0)tile: 	forward7ft(-0.8, 8.5*12.0)
//R2_CASE2_TIME	carpet: forward7ft(-0.8, 7 * 12.0)	tile:	forward7ft(-0.8, 7 * 12.0)
//R3_CASE1_FWD	carpet: forward7ft(-0.8, 9 * 12.0)	tile:	forward7ft(-0.8, 9 * 12.0)
//R3_CASE2_TURN	carpet: autonTurn(60, 5, -0.019)	tile:	autonTurn(60, 5, -0.012)
//R3_CASE3_FWD	carpet: forward7ft(-0.6, 2 * 12)	tile:	forward7ft(-0.5, 2 * 12.0)

////linear calibrations
//#define LINEAR_TOLERANCE ()
//#define KP_LINEAR (-0.8)
//#define KP_ROTATION ()
//#define LINEAR_SETTLING_TIME ()
//#define LINEAR_MAX_DRIVE_SPEED ()
//------------------------Rover calibrations END---------------------------

class Robot: public frc::IterativeRobot {

public:
	Robot() :
			Adrive(DriveLeft0, DriveLeft1, DriveRight0, DriveRight1), Bdrive(
					DriveLeft2, DriveRight2), chooser(), chooseEncoder(), Drivestick(
					0), OperatorStick(1), DriveLeft0(0), DriveLeft1(1), DriveLeft2(
					2), DriveRight0(3), DriveRight1(4), DriveRight2(5), AutonTimer(), EncoderLeft(
					0, 1), EncoderRight(2, 3),

			table(NULL), ahrs(NULL), modeState(0), AutonOverride(), AutoSw1(), AutoSw2(), AutoSw3(), DiIn9(
					9), DiIn8(8), DiIn7(7), AutoVal(), AutoVal0(), AutoVal1(), AutoVal2(), OutputX(), OutputY(), Winch0(
					11), Winch1(9), Shooter0(12), Shooter1(7), Conveyor(13), Agitator(
					6), FloorIntakeRoller(14), KickerWheel(8), DeflectorMotor(
					10), EncoderKicker(20, 21), EncoderShoot(4, 5), WinchStop(
					6), DeflectorAnglePOT(0, 270, 0), useClosedLoop(false), DeflectorTarget(
					0), DeflectorHighLimit(22), DeflectorLowLimit(23), useRightEncoder(), DeflectorPID(
					0.03, 0.0, 0.0, &DeflectorAnglePOT, &DeflectorMotor), KickerPID(
					0.03, 0.0, 0.0, &EncoderKicker, &KickerWheel), ShooterPID(0.03,
					0.0, 0.0, &EncoderShoot, &Shooter0) {
		//^^^^^^^^^^^^^adjust the kP value... right now they're just the same guess for each PID... which is wrong :)
		//GRIPTable = NetworkTable::GetTable("GRIP/myContuorsReport");
		//Shooter = new MultiSpeedController();
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

		chooseEncoder.AddDefault(RH_Encoder, RH_Encoder);
		chooseEncoder.AddObject(LH_Encoder, LH_Encoder);
		frc::SmartDashboard::PutData("Encoder", &chooseEncoder);

		driveSolenoid->Set(false);      //turn off all solenoids

		Adrive.SetSafetyEnabled(false);
		Bdrive.SetSafetyEnabled(false);

		EncoderRight.SetReverseDirection(true);
		EncoderShoot.SetReverseDirection(true);
		EncoderKicker.SetReverseDirection(true);

		EncoderLeft.SetDistancePerPulse(0.0243228675 * 4);
		EncoderRight.SetDistancePerPulse(0.0243228675 * 4);
		EncoderShoot.SetDistancePerPulse(1.0 / 32.0 * 4.0);
		EncoderKicker.SetDistancePerPulse(1.0 / 32.0 * 4.0);
		OutputX = 0, OutputY = 0;

		//variable that chooses which encoder robot is reading
		useRightEncoder = true;

		//reads sensors for speed not distance
		DeflectorAnglePOT.SetPIDSourceType(PIDSourceType::kDisplacement);
		EncoderKicker.SetPIDSourceType(PIDSourceType::kRate);

		// Turn off the the sensors/reset
		if (useClosedLoop) {
			DeflectorPID.Enable();
			KickerPID.Enable();
			ShooterPID.Enable();
		} else {
			DeflectorPID.Disable();
			KickerPID.Disable();
			ShooterPID.Disable();
		}

		//from NAVX mxp data monitor example
		try { /////***** Let's do this differently.  We want Auton to fail gracefully, not just abort. Remember Ariane 5
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
		// This gives the NAVX time to reset.
		// It takes about 0.5 seconds for the reset to complete.
		// RobotInit runs well before the autonomous mode starts,
		//		so there is plenty of time.
		Wait(1);
	}

	void AutonomousInit() override {
		modeState = 1;
		isWaiting = 0;							/////***** Rename this.

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
		//forces robot into low gear
		driveSolenoid->Set(false);

		SmartDashboard::PutString("autonMode", "Off");

		if (AutonOverride) {
			autoSelected = chooser.GetSelected();
			std::cout << "Auto selected: " << autoSelected << std::endl;
		} else {
			// This decodes all of the possible switch outputs.
			// Only 3 bits are connected to the RoboRio, so they aren't all needed.
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

		OutputX = 0, OutputY = 0;
	}

	void RobotPeriodic() {
		Shooter1.Set(-Shooter0.Get());
		SmartDashboardSenser();
	}
	void DisabledPeriodic() {
		//SmartDashboardSenser();
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
			stopMotors();

		//SmartDashboardSenser();
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

		//SmartDashboardSenser();

		// Turn on the shooter, conveyer, and agitator
		if (OperatorStick.GetRawAxis(3) > Deadband) {

			Shooter0.Set(-OperatorStick.GetRawAxis(3));
			Conveyor.Set(OperatorStick.GetRawAxis(3));
			Agitator.Set(OperatorStick.GetRawAxis(3));

		} else {

			Shooter0.Set(0);
			Conveyor.Set(0);
			Agitator.Set(0);

		}

		// Turn on Kicker WHeel
		if (OperatorStick.GetRawButton(1)) {
			KickerWheel.Set(1);
		} else {
			KickerWheel.Set(0);
		}

		//Put out intake
		if (OperatorStick.GetRawAxis(2) > Deadband) {
			FloorIntakeRoller.Set(1);
			FloorIntakeArm->Set(true);
		} else {
			FloorIntakeRoller.Set(0);
			FloorIntakeArm->Set(false);
		}

		//Button to get and release the gear
		GearIn->Set(OperatorStick.GetRawButton(3));
		GearOut->Set(OperatorStick.GetRawButton(3));

		//turn on winch
		if (abs(OperatorStick.GetRawAxis(1)) > Deadband) {
			Winch0.Set(OperatorStick.GetRawAxis(1));
			Winch1.Set(-OperatorStick.GetRawAxis(1));
		} else {
			Winch0.Set(0.0);
			Winch1.Set(0.0);
		}

		//control deflector angle in open loop
		if (!useClosedLoop) {
			if (abs(OperatorStick.GetRawAxis(5)) > Deadband) {
				DeflectorMotor.Set(OperatorStick.GetRawAxis(5) * 0.1);
			}
			else{
				DeflectorMotor.Set(0.0);
			}
		} else {
			if (abs(OperatorStick.GetRawAxis(5)) < Deadband) {
				//stops the deflector once joystick is released
				DeflectorTarget = DeflectorAnglePOT.Get();
			} else {
				//will increment the DeflectorTarget by value set by joysticks
				DeflectorTarget += OperatorStick.GetRawAxis(5) * 0.1;
			}
			DeflectorPID.SetSetpoint(DeflectorTarget);
		}

		// Turn off the the sensors/reset
		if (OperatorStick.GetRawButton(8)) {
			useClosedLoop = true;
			DeflectorPID.Enable();
		}
		if (OperatorStick.GetRawButton(7)) {
			useClosedLoop = false;
			DeflectorPID.Disable();
		}

		//Controll the angle of the deflector
		if (OperatorStick.GetRawButton(5)) {
			DeflectorTarget = 90;
		}
		if (OperatorStick.GetRawButton(6)) {
			DeflectorTarget = 0;
		}
	}

// These are the state numbers for each part of autoBlue1
//		These are here so we can easily add states.
// 		State 1 is always the first one to run.
//		Always have an "end" state.
#define AB1_INIT 1
#define AB1_FWD 2
#define AB1_TURN90 3
#define AB1_BKUP 4
#define AB1_WAIT 5
#define AB1_FWD2 6
#define AB1_FACE_BOILER 7
#define AB1_TO_BOILER 8
#define AR1_SHOOT 9
#define AB1_END 10
//***needs INIT case still
	void autoBlue1(void) {

		//Blue boiler side code
		//drives turns then drives again
		switch (modeState) {
		case AB1_INIT:
			modeState = AB1_FWD;
			break;
		case AB1_FWD:
			// go forward 7 ft
			if (forward(BLUE_1_CASE1_FWD)) {
				modeState = AB1_TURN90;
				ahrs->ZeroYaw();
			}
			break;
		case AB1_TURN90:
			// turn 90 degrees clockwise
			if (autonTurn(BLUE_1_CASE2_TURN)) {
				//modeState = AB1_BKUP;
				modeState = AB1_WAIT;
				EncoderLeft.Reset();
				EncoderRight.Reset();
				ahrs->ZeroYaw();
			}
			break;
//		case AB1_BKUP:
//			// go forward 7 ft to hit hopper
//			//change to timed drive
//			if (forward(BLUE_1_CASE3_FWD)) {
//				modeState = AB1_WAIT;
//				AutonTimer.Reset();
//			}
//			break;
		case AB1_WAIT:
			//waits a couple of seconds for balls
			if (timedDrive(BLUE_1_CASE4_FWD_TIME, BLUE_1_CASE4_FWD_LEFT_SPD,
			BLUE_1_CASE4_FWD_RIGHT_SPD)) {
				modeState = AB1_FWD2;
				EncoderLeft.Reset();
				EncoderRight.Reset();
				ahrs->ZeroYaw();
			}
			break;
		case AB1_FWD2:
			//go backward 4-ish feet
			if (forward(BLUE_1_CASE5_FWD)) {
				modeState = AB1_FACE_BOILER;
				ahrs->ZeroYaw();
			}
			break;
		case AB1_FACE_BOILER:
			// turn enough degrees to face boiler
			if (autonTurn(BLUE_1_CASE6_TURN)) {
				modeState = AB1_TO_BOILER;
				EncoderLeft.Reset();
				EncoderRight.Reset();
				ahrs->ZeroYaw();
			}
			break;
		case AB1_TO_BOILER:
			//go forward 7-ish feet to run into boiler
			//change to timed drive
			if (forward(BLUE_1_CASE7_FWD)) {
				modeState = AR1_SHOOT;
				ahrs->ZeroYaw();
			}
			break;
		case AR1_SHOOT:
			//launch
			modeState = AB1_END;
			break;
		default:
			stopMotors();
		}
		return;
	}

	void autoBlue1A(void) {

		//Blue boiler side code
		//drives diagonally toward hopper
		switch (modeState) {
		case 1:									/////***** Use #define
			// go forward 8 ft diagonally towards hopper
			if (forward(7 * 12.0)) {			/////***** Use #define
				//rover on carpet:forward7ft(-0.8, 6.5 * 12.0)
				//rover on tile: forward7ft(-0.8, 6.5 *12.0)
				modeState = 2;					/////***** Use #define
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
				modeState = 3;					/////***** Use #define
			}
			break;
		case 3:									/////***** Use #define
			//go backward 4-ish feet
			if (forward(4 * 12.0)) {			/////***** Use #define
				//rover on carpet: forward(-0.8, 4 * 12.0)
				//rover on tile: forward7ft(-0.8, 4 * 12.0)
				modeState = 4;					/////***** Use #define
				ahrs->ZeroYaw();
			}
			break;
		case 4:
			// turn enough degrees to face boiler
			if (autonTurn(120)) {				/////***** Use #define
				//rover on carpet: autonTurn(120, 5, -0.012)
				//rover on tile: autonTurn(105, 5, -0.012)
				modeState = 5;					/////***** Use #define
				EncoderLeft.Reset();
				EncoderRight.Reset();
			}
			break;
		case 5:									/////***** Use #define
			//go forward 7-ish feet to run into boiler
			if (forward(8.5 * 12.0)) {			/////***** Use #define
				//rover on carpet:forward7ft(-0.8, 8.5 * 12.0)
				//rover on tile: forward7ft(-0.8, 8.5 * 12.0)
				modeState = 8;						/////***** Use #define
				ahrs->ZeroYaw();
			}
			break;
		case 6:									/////***** Use #define
			//launch
			modeState = 9;						/////*****use #define
			break;
		default:
			stopMotors();
		}

		return;

	}

#define AB2_INIT 1
#define AB2_FWD 2
#define AB2_END 3

	void autoBlue2(void) {
		//blue side code
		//goes forward to put gear on pin

		switch (modeState) {
		case AB2_INIT:									/////***** use #define
			// This uses state 1 for initialization.
			// This keeps the initialization and the code all in one place.
			ahrs->ZeroYaw();
			modeState = AB2_FWD;						/////***** use #define
			break;

		case AB2_FWD:									/////***** use #define
			if (timedDrive(BLUE_2_CASE2_TIME, BLUE_2_CASE2_LSPEED,
			BLUE_2_CASE2_RSPEED)) {
				//change to timed drive
				//if (forward7ft(-0.4, 7 * 12.0)) {
				modeState = AB2_END;					/////***** use #define
			}
			break;

		default:
			stopMotors();

		}
		return;
	}

#define AB3_INIT 1
#define AB3_FWD 2
#define AB3_TURN 3
#define AB3_STR8 4
#define AB3_END 5
	void autoBlue3(void) {
		//blue side code
		//puts gear on pin on side of airship

		switch (modeState) {
		case AB3_INIT:
			modeState = AB3_FWD;
			break;
		case AB3_FWD:
			// go forward 7 ft
			if (forward(BLUE_3_CASE1_FWD)) {
				modeState = AB3_TURN;
				ahrs->ZeroYaw();
			}
			break;
		case AB3_TURN:
			// turn 90 degrees counterclockwise
			if (autonTurn(BLUE_3_CASE2_TURN)) {
				modeState = AB3_STR8;
				EncoderLeft.Reset();
				EncoderRight.Reset();
				ahrs->ZeroYaw();
			}
			break;
		case AB3_STR8:
			// go forward
			//timed drive
			//CHANGE THIS IT DOESN'T WORK BC IT DOESN'T MOVE FORWARD LIKE IT'S SUPPOSED TO :) (2/10)

			if (timedDrive(BLUE_3_CASE3_TIME, BLUE_3_CASE3_LSPEED,
			BLUE_3_CASE3_RSPEED)) {
				modeState = AB3_END;
				EncoderLeft.Reset();
				EncoderRight.Reset();
			}
			break;
		default:
			stopMotors();
		}
		return;
	}

#define AR1_INIT 1
#define AR1_FWD 2
#define AR1_TURN90 3
#define AR1_BKUP 4
#define AR1_WAIT 5
#define AR1_FWD2 6
#define AR1_FACE_BOILER 7
#define AR1_TO_BOILER 8
#define AR1_SHOOT 9
#define AR1_END 10
//***also still needs INIT case
	void autoRed1(void) {
		//Red center position code
		//this version turns the robot in a right angle

		switch (modeState) {
		case AR1_INIT:
			modeState = AR1_FWD;
			break;
		case AR1_FWD:
			// go forward 7 ft
			if (forward(RED_1_CASE1_FWD)) {
				modeState = AR1_TURN90;
				ahrs->ZeroYaw();
			}
			break;
		case AR1_TURN90:
			// turn 90 degrees counterclockwise
			if (autonTurn(RED_1_CASE2_TURN)) {
				modeState = AR1_BKUP;
				EncoderLeft.Reset();
				EncoderRight.Reset();
			}
			break;
		case AR1_BKUP:
			//change to timed drive
			// go forward 7 ft to hit hopper
			if (forward(RED_1_CASE3_FWD)) {
				modeState = AR1_WAIT;
				AutonTimer.Reset();
			}
			break;
		case AR1_WAIT:
			//waits in front of hopper a couple of seconds for balls
			if (timedDrive(RED_1_CASE4_FWD_TIME, RED_1_CASE4_FWD_LEFT_SPD,
			RED_1_CASE4_FWD_RIGHT_SPD)) {
				modeState = AR1_FWD2;

			}
			break;
		case AR1_FWD2:
			//go backward 3-ish feet
			if (forward(RED_1_CASE5_FWD)) {
				modeState = AR1_FACE_BOILER;
				ahrs->ZeroYaw();
			}
			break;
		case AR1_FACE_BOILER:
			// turns counterclockwise enough degrees to face boiler
			if (autonTurn(RED_1_CASE6_TURN)) {
				modeState = AR1_TO_BOILER;
				EncoderLeft.Reset();
				EncoderRight.Reset();
			}
			break;
		case AR1_TO_BOILER:
			//change to timed drive
			//go forward 7-ish feet to run into boiler
			if (forward(RED_1_CASE7_FWD)) {
				modeState = AR1_SHOOT;
				ahrs->ZeroYaw();
			}
			break;
		case AR1_SHOOT:
			//launch
			modeState = AR1_END;
			break;
		default:
			stopMotors();
		}
		return;
	}

	void autoRed1A(void) {
		//Red center position code
		//this version drive diagonally to the hopper

		switch (modeState) {
		case 1:									/////***** use #define
			// go forward 7 ft
			if (forward(8 * 12.0)) {			/////***** use #define
				//rover on carpet: forward7ft(-0.8, 6.5 * 12.0)
				//rover on tile: forward7ft(-0.8, 6.5 * 12.0)
				modeState = 2;					/////***** use #define
				//ahrs->Reset();
				ahrs->ZeroYaw();
			}
			break;
		case 2:									/////***** use #define
			//robot drives counterclockwise into hopper
			if (timedDrive(1, 0.2, 0.1)) {		/////***** use #define
				//rover on carpet: pause(1, 0) 		// changed from pause -> timedDrive()
				//rover on tile: pause(1, 0)
				modeState = 3;					/////***** use #define
			}
			break;
		case 3:									/////***** use #define
			//go backward 3-ish feet
			if (forward(3 * 12.0)) {/////***** use #define / It is not obvious that this is correct.  Why is backing up using a positive speed?
				//rover on carpet: forward7ft(-0.8, 3 * 12.0)
				//rover on tile:forward7ft(-0.8, 3 * 12.0)
				modeState = 4;					/////***** use #define
				//ahrs->Reset();
				ahrs->ZeroYaw();
			}
			break;
		case 4:									/////***** use #define
			// turns counterclockwise enough degrees to face boiler
			if (autonTurn(-125)) {/////***** use #define.  Remember right and left turns may need different angles.
				//rover on carpet: autonTurn(-125, 5, -0.012)
				//rover on tile: autonTurn(-115, 5, -0.012)
				modeState = 5;					/////***** use #define
				EncoderLeft.Reset();
				EncoderRight.Reset();
			}
			break;
		case 5:									/////***** use #define
			//go forward 7-ish feet to run into boiler
			if (forward(8.5 * 12.0)) {			/////***** use #define
				//rover on carpet: forward7ft(-0.8, 8.5 * 12.0)
				//rover on tile: forward7ft(-0.8, 8.5*12.0)
				modeState = 6;					/////*****use #define
				ahrs->ZeroYaw();
			}
			break;
		case 6:									/////***** use #define
			//launch
			modeState = 9;						/////***** use #define
			break;
		default:
			stopMotors();
		}
		return;
	}

#define AR2_INIT 1
#define AR2_FWD 2
#define AR2_END 3
	void autoRed2(void) {
		//change to timed drive
		//puts gear on front of airship
		switch (modeState) {
		case AR2_INIT:
			// This uses state 1 for initialization.
			// This keeps the initialization and the code all in one place.

			ahrs->ZeroYaw();
			modeState = AR2_FWD;
			break;
		case AR2_FWD:
			if (timedDrive(RED_2_CASE2_TIME, RED_2_CASE2_LSPEED,
			RED_2_CASE2_RSPEED)) {
				modeState = AR2_END;
			}
			break;
		default:
			stopMotors();
		}
		return;
	}

#define AR3_INIT 1
#define AR3_FWD 2
#define AR3_TURN 3
#define AR3_STR8 4
#define AR3_END 5
	void autoRed3(void) {
		//red three
		//puts gear onto side of airship

		switch (modeState) {
		case AR3_INIT:
			modeState = AR3_FWD;
			break;
		case AR3_FWD:
			// go forward 7 ft
			if (forward(RED_3_CASE1_FWD)) {
				modeState = AR3_TURN;
				ahrs->ZeroYaw();
			}
			break;
		case AR3_TURN:
			// turn 60 degrees clockwise
			if (autonTurn(RED_3_CASE2_TURN)) {
				modeState = AR3_STR8;
				EncoderLeft.Reset();
				EncoderRight.Reset();
			}
			break;
		case AR3_STR8:
			//change to timed drive
			// go forward
			if (forward(RED_3_CASE3_FWD)) {
				modeState = AR3_END;
				EncoderLeft.Reset();
				EncoderRight.Reset();
			}
			break;
		default:
			stopMotors();
		}
		return;
	}
	void SmartDashboardSenser() {
		//Read Auton Switch
		AutoSw1 = DiIn7.Get();
		AutoSw2 = DiIn8.Get();
		AutoSw3 = DiIn9.Get();

		AutoVal = !AutoSw1 * 1 + !AutoSw2 * 2 + !AutoSw3 * 4;
		SmartDashboard::PutNumber("Auton Switch Value", AutoVal);

		SmartDashboard::PutString("Auton Mode", autoSelected);
		SmartDashboard::PutNumber("Auton State", modeState);
		SmartDashboard::PutNumber("Auton Timer", AutonTimer.Get());

		double DistanceLeft = EncoderLeft.GetRaw();
		SmartDashboard::PutNumber("DistanceLeft(raw)", DistanceLeft);
		SmartDashboard::PutNumber("DistanceLeft(Inch)",
				EncoderLeft.GetDistance());

		double DistanceRight = EncoderRight.GetRaw();
		SmartDashboard::PutNumber("DistanceRight(raw)", DistanceRight);
		SmartDashboard::PutNumber("DistanceRight(Inch)",
				EncoderRight.GetDistance());

		encoderSelected = chooseEncoder.GetSelected();
		if (encoderSelected == RH_Encoder)
			useRightEncoder = true;
		else
			useRightEncoder = false;

		if (ahrs) {
			double gyroAngle = ahrs->GetAngle();
			SmartDashboard::PutNumber("Gyro Angle", gyroAngle);
		} else {
			SmartDashboard::PutNumber("Gyro Angle", 999); /////***** don't use sentinels / use #define
		}

		//Manipulator
		//shooter
		SmartDashboard::PutNumber("ShooterEncoder(raw)", EncoderShoot.GetRaw());
		SmartDashboard::PutNumber("ShooterEncoder(RPM)",
				EncoderShoot.GetRate());
		SmartDashboard::PutNumber("KickerEncoder(Rev)",
				EncoderKicker.GetDistance());

		//conveyor
		SmartDashboard::PutNumber("KickerEncoder(raw)", EncoderKicker.GetRaw());
		SmartDashboard::PutNumber("KickerEncoder(RPM)",
				EncoderKicker.GetRate());
		SmartDashboard::PutNumber("KickerEncoder(Rev)",
				EncoderKicker.GetDistance());

		SmartDashboard::PutBoolean("WinchStop", WinchStop.Get());
		SmartDashboard::PutBoolean("DeflectorHighLimit",
				DeflectorHighLimit.Get());
		SmartDashboard::PutBoolean("DeflectorLowLimit",
				DeflectorLowLimit.Get());

		//Get angle for the deflector
		SmartDashboard::PutNumber("DeflectorAnglePOT(DEG)",
				DeflectorAnglePOT.Get());

		//State varible
		SmartDashboard::PutNumber("DeflectorAngleTarget", DeflectorTarget);
		SmartDashboard::PutBoolean("DeflectorTarget", useClosedLoop);

		//grip table data
		//GRIPTable = NetworkTable::GetTable("GRIP/myContoursReport");

		//NetworkTable* t = GRIPTable.get();

//		std::vector<double> arr = GRIPTable->GetNumberArray("centerX",
//				llvm::ArrayRef<double>());
//
//		SmartDashboard::PutNumber("VisionTargetFind", arr.size());
//
//		if (arr.size() > 0) {
//			SmartDashboard::PutNumber("VisionCenterX", arr[0]);
//		} else {
//			SmartDashboard::PutNumber("VisionCenterX", 0);
//		}

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
		//put all encoder stuff in same place
		double encoderDistance;
		if (useRightEncoder)
			encoderDistance = EncoderRight.GetDistance();
		else
			encoderDistance = EncoderLeft.GetDistance();

		double encoderError = encoderDistance - targetDistance;
		double driveCommandLinear = encoderError * KP_LINEAR;

		//limits max drive speed
		if (driveCommandLinear > LINEAR_MAX_DRIVE_SPEED) {
			driveCommandLinear = LINEAR_MAX_DRIVE_SPEED;
		} else if (driveCommandLinear < -1 * LINEAR_MAX_DRIVE_SPEED) { /////***** "-1" is a "magic number." At least put a clear comment in here.
			driveCommandLinear = -1 * LINEAR_MAX_DRIVE_SPEED; /////***** same as above.
		}

		double gyroAngle = ahrs->GetAngle();
		double driveCommandRotation = gyroAngle * KP_ROTATION;
		//calculates and sets motor speeds
		motorSpeed(driveCommandLinear + driveCommandRotation,
				driveCommandLinear - driveCommandRotation);

		//routine helps prevent the robot from overshooting the distance
		if (isWaiting == 0) { /////***** Rename "isWaiting."  This isWaiting overlaps with the autonTurn() isWaiting.  There is nothing like 2 globals that are used for different things, but have the same name.
			if (abs(encoderError) < LINEAR_TOLERANCE) {
				isWaiting = 1;
				AutonTimer.Reset();
			}
		}
		//timed wait
		else {
			float currentTime = AutonTimer.Get();
			if (abs(encoderError) > LINEAR_TOLERANCE) {
				isWaiting = 0;					/////***** Rename
			} else if (currentTime > LINEAR_SETTLING_TIME) {
				isWaiting = 0;					/////***** Rename
				return 1;
			}
		}
		return 0;
	}

	int autonTurn(float targetYaw) {

		float currentYaw = ahrs->GetAngle();
		float yawError = currentYaw - targetYaw;

		motorSpeed(-1 * yawError * ERROR_GAIN, yawError * ERROR_GAIN);

		if (isWaiting == 0) {/////***** Rename "isWaiting."  This isWaiting overlaps with the forward() isWaiting.  There is nothing like 2 globals that are used for different things, but have the same name.
			if (abs(yawError) < ROTATIONAL_TOLERANCE) {
				isWaiting = 1;
				AutonTimer.Reset();
			}
		}
		//timed wait
		else {
			float currentTime = AutonTimer.Get();
			if (abs(yawError) > ROTATIONAL_TOLERANCE) {
				isWaiting = 0;					/////***** Rename
			} else if (currentTime > ROTATIONAL_SETTLING_TIME) {
				isWaiting = 0;					/////***** Rename
				return 1;
			}
		}
		return 0;
	}

	//need to change signs!!!
	int timedDrive(double driveTime, double leftMotorSpeed,
			double rightMotorSpeed) {
		float currentTime = AutonTimer.Get();
		if (currentTime < driveTime) {
			motorSpeed(leftMotorSpeed, rightMotorSpeed);
		} else {
			stopMotors();
			return 1;
		}
		return 0;
	}

	int stopMotors() {
		//sets motor speeds to zero
		motorSpeed(0, 0);
		return 1;
	}

	int setDeflectorAngle(double target) {
		DeflectorPID.SetSetpoint(target);
		//if the absolute value of the error is less than 1 degree return 1
		return (abs(DeflectorPID.GetError()) < 1.0);
	}

	int kickerSpeed(double speed) {
		KickerPID.SetSetpoint(speed);
		//if value of the error is less than 50 rpm return 1
		return (abs(KickerPID.GetError()) < 50.0);
	}

	int shooterSpeed(double speed) {
		ShooterPID.SetSetpoint(speed);
		//if value of the error is less than 50 rpm return 1
		return (abs(ShooterPID.GetError()) < 50.0);
	}

	void TestPeriodic() {
		lw->Run();
	}

private:
	RobotDrive Adrive, Bdrive;
	frc::LiveWindow* lw = LiveWindow::GetInstance();
	frc::SendableChooser<std::string> chooser, chooseEncoder;
	const std::string autonNameOFF = "0 OFF";
	const std::string autonNameBlue1 = "Blue 1";
	const std::string autonNameBlue2 = "Blue 2";
	const std::string autonNameBlue3 = "Blue 3";
	const std::string autonNameRed1 = "Red 1";
	const std::string autonNameRed2 = "Red 2";
	const std::string autonNameRed3 = "Red 3";
	const std::string RH_Encoder = "RH_Encoder";
	const std::string LH_Encoder = "LH_Encoder";
	std::string autoSelected, encoderSelected;
	Joystick Drivestick;
	Joystick OperatorStick;
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
	//tells us what state we are in in each auto mode
	int modeState;bool AutonOverride, AutoSw1, AutoSw2, AutoSw3;
	DigitalInput DiIn9, DiIn8, DiIn7;
	int AutoVal, AutoVal0, AutoVal1, AutoVal2;
	float OutputX, OutputY;
	std::shared_ptr<NetworkTable> GRIPTable;
	int isWaiting = 0;			/////***** Divide this into 2 variables.

	Solenoid *driveSolenoid = new Solenoid(0);
	//manipulator
	VictorSP Winch0, Winch1;
	VictorSP Shooter0, Shooter1;
	VictorSP Conveyor;
	VictorSP Agitator;
	VictorSP FloorIntakeRoller;
	VictorSP KickerWheel;
	VictorSP DeflectorMotor;
	Solenoid *FloorIntakeArm = new Solenoid(3);
	Solenoid *GearIn = new Solenoid(2);
	Solenoid *GearOut = new Solenoid(1);
	Encoder EncoderKicker;
	Encoder EncoderShoot;
	DigitalInput WinchStop;
	AnalogPotentiometer DeflectorAnglePOT;bool useClosedLoop;
	double DeflectorTarget;
	DigitalInput DeflectorHighLimit, DeflectorLowLimit;

	bool useRightEncoder;

	PIDController DeflectorPID, KickerPID, ShooterPID;

	//MultiSpeedController *Shooter;

}
;

START_ROBOT_CLASS(Robot)

