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

#define BLUE_1_CASE1_FWD (6.8 * 12.0)
#define BLUE_1_CASE2_TURN (90)
#define BLUE_1_CASE3_FWD (7.6 * -1 * 12.0)
#define BLUE_1_CASE4_FWD_TIME (5.0)
#define BLUE_1_CASE4_FWD_LEFT_SPD (0.5)
#define BLUE_1_CASE4_FWD_RIGHT_SPD (0.5)
#define BLUE_1_CASE5_FWD (3.5 * 12.0)
#define BLUE_1_CASE6_TURN (120)
#define BLUE_1_CASE7_FWD (6.5 * 12.0)
#define BLUE_1_CASE8_TIME (0.3)
#define BLUE_1_CASE8_LSPEED (-0.4)
#define BLUE_1_CASE8_RSPEED (-0.4)

// This will go to the gear in front of the middle start position.
//		This is timed so that collision with the airship will put the robot in position.
//		This can be supported with vision when it is ready.
//
// go forward for .75 seconds at .9 speed
#define BLUE_2_CASE2_TIME (0.85)
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
#define RED_1_CASE1_FWD (6.8 * 12.0)
#define RED_1_CASE2_TURN (-90)
#define RED_1_CASE3_FWD (7.6 * -1 * 12.0)
#define RED_1_CASE4_FWD_TIME (5.0)
#define RED_1_CASE4_FWD_LEFT_SPD (0.5)
#define RED_1_CASE4_FWD_RIGHT_SPD (0.5)
#define RED_1_CASE5_FWD (3.5 * 12.0)
#define RED_1_CASE6_TURN (-120)
#define RED_1_CASE7_FWD (6.5 * 12.0)
#define RED_1_CASE8_TIME (0.3)
#define RED_1_CASE8_LSPEED (-0.4)
#define RED_1_CASE8_RSPEED (-0.4)

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
//#define RED_3_CASE3_FWD (2 * 12)
#define RED_3_CASE3_TIME (0.1)
#define RED_3_CASE3_LSPEED (-0.75)
#define RED_3_CASE3_RSPEED (-0.75)

//linear calibrations
#define LINEAR_TOLERANCE (0.2)
// This is the gain for using the encoders to set the distance
//		while going straight.
#define KP_LINEAR (0.27)
// This is the gain for using the gyroscope to go straight
#define KP_ROTATION (0.012)
#define LINEAR_SETTLING_TIME (0.1)
#define LINEAR_MAX_DRIVE_SPEED (0.75)

//turning calibrations
#define ROTATIONAL_TOLERANCE (5.0)
// This is the gain for turning using the gyroscope
#define ERROR_GAIN (-0.05)
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
			Adrive(DriveLeft0, DriveRight0), Drivestick(0), OperatorStick(1), DriveLeft0(
					0), DriveLeft1(1), DriveLeft2(2), DriveRight0(3), DriveRight1(
					4), DriveRight2(5), EncoderLeft(0, 1), EncoderRight(2, 3), table(
			NULL), ahrs(NULL), modeState(0), DiIn9(9), DiIn8(8), DiIn7(7), Winch0(
					11), Winch1(9), Shooter0(12), Shooter1(7), Conveyor(13), Agitator0(
					6), Agitator1(15), FloorIntakeRoller(14), KickerWheel(8), DeflectorMotor(
					10), EncoderKicker(20, 21), EncoderShoot(4, 5), WinchStop(
					6), DeflectorAnglePOT(0, 270, 0), DeflectorTarget(0), ConvCommandPWM(
					0.1), ShootCommandPWM(0.75), DeflectAngle(145), DeflectorHighLimit(
					22), DeflectorLowLimit(23), DeflectorPID(-0.08, 0.0, 0.0,
					&DeflectorAnglePOT, &DeflectorMotor), KickerPID(0.003, 0.0,
					0.0, &EncoderKicker, &KickerWheel), ShooterPID(0.0, 0.0,
					-0.003, 0.0, &EncoderShoot, &Shooter0), DrivePID(0.0, 0.0,
					0.0, 0.0, &EncoderRight, &DriveRight0) {

		//GRIPTable = NetworkTable::GetTable("GRIP/myContuorsReport");
		//Shooter = new MultiSpeedController();

	}

	void RobotInit() {
		//setup smartDashboard choosers
		chooseAutonSelector.AddDefault(AutonNameSwitch, AutonNameSwitch);
		chooseAutonSelector.AddObject(autonNameOFF, autonNameOFF);
		chooseAutonSelector.AddObject(autonNameRed1, autonNameRed1);
		chooseAutonSelector.AddObject(autonNameRed2, autonNameRed2);
		chooseAutonSelector.AddObject(autonNameRed3, autonNameRed3);
		chooseAutonSelector.AddObject(autonNameBlue1, autonNameBlue1);
		chooseAutonSelector.AddObject(autonNameBlue2, autonNameBlue2);
		chooseAutonSelector.AddObject(autonNameBlue3, autonNameBlue3);
		frc::SmartDashboard::PutData("Auto Modes", &chooseAutonSelector);

		chooseDriveEncoder.AddDefault(LH_Encoder, LH_Encoder);
		chooseDriveEncoder.AddObject(RH_Encoder, RH_Encoder);
		frc::SmartDashboard::PutData("Encoder", &chooseDriveEncoder);

		chooseDeflector.AddDefault(chooserClosedLoop, chooserClosedLoop);
		chooseDeflector.AddDefault(chooserOpenLoop, chooserOpenLoop);
		frc::SmartDashboard::PutData("Deflector", &chooseDeflector);

		chooseKicker.AddObject(chooserOpenLoop, chooserOpenLoop);
		chooseKicker.AddObject(chooserClosedLoop, chooserClosedLoop);
		frc::SmartDashboard::PutData("Kicker", &chooseKicker);

		chooseShooter.AddDefault(chooserOpenLoop, chooserOpenLoop);
		chooseShooter.AddObject(chooserClosedLoop, chooserClosedLoop);
		frc::SmartDashboard::PutData("Shooter", &chooseShooter);

		chooseDeflectorLimit.AddDefault(Enable, Enable);
		chooseDeflectorLimit.AddObject(Disable, Disable);
		frc::SmartDashboard::PutData("Deflector Limits", &chooseDeflectorLimit);

		// Inialize settings from Smart Dashboard
		ShootCommandPWM = 0.8;
		ShootCommandRPM = 2800;
		ShootKP = -0.003;
		ShootKI = 0.0;
		ShootKD = 0.0;
		ShootKF = -1.0 / 3200.0; //   1 / MAX RPM
		KickerCommandPWM = 0.75;
		KickerCommandRPM = 500;
		DeflectorTarget = 163;  // Default Angle (Degrees)
		ConvCommandPWM = 0.75;
		AgitatorCommandPWM = 0.75;
		IntakeCommandPWM = 0.75;
		autoBackupDistance = -2.0;

		SmartDashboard::PutNumber("IN: Shooter CMD (PWM)", ShootCommandPWM);
		SmartDashboard::PutNumber("IN: Shooter CMD (RPM)", ShootCommandRPM);
		SmartDashboard::PutNumber("IN: ShootKP", ShootKP);
		SmartDashboard::PutNumber("IN: ShootKI", ShootKI);
		SmartDashboard::PutNumber("IN: ShootKD", ShootKD);
		SmartDashboard::PutNumber("IN: ShootKF", ShootKF);
		SmartDashboard::PutNumber("IN: Kicker CMD (PWM)", KickerCommandPWM);
		SmartDashboard::PutNumber("IN: Kicker CMD (RPM)", KickerCommandRPM);
		SmartDashboard::PutNumber("IN: Deflector CMD (DEG)", DeflectorTarget);
		SmartDashboard::PutNumber("IN: Conveyor CMD (PWM)", ConvCommandPWM);
		SmartDashboard::PutNumber("IN: Agitator CMD (PWM)", AgitatorCommandPWM);
		SmartDashboard::PutNumber("IN: Intake CMD (PWM)", IntakeCommandPWM);
		SmartDashboard::PutNumber("IN: Auto Backup Distance (Inch)",
				autoBackupDistance);

		//turn off shifter solenoids
		driveSolenoid->Set(false);

		//disable drive watchdogs ADLAI - Is this something we should be doing?
		Adrive.SetSafetyEnabled(false);

		//changes these original negative values to positive values
		EncoderLeft.SetReverseDirection(true);
		EncoderRight.SetReverseDirection(false);
		EncoderShoot.SetReverseDirection(true);
		EncoderKicker.SetReverseDirection(true);

		//calibrations for encoders
		EncoderLeft.SetDistancePerPulse(98.0 / 3125.0 * 4.0);
		EncoderRight.SetDistancePerPulse(98.0 / 3125.0 * 4.0);
		EncoderShoot.SetDistancePerPulse(1.0 / 3328.0 * 4.0);
		EncoderKicker.SetDistancePerPulse(1.0 / 4122.0 * 4.0);

		//configure PIDs
		DeflectorPID.SetOutputRange(-0.1, 0.2);

		//drive command averaging filter ADLAI - Does this just make sure joysticks are reading 0? Should it be both here and in teleopinit?
		OutputX = 0, OutputY = 0;

		//variable that chooses which encoder robot is reading for autonomous mode
		useRightEncoder = true;

		// Turn off the the sensors/reset
		if (DeflectorClosedLoop) {
			DeflectorPID.Enable();
		} else {
			DeflectorPID.Disable();
		}

		if (KickerClosedLoop) {
			KickerPID.Enable();
		} else {
			KickerPID.Disable();
		}

		if (ShooterClosedLoop) {
			ShooterPID.Enable();
		} else {
			ShooterPID.Disable();
		}

		//determines that a sensor is being read for either displacement or velocity
		DeflectorAnglePOT.SetPIDSourceType(PIDSourceType::kDisplacement);
		EncoderKicker.SetPIDSourceType(PIDSourceType::kRate);
		EncoderShoot.SetPIDSourceType(PIDSourceType::kRate);

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

		DeflectorPID.Reset();

		//forces robot into low gear
		driveSolenoid->Set(false);

		//makes sure gear doesn't eject
		GearOut->Set(false);

	}

	void TeleopInit() {

		OutputX = 0, OutputY = 0;
	}

	void RobotPeriodic() {
		//links multiple motors together
		Winch1.Set(-Winch0.Get());
		Shooter1.Set(-Shooter0.Get());
		DriveLeft1.Set(DriveLeft0.Get());
		DriveLeft2.Set(DriveLeft0.Get());
		DriveRight1.Set(DriveRight0.Get());
		DriveRight2.Set(DriveRight0.Get());
		Agitator1.Set(Agitator0.Get());

		//Read Auton Switch
		AutoSw1 = DiIn7.Get();
		AutoSw2 = DiIn8.Get();
		AutoSw3 = DiIn9.Get();

		AutoVal = !AutoSw1 * 1 + !AutoSw2 * 2 + !AutoSw3 * 4;

		// Select Auto Program
		autoSelected = chooseAutonSelector.GetSelected();
		if (autoSelected == AutonNameSwitch) {
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
			case 7:
				autoSelected = autonNameRed2;
				break;
			default:
				autoSelected = autonNameOFF;
			}
		}


		//enables deflector PID
		if (DeflectorClosedLoop) {
			DeflectorPID.Enable();
		} else {
			DeflectorPID.Disable();
		}

		//displays sensor and motor info to smartDashboard
		SmartDashboardUpdate();
	}
	void DisabledPeriodic() {
		//SmartDashboardUpdate();
	}
	void AutonomousPeriodic() { // ADLAI - This stuff probably shouldn't be in here, we shouldn't need to change program after autonomous begins?
		if (autoSelected == autonNameRed1)
			autoRed1();
		else if (autoSelected == autonNameRed2)
			autoForward();
		else if (autoSelected == autonNameRed3)
			autoRed3();
		else if (autoSelected == autonNameBlue1)
			autoBlue1();
		else if (autoSelected == autonNameBlue2)
			autoMiddleGearEject();
		else if (autoSelected == autonNameBlue3)
			autoBlue3();
		else
			stopMotors();

		//SmartDashboardUpdate();
	}

	void TeleopPeriodic() {
		double Deadband = 0.11;
		double DriveCreepSpeed = 0.5;

		//high gear & low gear controls
		if (Drivestick.GetRawButton(6))
			driveSolenoid->Set(true);			// High gear press LH bumper
		if (Drivestick.GetRawButton(5))
			driveSolenoid->Set(false);			// Low gear press RH bumper

		// Temporary high gear when right trigger pushed
		if (Drivestick.GetRawAxis(3) > Deadband) {
			driveSolenoid->Set(true);
			driveRightTriggerPrev = true;
		} else if (driveRightTriggerPrev) {
			driveSolenoid->Set(false);
			driveRightTriggerPrev = false;
		}

		//  Rumble code
		//  Read all motor current from PDP and display on drivers station
		double driveCurrent = pdp->GetTotalCurrent();	// Get total current

		// rumble if current to high
		double LHThr = 0.0;		// Define value for rumble
		if (driveCurrent > 125.0)// Rumble if greater than 125 amps motor current
			LHThr = 0.5;
		Joystick::RumbleType Vibrate;				// define Vibrate variable
		Vibrate = Joystick::kLeftRumble;		// set Vibrate to Left
		Drivestick.SetRumble(Vibrate, LHThr);  	// Set Left Rumble to RH Trigger
		Vibrate = Joystick::kRightRumble;		// set vibrate to Right
		Drivestick.SetRumble(Vibrate, LHThr);// Set Right Rumble to RH Trigger

		//  Read climber motor current from PDP and display on drivers station
		double climberCurrentLeft = pdp->GetCurrent(3);
		double climberCurrentRight = pdp->GetCurrent(12);
		SmartDashboard::PutNumber("DBG climber current", climberCurrentLeft);

		// rumble if current to high
		double LHClimb = 0.0;		// Define value for rumble
		double climberMaxCurrent = 30.0;//needs to be changed... 30.0 is a guess
		if (climberCurrentLeft > climberMaxCurrent)	// Rumble Left if greater than climberMaxCurrent
			LHClimb = 0.5;
		Vibrate = Joystick::kLeftRumble;		// set Vibrate to Left
		OperatorStick.SetRumble(Vibrate, LHClimb); // Set Left Rumble to LH Trigger

		double RHClimb = 0.0;		// Define value for rumble
		if (climberCurrentRight > climberMaxCurrent)// Rumble Right if greater than climberMaxCurrent
			RHClimb = 0.5;
		Vibrate = Joystick::kRightRumble;		// set vibrate to Right
		OperatorStick.SetRumble(Vibrate, RHClimb);// Set Right Rumble to RH Trigger

		//drive controls ADLAI - I know this works, I just don't understand how returning only negative values works for this.
		double SpeedLinear = Drivestick.GetRawAxis(1) * -1; // get Yaxis value (forward)
		double SpeedRotate = Drivestick.GetRawAxis(4) * -1; // get Xaxis value (turn)

		// Set dead band for X and Y axis
		if (fabs(SpeedLinear) < Deadband)
			SpeedLinear = 0.0;
		if (fabs(SpeedRotate) < Deadband)
			SpeedRotate = 0.0;

		//Reduce turn speed when left trigger is pressed
		if (Drivestick.GetRawAxis(2) > Deadband) {
			SpeedLinear = SpeedLinear * DriveCreepSpeed;  // Reduce turn speed
			SpeedRotate = SpeedRotate * DriveCreepSpeed;  // Reduce drive speed
		}

		//slow down direction changes from 1 cycle to 5
		OutputY = (0.8 * OutputY) + (0.2 * SpeedLinear);
		OutputX = (0.8 * OutputX) + (0.2 * SpeedRotate);

		//drive
		if (Drivestick.GetRawButton(4)) {
			//boiler auto back up when y button pushed
			if (!driveButtonYPrev) {
				EncoderRight.Reset();
				EncoderLeft.Reset();
				ahrs->ZeroYaw();
				driveButtonYPrev = true;
			}
			forward(autoBackupDistance);
		} else {
			//manual control
			driveButtonYPrev = false;
			Adrive.ArcadeDrive(OutputY, OutputX, true);
		}


		/*
		 * MANIP CODE
		 */

		// Turn on the shooter when right hand trigger is pushed
		if (OperatorStick.GetRawAxis(3) > Deadband) {
			if (!operatorRightTriggerPrev) {
				ShooterDelay.Reset();
				ShooterDelay.Start();

				if (ShooterClosedLoop) {
					ShooterPID.Reset();
					ShooterPID.Enable();
				}
				operatorRightTriggerPrev = true;
			}
			if (ShooterClosedLoop) {
				//1.75 is a scaling factor to make the PID reach desired RPM
				ShooterPID.SetSetpoint(ShootCommandRPM / 60.0);
			} else {
				Shooter0.Set(-ShootCommandPWM); // negative so they turn the correct way.
			}

		} else {
			operatorRightTriggerPrev = false;
			if (ShooterClosedLoop) {
				ShooterPID.SetSetpoint(0.0); // ADLAI - this might be redundant with the disable but I don't think it matters.
				ShooterPID.Disable();
			} else {
				Shooter0.Set(0.0);
			}
		}

		// Turn on Kicker Wheel, conveyor, and agitators when right trigger is pressed
		if (OperatorStick.GetRawAxis(3) > Deadband
				and ShooterDelay.Get() > 0.2) {

			Conveyor.Set(OperatorStick.GetRawAxis(3));
			Agitator0.Set(OperatorStick.GetRawAxis(3));

			if (KickerClosedLoop) {
				KickerPID.SetSetpoint(KickerCommandRPM / 60.0);
				KickerPID.Enable();
			} else {
				KickerWheel.Set(KickerCommandPWM);
			}
		} else {

			Conveyor.Set(0.0);
			Agitator0.Set(0.0);

			if (KickerClosedLoop) {
				KickerPID.SetSetpoint(0.0);
				KickerPID.Disable();
			} else {
				KickerWheel.Set(0.0);
			}
		}

		//Spin intake when left trigger is pushed
		if (OperatorStick.GetRawAxis(2) > Deadband) {
			FloorIntakeRoller.Set(OperatorStick.GetRawAxis(2));
		} else {
			FloorIntakeRoller.Set(0.0);
		}

		// RH Bumper - Retract Intake
		if (OperatorStick.GetRawButton(6)) {
			intakeDeployed = false;
		}
		// LH Bumper - Deploy Intake
		if (OperatorStick.GetRawButton(5)) {
			intakeDeployed = true;
		}

		//A button for intake un-jam
		if (OperatorStick.GetRawButton(1))
			FloorIntakeArm->Set(!intakeDeployed);
		else
			FloorIntakeArm->Set(intakeDeployed);

		//X Button to get and B button release the gear
		GearIn->Set(OperatorStick.GetRawButton(2));
		GearOut->Set(OperatorStick.GetRawButton(3));

		//Right joystick for agitator un-jam
		if (fabs(OperatorStick.GetRawAxis(4)) > Deadband)
			Agitator0.Set(OperatorStick.GetRawAxis(4));

		//turn on winch using the left joystick
		if (OperatorStick.GetRawAxis(1) > Deadband) {
			Winch0.Set(OperatorStick.GetRawAxis(1));
		} else {
			Winch0.Set(0.0);
		}

		//move deflector using d-pad(POV)
		double DeflectorLimitLower = 120.0;	//degrees
		double DeflectorLimitUpper = 183.0;	//degrees
		double DeflectorMotorOutputMax = 0.1;	//PWM
		double DeflectorIncrement = 0.03;	//degrees

		if (!DeflectorClosedLoop) {
			//move deflector up
			if (DeflectorAnglePOT.Get() < DeflectorLimitUpper
					&& OperatorStick.GetPOV(0) == 0) {
				DeflectorMotor.Set(DeflectorMotorOutputMax);
			}
			//move deflector down
			else if (DeflectorAnglePOT.Get() > DeflectorLimitLower
					&& OperatorStick.GetPOV(0) == 180) {
				DeflectorMotor.Set(-DeflectorMotorOutputMax);
			} else {
				DeflectorMotor.Set(0.0);
			}
		}
		//in closed loop
		else {
			if (OperatorStick.GetPOV(0) == 0
					and DeflectorTarget < DeflectorLimitUpper) {
				//will increment the DeflectorTarget by value set by joysticks
				DeflectorTarget += DeflectorIncrement;
			} else if (OperatorStick.GetPOV(0) == 180
					and DeflectorTarget > DeflectorLimitLower) {
				//will increment the DeflectorTarget by value set by joysticks
				DeflectorTarget -= DeflectorIncrement;
			}

			//Control the angle of the deflector
			if (OperatorStick.GetPOV(0) == 90) {
				DeflectorTarget = 163.0;
			}
			if (OperatorStick.GetPOV(0) == 270) {
				DeflectorTarget = 135.0;
			}

			DeflectorPID.SetSetpoint(DeflectorTarget);

			// Emergency Disable
			if (DeflectorAnglePOT.Get() > DeflectorLimitUpper
					or DeflectorAnglePOT.Get() < DeflectorLimitLower) {
				DeflectorPID.Disable();
				DeflectorMotor.Set(0.0);
			}
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
#define AB1_TIME_FWD 9
#define AB1_SHOOT 10
#define AB1_END 11

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
			//possibly will take out if can shoot from hopper
		case AB1_TO_BOILER:
			//go forward 7-ish feet to run into boiler
			//change to timed drive
			if (forward(BLUE_1_CASE7_FWD)) {
				modeState = AB1_TIME_FWD;
				ahrs->ZeroYaw();
			}
			break;
		case AB1_TIME_FWD:
			//go forward 7-ish feet to run into boiler
			//change to timed drive
			if (timedDrive(BLUE_1_CASE8_TIME, BLUE_1_CASE8_LSPEED,
					BLUE_1_CASE8_RSPEED)) {
				modeState = AB1_SHOOT;
				ahrs->ZeroYaw();
			}
			break;
		case AB1_SHOOT:
			if (shoot()) {
				modeState = AB1_END;
			}
			break;
		default:
			stopMotors();
		}
		return;
	}

#define AB1A_INIT 1
#define AB1A_FWD 2
#define AB1A_TURN90 3
#define AB1A_BKUP 4
#define AB1A_WAIT 5
#define AB1A_FWD2 6
#define AB1A_FACE_BOILER 7
#define AB1A_SHOOT 9
#define AB1A_END 10

	void autoBlue1A(void) {

		//Blue boiler side code
		//drives turns then drives again
		switch (modeState) {
		case AB1A_INIT:
			modeState = AB1A_FWD;
			break;
		case AB1A_FWD:
			// go forward 7 ft
			if (forward(BLUE_1_CASE1_FWD)) {
				modeState = AB1A_TURN90;
				ahrs->ZeroYaw();
			}
			break;
		case AB1A_TURN90:
			// turn 90 degrees clockwise
			if (autonTurn(BLUE_1_CASE2_TURN)) {
				//modeState = AB1_BKUP;
				modeState = AB1A_WAIT;
				EncoderLeft.Reset();
				EncoderRight.Reset();
				ahrs->ZeroYaw();
			}
			break;
//		case AB1A_BKUP:
//			// go forward 7 ft to hit hopper
//			//change to timed drive
//			if (forward(BLUE_1_CASE3_FWD)) {
//				modeState = AB1_WAIT;
//				AutonTimer.Reset();
//			}
//			break;
		case AB1A_WAIT:
			//waits a couple of seconds for balls
			if (timedDrive(BLUE_1_CASE4_FWD_TIME, BLUE_1_CASE4_FWD_LEFT_SPD,
			BLUE_1_CASE4_FWD_RIGHT_SPD)) {
				modeState = AB1A_FWD2;
				EncoderLeft.Reset();
				EncoderRight.Reset();
				ahrs->ZeroYaw();
			}
			break;
		case AB1A_FWD2:
			//go backward 4-ish feet
			if (forward(BLUE_1_CASE5_FWD)) {
				modeState = AB1A_FACE_BOILER;
				ahrs->ZeroYaw();
			}
			break;
		case AB1A_FACE_BOILER:
			// turn enough degrees to face boiler
			if (autonTurn(BLUE_1_CASE6_TURN)) {
				modeState = AB1A_SHOOT;
				EncoderLeft.Reset();
				EncoderRight.Reset();
				ahrs->ZeroYaw();
			}
			break;
		case AB1A_SHOOT:
			//launch
			modeState = AB1A_END;
			break;
		default:
			stopMotors();
		}
		return;
	}

#define AB2_INIT 1
#define AB2_FWD 2
#define	AB2_TIMED 3
#define AB2_GEAR_WAIT 4
#define AB2_GEAR 5
#define AB2_BACK 6
#define AB2_END 7
	void autoMiddleGearEject(void) {
		//blue side code
		//goes forward to put gear on pin
		switch (modeState) {
		case AB2_INIT:
			// This uses state 1 for initialization.
			// This keeps the initialization and the code all in one place.
			ahrs->ZeroYaw();
			modeState = AB2_FWD;
			break;
		case AB2_FWD:
			if (forward(71.0)) {
				//timedDrive(BLUE_2_CASE2_TIME, BLUE_2_CASE2_LSPEED,
				//BLUE_2_CASE2_RSPEED)
				//if (forward7ft(-0.4, 7 * 12.0)) {
				AutonTimer.Reset();
				modeState = AB2_TIMED;
			}
			break;
		case AB2_TIMED:
			if (timedDrive(1.0, -0.2, -0.2)) {

				//if (forward7ft(-0.4, 7 * 12.0)) {
				AutonTimer.Reset();
				modeState = AB2_GEAR_WAIT;
			}
			break;
		case AB2_GEAR_WAIT:
			if (AutonTimer.Get() > 2.5) {
				modeState = AB2_GEAR;
			}
			break;
		case AB2_GEAR:
			if (1) {
				//change to timed drive
				//if (forward7ft(-0.4, 7 * 12.0)) {
				GearOut->Set(true);
				modeState = AB2_BACK;
			}
			break;
		case AB2_BACK:
			if (timedDrive(5.0, 0.3, 0.3)) {
				//if (forward7ft(-0.4, 7 * 12.0)) {
				GearOut->Set(false);
				AutonTimer.Reset();
				modeState = AB2_END;
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
#define AB3_GEAR_WAIT 5
#define AB3_GEAR 6
#define AB3_END 7
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
			if (timedDrive(BLUE_3_CASE3_TIME, BLUE_3_CASE3_LSPEED,
			BLUE_3_CASE3_RSPEED)) {
				modeState = AB3_GEAR_WAIT;
				EncoderLeft.Reset();
				EncoderRight.Reset();
			}
			break;
		case AB3_GEAR_WAIT:
			// go forward
			//timed drive
			if (AutonTimer.Get() > 0.5) {
				modeState = AB3_GEAR;
			}
			break;
		case AB3_GEAR:
			// deploy gear
			if (1) {
				modeState = AB3_END;
				EncoderLeft.Reset();
				EncoderRight.Reset();
				GearOut->Set(true);
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
#define	AR1_TIME_FWD 9
#define AR1_SHOOT 10
#define AR1_END 11
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
				modeState = AR1_WAIT;
				EncoderLeft.Reset();
				EncoderRight.Reset();
				ahrs->ZeroYaw();
			}
			break;
//		case AR1_BKUP:
//			//change to timed drive
//			// go forward 7 ft to hit hopper
//			if (forward(RED_1_CASE3_FWD)) {
//				modeState = AR1_WAIT;
//				AutonTimer.Reset();
//			}
//			break;
		case AR1_WAIT:
			//waits in front of hopper a couple of seconds for balls
			if (timedDrive(RED_1_CASE4_FWD_TIME, RED_1_CASE4_FWD_LEFT_SPD,
			RED_1_CASE4_FWD_RIGHT_SPD)) {
				modeState = AR1_FWD2;
				EncoderLeft.Reset();
				EncoderRight.Reset();
				ahrs->ZeroYaw();
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
				ahrs->ZeroYaw();
			}
			break;
		case AR1_TO_BOILER:
			//change to timed drive
			//go forward 7-ish feet to run into boiler
			if (forward(RED_1_CASE7_FWD)) {
				modeState = AR1_TIME_FWD;
				ahrs->ZeroYaw();
			}
			break;
		case AR1_TIME_FWD:
			//change to timed drive
			//go forward 7-ish feet to run into boiler
			if (timedDrive(RED_1_CASE8_TIME, RED_1_CASE8_LSPEED,
					RED_1_CASE8_RSPEED)) {
				modeState = AR1_SHOOT;
				ahrs->ZeroYaw();
			}
			break;
		case AR1_SHOOT:
			if (shoot()) {
				modeState = AR1_END;
			}
			break;
		default:
			stopMotors();
		}
		return;
	}

#define AR1A_INIT 1
#define AR1A_FWD 2
#define AR1A_TURN90 3
#define AR1A_BKUP 4
#define AR1A_WAIT 5
#define AR1A_FWD2 6
#define AR1A_FACE_BOILER 7
#define AR1A_SHOOT 9
#define AR1A_END 10

	void autoRed1A(void) {
		//Red center position code
		//this version turns the robot in a right angle

		switch (modeState) {
		case AR1A_INIT:
			modeState = AR1A_FWD;
			break;
		case AR1A_FWD:
			// go forward 7 ft
			if (forward(RED_1_CASE1_FWD)) {
				modeState = AR1A_TURN90;
				ahrs->ZeroYaw();
			}
			break;
		case AR1A_TURN90:
			// turn 90 degrees counterclockwise
			if (autonTurn(RED_1_CASE2_TURN)) {
				modeState = AR1A_BKUP;
				EncoderLeft.Reset();
				EncoderRight.Reset();
			}
			break;
		case AR1A_BKUP:
			//change to timed drive
			// go forward 7 ft to hit hopper
			if (forward(RED_1_CASE3_FWD)) {
				modeState = AR1A_WAIT;
				AutonTimer.Reset();
			}
			break;
		case AR1A_WAIT:
			//waits in front of hopper a couple of seconds for balls
			if (timedDrive(RED_1_CASE4_FWD_TIME, RED_1_CASE4_FWD_LEFT_SPD,
			RED_1_CASE4_FWD_RIGHT_SPD)) {
				modeState = AR1A_FWD2;

			}
			break;
		case AR1A_FWD2:
			//go backward 3-ish feet
			if (forward(RED_1_CASE5_FWD)) {
				modeState = AR1A_FACE_BOILER;
				ahrs->ZeroYaw();
			}
			break;
		case AR1A_FACE_BOILER:
			// turns counterclockwise enough degrees to face boiler
			if (autonTurn(RED_1_CASE6_TURN)) {
				modeState = AR1A_END;
				EncoderLeft.Reset();
				EncoderRight.Reset();
			}
			break;
		case AR1A_SHOOT:
			//launch
			modeState = AR1A_END;
			break;
		default:
			stopMotors();
		}
		return;
	}

#define AR2_INIT 1
#define AR2_FWD 2
#define AR2_TIMED 3
#define AR2_END 4
	void autoForward(void) {
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
			if (forward(71.0)) {
				//timedDrive(BLUE_2_CASE2_TIME, BLUE_2_CASE2_LSPEED,
				//BLUE_2_CASE2_RSPEED)
				//if (forward7ft(-0.4, 7 * 12.0)) {
				AutonTimer.Reset();
				modeState = AR2_TIMED;
			}
			break;
		case AR2_TIMED:
			if (timedDrive(1.0, -0.2, -0.2)) {
				//if (forward7ft(-0.4, 7 * 12.0)) {
				AutonTimer.Reset();
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
#define AR3_GEAR 5
#define AR3_END 6
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
				modeState = AR3_GEAR;
				EncoderLeft.Reset();
				EncoderRight.Reset();
			}
			break;
//		case AR3_STR8:
//			//change to timed drive
//			// go forward
//			if (forward(RED_3_CASE3_FWD)) {
//				modeState = AR3_GEAR;
//				EncoderLeft.Reset();
//				EncoderRight.Reset();
//			}
//			break;
		case AR3_GEAR:
			//change to timed drive
			// go forward
			if (1) {
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
	void SmartDashboardUpdate() {

		// Auto State
		SmartDashboard::PutNumber("Auto Switch (#)", AutoVal);
		SmartDashboard::PutString("Auto Program", autoSelected);
		SmartDashboard::PutNumber("Auto State (#)", modeState);
		SmartDashboard::PutNumber("Auto Timer (s)", AutonTimer.Get());

		// Drive Encoders
		SmartDashboard::PutNumber("Drive Encoder Left (RAW)",
				EncoderLeft.GetRaw());
		SmartDashboard::PutNumber("Drive Encoder Left (Inches)",
				EncoderLeft.GetDistance());

		SmartDashboard::PutNumber("Drive Encoder Right (RAW)",
				EncoderRight.GetRaw());
		SmartDashboard::PutNumber("Drive Encoder Right (Inch)",
				EncoderRight.GetDistance());

		encoderSelected = chooseDriveEncoder.GetSelected();
		useRightEncoder = (encoderSelected == RH_Encoder);

		autoBackupDistance = SmartDashboard::GetNumber(
				"IN: Auto Backup Distance (Inch)", autoBackupDistance);
		SmartDashboard::PutNumber("Auto Backup Distance", autoBackupDistance);

		// Gyro
		if (ahrs) {
			double gyroAngle = ahrs->GetAngle();
			SmartDashboard::PutNumber("Gyro Angle", gyroAngle);
		} else {
			SmartDashboard::PutNumber("Gyro Angle", 999);
		}

		/*
		 * Manipulator
		 */

		//shooter
		SmartDashboard::PutNumber("Shooter Encoder (RAW)",
				EncoderShoot.GetRaw());
		SmartDashboard::PutNumber("Shooter Encoder (RPM)",
				EncoderShoot.GetRate() * 60.0);
		SmartDashboard::PutNumber("Shooter Encoder (REV)",
				EncoderShoot.GetDistance());
		ShootCommandPWM = SmartDashboard::GetNumber("IN: Shooter CMD (PWM)",
				ShootCommandPWM);
		ShootCommandRPM = SmartDashboard::GetNumber("IN: Shooter CMD (RPM)",
				ShootCommandRPM);
		SmartDashboard::PutNumber("Shooter CMD (PWM)", ShootCommandPWM);
		SmartDashboard::PutNumber("Shooter CMD (RPM)", ShootCommandRPM);
		//Shooter PID
		ShootKP = SmartDashboard::GetNumber("IN: ShootKP", ShootKP);
		ShootKI = SmartDashboard::GetNumber("IN: ShootKI", ShootKI);
		ShootKD = SmartDashboard::GetNumber("IN: ShootKD", ShootKD);
		ShootKF = SmartDashboard::GetNumber("IN: ShootKF", ShootKF);
		SmartDashboard::PutNumber("ShootKP", ShootKP);
		SmartDashboard::PutNumber("ShootKI", ShootKI);
		SmartDashboard::PutNumber("ShootKD", ShootKD);
		SmartDashboard::PutNumber("ShootKF", ShootKF);
		ShooterPID.SetPID(ShootKP, ShootKI, ShootKD, ShootKF);

		//Kicker
		SmartDashboard::PutNumber("Kicker Encoder (RAW)",
				EncoderKicker.GetRaw());
		SmartDashboard::PutNumber("Kicker Encoder (RPM)",
				EncoderKicker.GetRate() * 60.0);
		SmartDashboard::PutNumber("Kicker Encoder (REV)",
				EncoderKicker.GetDistance());
		KickerCommandPWM = SmartDashboard::GetNumber("IN: Kicker CMD (PWM)",
				KickerCommandPWM);
		KickerCommandRPM = SmartDashboard::GetNumber("IN: Kicker CMD (RPM)",
				KickerCommandRPM);
		SmartDashboard::PutNumber("Kicker CMD (PWM)", KickerCommandPWM);
		SmartDashboard::PutNumber("Kicker CMD (RPM)", KickerCommandRPM);
		KickerPID.SetPID(ShootKP, ShootKI, ShootKD, ShootKF);

		//winch/climber... whatever you want to call it
		SmartDashboard::PutBoolean("Winch Stop Switch", !WinchStop.Get());

		//deflector
		SmartDashboard::PutBoolean("Deflector Limit Low",
				!DeflectorLowLimit.Get());
		SmartDashboard::PutBoolean("Deflector Limit High",
				!DeflectorHighLimit.Get());
		SmartDashboard::PutNumber("Deflector Angle POT (DEG)",
				DeflectorAnglePOT.Get());

		double deflectorTargetCommand;
		deflectorTargetCommand = SmartDashboard::GetNumber(
				"IN: Deflector CMD (DEG)", DeflectorTarget);
		if (deflectorTargetCommand != deflectorTargetMemory) {
			deflectorTargetMemory = deflectorTargetCommand;
			DeflectorTarget = deflectorTargetCommand;
		}

		SmartDashboard::PutNumber("Deflector CMD (Deg)", DeflectorTarget);

		//Conveyor, Agitator, Intake Settings
		ConvCommandPWM = SmartDashboard::GetNumber("IN: Conveyor CMD (PWM)",
				ConvCommandPWM);
		AgitatorCommandPWM = SmartDashboard::GetNumber("IN: Agitator CMD (PWM)",
				AgitatorCommandPWM);
		IntakeCommandPWM = SmartDashboard::GetNumber("IN: Intake CMD (PWM)",
				IntakeCommandPWM);

		// PWM displays
		SmartDashboard::PutNumber("Drive L0 Output", DriveLeft0.Get());
		SmartDashboard::PutNumber("Drive L1 Output", DriveLeft1.Get());
		SmartDashboard::PutNumber("Drive L2 Output", DriveLeft2.Get());
		SmartDashboard::PutNumber("Drive R0 Output", DriveRight0.Get());
		SmartDashboard::PutNumber("Drive R1 Output", DriveRight1.Get());
		SmartDashboard::PutNumber("Drive R2 Output", DriveRight2.Get());

		SmartDashboard::PutNumber("Shooter Motor0 Output", Shooter0.Get());
		SmartDashboard::PutNumber("Shooter Motor1 Output", Shooter1.Get());
		SmartDashboard::PutNumber("Winch Motor0 Output", Winch0.Get());
		SmartDashboard::PutNumber("Winch Motor1 Output", Winch1.Get());
		SmartDashboard::PutNumber("Kicker Motor Output", KickerWheel.Get());
		SmartDashboard::PutNumber("Agitator Motor0 Output", Agitator0.Get());
		SmartDashboard::PutNumber("Agitator Motor1 Output", Agitator1.Get());
		SmartDashboard::PutNumber("Floor Intake Motor Output",
				FloorIntakeRoller.Get());
		SmartDashboard::PutNumber("Conveyor Motor  Output", Conveyor.Get());
		SmartDashboard::PutNumber("Deflector Motor Output",
				DeflectorMotor.Get());

		//chooser code for manip in open/closed loop
		ShooterClosedLoop = (chooseShooter.GetSelected() == chooserClosedLoop);
		KickerClosedLoop = (chooseKicker.GetSelected() == chooserClosedLoop);
		DeflectorClosedLoop = (chooseDeflector.GetSelected()
				== chooserClosedLoop);
		DeflectorLimitEnabled = (chooseDeflectorLimit.GetSelected() == Enable);

		//**TEST
		SmartDashboard::PutNumber("DBG POV", OperatorStick.GetPOV(0));

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

	void motorSpeed(double leftMotor, double rightMotor) { // ADLAI - Possibly redundant with the previous drive motor setup in robotperiodic
		DriveLeft0.Set(leftMotor * -1);
		DriveLeft1.Set(leftMotor * -1);
		DriveLeft2.Set(leftMotor * -1);
		DriveRight0.Set(rightMotor);
		DriveRight1.Set(rightMotor);
		DriveRight2.Set(rightMotor);
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

	//UNTESTED UNTESTED UNTESTED UNTESTED
	//
	// This implements the timed drive with the direction determined
	//		by the gyro.
	// This points the robot to zero degrees,
	//		so reset the gyro if you want it to go straight.
	//
	// The signs of the motorSpeed may be wrong.  I changed the signs on
	//		leftMotorSpeed and rightMotorSpeed so that positive motor
	//		speeds should send the robot forward.
	// ---> I don't know if the signs on driveCommandRotation are correct.
	//		These signs match the way 'forward' does it.
	// ---> I think KP_ROTATION is correct, because this code comes from 'forward'.
	int timedGyroDrive(double driveTime, double leftMotorSpeed,
			double rightMotorSpeed) {
		float currentTime = AutonTimer.Get();
		if (currentTime < driveTime) {
			double driveCommandRotation = ahrs->GetAngle() * KP_ROTATION;
			motorSpeed(-leftMotorSpeed + driveCommandRotation,
					-rightMotorSpeed - driveCommandRotation);
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

	int shoot() {
		//resets shooter and kicker encoders
		if (ShooterClosedLoop) {
			ShooterPID.Reset();
			ShooterPID.Enable();
		}
		if (KickerClosedLoop) {
			KickerPID.Reset();
			KickerPID.Enable();
		}

		//turn on conveyor, agitators, and kicker
		Conveyor.Set(0.8);
		Agitator0.Set(0.8);

		if (KickerClosedLoop) {
			KickerPID.SetSetpoint(KickerCommandRPM / 60.0);
			KickerPID.Enable();
		} else {
			KickerWheel.Set(KickerCommandPWM);
		}

		//launch
		if (ShooterClosedLoop) {
			//1.75 is a scaling factor to make the PID reach desired RPM
			ShooterPID.SetSetpoint(ShootCommandRPM / 60.0);
		} else {
			Shooter0.Set(-ShootCommandPWM); // negative so they turn the correct way.
		}

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

		if (ShooterClosedLoop) {
			ShooterPID.SetSetpoint(speed);
			//if value of the error is less than 50 rpm return 1
			return (abs(ShooterPID.GetError()) < 50.0);
		}

		return 1; // Not implemented or used anywhere...
	}

	void TestPeriodic() {
		lw->Run();
	}

private:
	RobotDrive Adrive;
	frc::LiveWindow* lw = LiveWindow::GetInstance();
	frc::SendableChooser<std::string> chooseAutonSelector, chooseDriveEncoder,
			chooseDeflector, chooseKicker, chooseShooter, chooseDeflectorLimit;
	const std::string AutonNameSwitch = "Use Switch";
	const std::string autonNameOFF = "0 OFF";
	const std::string autonNameBlue1 = "Blue Boiler";
	const std::string autonNameBlue2 = "Middle Gear Eject";
	const std::string autonNameBlue3 = "Blue Right Side Gear";
	const std::string autonNameRed1 = "Red Boiler";
	const std::string autonNameRed2 = "Go Forward Only";
	const std::string autonNameRed3 = "Red Right Side Gear";
	const std::string RH_Encoder = "RH_Encoder";
	const std::string LH_Encoder = "LH_Encoder";
	const std::string chooserClosedLoop = "Closed Loop";
	const std::string chooserOpenLoop = "Open Loop";
	const std::string Disable = "Disable";
	const std::string Enable = "Enable";
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
	Timer ShooterDelay;
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
	// create pdp variable
	PowerDistributionPanel *pdp = new PowerDistributionPanel();

//manipulator
	VictorSP Winch0, Winch1;
	VictorSP Shooter0, Shooter1;
	VictorSP Conveyor;
	VictorSP Agitator0, Agitator1;
	VictorSP FloorIntakeRoller;
	VictorSP KickerWheel;
	VictorSP DeflectorMotor;
	Solenoid *FloorIntakeArm = new Solenoid(2);
	Solenoid *GearIn = new Solenoid(3);
	Solenoid *GearOut = new Solenoid(1);
	Encoder EncoderKicker;
	Encoder EncoderShoot;
	DigitalInput WinchStop;
	AnalogPotentiometer DeflectorAnglePOT;
	double DeflectorTarget, IntakeCommandPWM, AgitatorCommandPWM,
			ConvCommandPWM, ShootCommandRPM, ShootCommandPWM, DeflectAngle,
			KickerCommandRPM, KickerCommandPWM, ShootKP, ShootKI, ShootKD,
			ShootKF;
	DigitalInput DeflectorHighLimit, DeflectorLowLimit;

	bool useRightEncoder;bool DeflectorClosedLoop;bool KickerClosedLoop;bool ShooterClosedLoop;bool DeflectorLimitEnabled;

	PIDController DeflectorPID, KickerPID, ShooterPID, DrivePID;

	bool driveRightTriggerPrev = false;bool driveButtonYPrev = false;bool operatorRightTriggerPrev =
	false;bool intakeDeployed = false;

	double autoBackupDistance;
	double deflectorTargetMemory;
//MultiSpeedController *Shooter;

}
;

START_ROBOT_CLASS(Robot)

