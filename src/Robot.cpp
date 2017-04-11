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

//Blue 1 - middle gear shoot auto (autoBlueMiddleShoot)
// 1) Initialize for this auto.							Done: Set gear deflector
//															Set floor intake arm
// 2) Move forward 71"									Done: reset timer
//		gyro keeps us straight
//		measured by encoders
// 3) Move forward 0.5 sec at 20% power.				Done: reset timer
//		time measured by timer
// 4) Wait for 0.5 seconds	(hard coded)				Done: 
//		time measured by timer
// 5) Start deploying the gear							Done: reset encoders, 
//		Always terminates the first time though				zero gyro yaw, 
// 6) Back up 48"										Done: Zero yaw
//		gyro keeps us straight
// 		encoders measure distance
// 7) Turn to face the boiler							Done: reset encoders,
//		turn measured by the gyro								Zero yaw
// 8) Go forward ~7 ft.									Done: Zero yaw
//		gyro keeps us straight
// 9) Shoot
//10) Stop
#define BLUE_1_CASE1_FWD (71.0)
#define BLUE_1_CASE2_FWD_TIME (0.5)
#define BLUE_1_CASE2_FWD_LEFT_SPD (-0.2)
#define BLUE_1_CASE2_FWD_RIGHT_SPD (-0.2)
#define BLUE_1_CASE5_BACK (-48.0)
#define BLUE_1_CASE6_TURN (100.0)
#define BLUE_1_CASE7_FWD (-60.0)

//drives along the key line into the hopper, backs up, turns, runs into the boiler and shoots
// (autoBlueKeyShoot)
// 1) Initialize for this auto.							Done: Set gear deflector
//															Set floor intake arm
// 2) Move forward 48"									Done: reset timer	////////Why is this negative???????????
//		gyro keeps us straight
//		measured by encoders
// 3) Make a wide turn to the hopper					Done: Reset timer
//		timer-based dead reckoning
#define BLUE_1A_CASE1_FWD (-4.0 * 12.0)
#define BLUE_1A_CASE2_HOPPER_TIME (2.2)
#define BLUE_1A_CASE2_LSPEED (0.5)
#define BLUE_1A_CASE2_RSPEED (0.6)

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
#define BLUE_3_CASE2_TURN (60.0)
#define BLUE_3_CASE3_STR8 (36.0)
#define BLUE_3_CASE6_BACK (-48.0)
#define BLUE_3_CASE7_TURN (-190.0)

//left side gear calibrations
#define BLUE_4_CASE1_FWD (6.7 * 12.0)
#define BLUE_4_CASE2_TURN (60.0)
#define BLUE_4_CASE3_STR8 (56.0)
#define BLUE_4_CASE6_BACK (-48.0)

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
#define RED_1_CASE1_FWD (71.0)
#define RED_1_CASE2_FWD_TIME (0.5)
#define RED_1_CASE2_FWD_LEFT_SPD (-0.2)
#define RED_1_CASE2_FWD_RIGHT_SPD (-0.2)
#define RED_1_CASE5_BACK (-48.0)
#define RED_1_CASE6_TURN (-100.0)
#define RED_1_CASE7_FWD (-60.0)

//drives along the key line into the hopper, backs up, turns, runs into the boiler and shoots
#define RED_1A_CASE1_FWD (-4.0 * 12.0)
#define RED_1A_CASE2_HOPPER_TIME (2.2)
#define RED_1A_CASE2_LSPEED (0.6)
#define RED_1A_CASE2_RSPEED (0.5)

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
#define RED_3_CASE2_TURN (-60)
#define RED_3_CASE3_STR8 (34.0)
#define RED_3_CASE6_BACK (-48.0)
#define RED_3_CASE7_TURN (190.0)

//right side gear auto calibrations
#define RED_4_CASE1_FWD (6.0 * 12.0)
#define RED_4_CASE2_TURN (-60.0)
#define RED_4_CASE3_STR8 (56.0)
#define RED_4_CASE6_BACK (-48.0)

//linear calibrations
// tolerance in inches
#define LINEAR_TOLERANCE (0.2)
// This is the gain for using the encoders to set the distance
//		while going straight.
#define KP_LINEAR (0.27)
// This is the gain for using the gyroscope to go straight
#define KP_ROTATION (0.017)
#define LINEAR_SETTLING_TIME (0.1)
#define LINEAR_MAX_DRIVE_SPEED (0.75)

//turning calibrations
#define ROTATIONAL_TOLERANCE (1.0)
// This is the gain for turning using the gyroscope
#define ERROR_GAIN (-0.05)
#define ROTATIONAL_SETTLING_TIME (0.1)

//encoder max drive time
#define MAX_DRIVE_TIME (3.0)

//------------------------Bob on tile calibrations END---------------------

class Robot: public frc::IterativeRobot {

public:
	Robot() :
			Adrive(DriveLeft0, DriveRight0), Drivestick(0), OperatorStick(1), DriveLeft0(
					0), DriveLeft1(1), DriveLeft2(2), DriveRight0(3), DriveRight1(
					4), DriveRight2(5), EncoderLeft(0, 1), EncoderRight(2, 3), table(
			NULL), ahrs(NULL), modeState(0), Winch0(11), Winch1(9), Shooter0(
					12), Shooter1(13), Agitator0(6), Agitator1(15), FloorIntakeRoller(
					14), KickerWheel(8), EncoderKicker(20, 21), EncoderShoot(4,
					5, true, CounterBase::k1X), ShootCommandPWM(0.75), KickerPID(
					0.003, 0.0, 0.0, &EncoderKicker, &KickerWheel), ShooterPID(
					0.0, 0.0, -0.003, 0.0, &EncoderShoot, &Shooter0) {

	}

	void RobotInit() {
		//setup smartDashboard choosers
		chooseAutonSelector.AddDefault(autonNameOFF, autonNameOFF);
		chooseAutonSelector.AddObject(autonNameRed1, autonNameRed1);
		chooseAutonSelector.AddObject(autonNameRed2, autonNameRed2);
		chooseAutonSelector.AddObject(autonNameRed3, autonNameRed3);
		chooseAutonSelector.AddObject(autonNameRed4, autonNameRed4);
		chooseAutonSelector.AddObject(autonNameBlue1, autonNameBlue1);
		chooseAutonSelector.AddObject(autonNameBlue2, autonNameBlue2);
		chooseAutonSelector.AddObject(autonNameBlue3, autonNameBlue3);
		chooseAutonSelector.AddObject(autonNameBlue4, autonNameBlue4);
		frc::SmartDashboard::PutData("Auto Modes", &chooseAutonSelector);

		chooseDriveEncoder.AddDefault(LH_Encoder, LH_Encoder);
		chooseDriveEncoder.AddObject(RH_Encoder, RH_Encoder);
		frc::SmartDashboard::PutData("Encoder", &chooseDriveEncoder);

		chooseKicker.AddDefault(chooserOpenLoop, chooserOpenLoop);
		chooseKicker.AddObject(chooserClosedLoop, chooserClosedLoop);
		frc::SmartDashboard::PutData("Kicker", &chooseKicker);

		chooseShooter.AddDefault(chooserOpenLoop, chooserOpenLoop);
		chooseShooter.AddObject(chooserClosedLoop, chooserClosedLoop);
		frc::SmartDashboard::PutData("Shooter", &chooseShooter);

		// Inialize settings from Smart Dashboard
		ShootCommandPWM = 0.9;
		ShootCommandRPM = 2600;
		ShootKP = 0.02;
		ShootKI = 0.0;
		ShootKD = 0.003;
		//ShootKF = -1.0 / 3200.0; //   1 / MAX RPM
		KickerCommandPWM = 0.5;
		KickerCommandRPM = 500;
		AgitatorCommandPWM = 0.2;
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
		SmartDashboard::PutNumber("IN: Agitator CMD (PWM)", AgitatorCommandPWM);
		SmartDashboard::PutNumber("IN: Intake CMD (PWM)", IntakeCommandPWM);
		SmartDashboard::PutNumber("IN: Auto Backup Distance (Inch)",
				autoBackupDistance);

		//turn off shifter solenoids
		driveSolenoid->Set(false);

		//disable drive watchdogs
		Adrive.SetSafetyEnabled(false);

		//changes these original negative values to positive values
		EncoderLeft.SetReverseDirection(true);
		EncoderRight.SetReverseDirection(false);
		EncoderShoot.SetReverseDirection(true);
		EncoderKicker.SetReverseDirection(true);

		//calibrations for encoders
		EncoderLeft.SetDistancePerPulse(98.0 / 3125.0 * 4.0);
		EncoderRight.SetDistancePerPulse(98.0 / 3125.0 * 4.0);
		//EncoderShoot.SetDistancePerPulse(1.0 / 3328.0 * 4.0);
		EncoderShoot.SetDistancePerPulse(1.0 / 1024.0);
		EncoderKicker.SetDistancePerPulse(1.0 / 4122.0 * 4.0);

		EncoderShoot.SetSamplesToAverage(120);

		//drive command averaging filter
		OutputX = 0, OutputY = 0;

		//variable that chooses which encoder robot is reading for autonomous mode
		useRightEncoder = true;

		// Turn off the the sensors/reset
		if (KickerClosedLoop) {
			KickerPID.Enable();
		} else {
			KickerPID.Disable();
		}

		if (ShooterClosedLoop) {
			// ShooterPID.Enable();  // enabled when trigger is depressed
		} else {
			ShooterPID.Disable();
		}

		//determines that a sensor is being read for either displacement or velocity
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
		EncoderCheckTimer.Reset();
		EncoderCheckTimer.Start();
		// Encoder based auton
		resetEncoder();
		// Turn off drive motors
		DriveLeft0.Set(0);
		DriveLeft1.Set(0);
		DriveLeft2.Set(0);
		DriveRight0.Set(0);
		DriveRight1.Set(0);
		DriveRight2.Set(0);
		//Turn off shooter motor
		Shooter0.Set(0.0);
		KickerWheel.Set(0.0);
		Agitator0.Set(0.0);
		//zeros the navX
		if (ahrs) {
			ahrs->ZeroYaw();
		}

		//forces robot into low gear
		driveSolenoid->Set(false);

		//makes sure gear doesn't eject
		GearOut->Set(false);
		//Gear Deflector down
		GearDeflector->Set(false);

		//makes sure intake is in
		FloorIntakeArm->Set(false);

	}

	void TeleopInit() {
		OutputX = 0, OutputY = 0;
		Shooter0.Set(0.0);
	}

	void RobotPeriodic() {
		//links multiple motors together
		Winch1.Set(-Winch0.Get());
		DriveLeft1.Set(DriveLeft0.Get());
		DriveLeft2.Set(DriveLeft0.Get());
		DriveRight1.Set(DriveRight0.Get());
		DriveRight2.Set(DriveRight0.Get());
		Agitator1.Set(Agitator0.Get());
		Shooter1.Set(-Shooter0.Get());

		// Encoder Selection for autotools
		encoderSelected = chooseDriveEncoder.GetSelected();
		useRightEncoder = (encoderSelected == RH_Encoder);

		// Select Auto Program
		autoSelected = chooseAutonSelector.GetSelected();

		//displays sensor and motor info to smartDashboard
		try {
			SmartDashboardUpdate();
		} catch (std::exception ex) {
			std::string err_string = "Smart Dash Errorz! :  ";
			err_string += ex.what();
			DriverStation::ReportError(err_string.c_str());
		}

	}
	void DisabledPeriodic() {
		//SmartDashboardUpdate();
	}
	void AutonomousPeriodic() {
		if (autoSelected == autonNameRed1)
			autoRedMiddleShoot();
		else if (autoSelected == autonNameRed2)
			autoForward();
		else if (autoSelected == autonNameRed3)
			autoRed3();
		else if (autoSelected == autonNameBlue1)
			autoBlueMiddleShoot();
		else if (autoSelected == autonNameBlue2)
			autoMiddleGearEject();
		else if (autoSelected == autonNameBlue3)
			autoBlue3();
		else if (autoSelected == autonNameRed4)
			autoGearLeft();
		else if (autoSelected == autonNameBlue4)
			autoGearRight();
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

		//drive controls
		double SpeedLinear = Drivestick.GetRawAxis(1) * 1; // get Yaxis value (forward)
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

		// Turn on the shooter when left hand trigger is pushed
		if (OperatorStick.GetRawAxis(2) > Deadband) {
			shooterOn = true;
			if (!operatorRightTriggerPrev and ShooterClosedLoop) {
				if (ShooterClosedLoop) {
					ShooterPID.Reset();
					ShooterPID.Enable();
				}
				operatorRightTriggerPrev = true;
			} else if (!operatorRightTriggerPrev) {
				operatorRightTriggerPrev = true;
			}

			if (ShooterClosedLoop) {
				//1.75 is a scaling factor to make the PID reach desired RPM
				ShooterPID.SetSetpoint(ShootCommandRPM / 60.0);
			} else {
				Shooter0.Set(ShootCommandPWM); // positive so they turn the correct way.
			}
			//right POV button - shooter off
		} else if (OperatorStick.GetPOV(0) == 270) {
			shooterOn = false;
			operatorRightTriggerPrev = false;
			if (ShooterClosedLoop) {
				ShooterPID.SetSetpoint(0.0);
				ShooterPID.Disable();
			} else {
				Shooter0.Set(0.0);
			}
		}

		// Turn on Kicker Wheel, conveyor, and agitators when right trigger is pressed
		if (OperatorStick.GetRawAxis(3) > Deadband) {
			Agitator0.Set(OperatorStick.GetRawAxis(3));

			if (KickerClosedLoop) {
				KickerPID.SetSetpoint(KickerCommandRPM / 60.0);
				KickerPID.Enable();
			} else {
				KickerWheel.Set(KickerCommandPWM);
			}
		} else {
			Agitator0.Set(0.0);

			if (KickerClosedLoop) {
				KickerPID.SetSetpoint(0.0);
				KickerPID.Disable();
			} else {
				KickerWheel.Set(0.0);
			}
		}

		//B Button to get and X button release the gear
		GearIn->Set(OperatorStick.GetRawButton(2));

		//prevents robot from violating frame perimeter rules
		//if X button release gear
		GearOut->Set(OperatorStick.GetRawButton(3));
		//if X button pressed, retract intake
		if (OperatorStick.GetRawButton(3)) {
			intakeDeployed = false;
			FloorIntakeArm->Set(intakeDeployed);
		}
		//else LH Bumper - Deploy Intake
		else if (OperatorStick.GetRawButton(5)) {
			intakeDeployed = true;
			FloorIntakeArm->Set(intakeDeployed);
		}

		// RH Bumper - Retract Intake
		if (OperatorStick.GetRawButton(6)) {
			intakeDeployed = false;
			FloorIntakeArm->Set(intakeDeployed);
		}

		//A Button - close shot/ gear flipper up
		GearDeflector->Set(OperatorStick.GetRawButton(1));

		//Right joystick for agitator un-jam
		if (fabs(OperatorStick.GetRawAxis(4)) > Deadband)
			Agitator0.Set(OperatorStick.GetRawAxis(4));

		//turn on winch using the left joystick
		if (OperatorStick.GetRawAxis(1) > Deadband) {
			Winch0.Set(OperatorStick.GetRawAxis(1));
		} else {
			Winch0.Set(0.0);
		}

		//POV will control floor intake stuff
		//top POV button - spin roller to intake gear
		if (OperatorStick.GetPOV(0) == 0) {
			FloorIntakeRoller.Set(0.8);
		}
		//bottom POV button - spin roller to intake balls
		else if (OperatorStick.GetPOV(0) == 180) {
			FloorIntakeRoller.Set(-0.8);
		} else {
			FloorIntakeRoller.Set(0.0);
		}

	}

// These are the state numbers for each part of autoBlue1
//		These are here so we can easily add states.
// 		State 1 is always the first one to run.
//		Always have an "end" state.

#define AB1_INIT 1
#define AB1_FWD 2
#define AB1_TIMED 3
#define AB1_WAIT 4
#define AB1_GEAR 5
#define AB1_BACK 6
#define AB1_TURN 7
#define AB1_FWD2 8
#define AB1_SHOOT 9
#define AB1_END 10

	void autoBlueMiddleShoot(void) {
		//Blue middle gear then shoot auto
		switch (modeState) {
		case AB1_INIT:
			GearDeflector->Set(true);
			FloorIntakeArm->Set(true);
			modeState = AB1_FWD;
			break;
		case AB1_FWD:
			// go forward toward airship
			if (forward(BLUE_1_CASE1_FWD)) {
				modeState = AB1_TIMED;
				AutonTimer.Reset();
			}
			break;
		case AB1_TIMED:
			//timed drive to get gear on peg
			if (timedDrive(BLUE_1_CASE2_FWD_TIME, BLUE_1_CASE2_FWD_LEFT_SPD,
			BLUE_1_CASE2_FWD_RIGHT_SPD)) {
				modeState = AB1_WAIT;
				AutonTimer.Reset();
			}
			break;
		case AB1_WAIT:
			//waits to make sure robot is in place
			if (AutonTimer.Get() > 0.5) {
				modeState = AB1_GEAR;
			}
			break;
		case AB1_GEAR:
			//deploys gear
			if (1) {
				GearOut->Set(true);
				modeState = AB1_BACK;
				resetEncoder();
				ahrs->ZeroYaw();
			}
			break;
		case AB1_BACK:
			//move backwards
			if (forward(BLUE_1_CASE5_BACK)) {
				modeState = AB1_TURN;
				ahrs->ZeroYaw();
			}
			break;
		case AB1_TURN:
			//turn to face boiler
			//not entirely sure if angle is right sign (3/25/17)
			if (autonTurn(BLUE_1_CASE6_TURN)) {
				modeState = AB1_FWD2;
				resetEncoder();
				ahrs->ZeroYaw();
			}
			break;
		case AB1_FWD2:
			//go forward 7-ish feet to get itno position to shoot
			if (forward(BLUE_1_CASE7_FWD)) {
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
			if (timedDrive(0.5, -0.2, -0.2)) {

				//if (forward7ft(-0.4, 7 * 12.0)) {
				AutonTimer.Reset();
				modeState = AB2_GEAR_WAIT;
			}
			break;
		case AB2_GEAR_WAIT:
			if (AutonTimer.Get() > 0.5) {
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
#define AB3_BACK 7
#define AB3_TURN2 8
#define AB3_SHOOT 9
#define AB3_END 10
	void autoBlue3(void) {
		//blue side code
		//puts gear on pin on side of airship
		switch (modeState) {
		case AB3_INIT:
			GearDeflector->Set(false);
			FloorIntakeArm->Set(false);
			modeState = AB3_FWD;
			break;
		case AB3_FWD:
			// go forward 7 ft
			if (forward(100.0)) {
				//modeState = AB3_TURN;
				modeState = AB3_TURN;
				ahrs->ZeroYaw();
			}
			break;
		case AB3_TURN:
			// turn 90 degrees counterclockwise
			if (autonTurn(BLUE_3_CASE2_TURN)) {
				modeState = AB3_STR8;
				resetEncoder();
				ahrs->ZeroYaw();
			}
			break;
		case AB3_STR8:
			// go forward
			//timed drive
			if (forward(18.0)) {
				modeState = AB3_GEAR_WAIT;
				resetEncoder();
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
				modeState = AB3_BACK;
				resetEncoder();
				GearOut->Set(true);
			}
			break;
		case AB3_BACK:
			//back up to the shooting radius
			Shooter0.Set(0.95);
			if (forward(-65.0)) {
				GearOut->Set(false);
				ahrs->ZeroYaw();
				modeState = AB3_TURN2;
			}
			break;
		case AB3_TURN2:
			//turn to face boiler
			if (autonTurn(-15.0)) {
				modeState = AB3_SHOOT;
			}
			break;
		case AB3_SHOOT:
			//turn to face boiler
			if (shoot()) {
				modeState = AB2_END;
			}
			break;
		default:
			stopMotors();
		}
		return;
	}

#define AR1_INIT 1
#define AR1_FWD 2
#define AR1_TIMED 3
#define AR1_WAIT 4
#define AR1_GEAR 5
#define AR1_BACK 6
#define AR1_TURN 7
#define AR1_FWD2 8
#define AR1_SHOOT 9
#define AR1_END 10
	void autoRedMiddleShoot(void) {
		switch (modeState) {
		case AR1_INIT:
			GearDeflector->Set(true);
			FloorIntakeArm->Set(true);
			modeState = AR1_FWD;
			break;
		case AR1_FWD:
			// go forward 7 ft
			if (forward(RED_1_CASE1_FWD)) {
				modeState = AR1_TIMED;
				AutonTimer.Reset();
			}
			break;
		case AR1_TIMED:
			// turn 90 degrees counterclockwise
			if (timedDrive(RED_1_CASE2_FWD_TIME, RED_1_CASE2_FWD_LEFT_SPD,
			RED_1_CASE2_FWD_RIGHT_SPD)) {
				modeState = AR1_WAIT;
				AutonTimer.Reset();
			}
			break;
		case AR1_WAIT:
			//waits in front of hopper a couple of seconds for balls
			if (AutonTimer.Get() > 0.5) {
				modeState = AR1_GEAR;
			}
			break;
		case AR1_GEAR:
			//go backward 3-ish feet
			if (1) {
				GearOut->Set(true);
				resetEncoder();
				ahrs->ZeroYaw();
				modeState = AR1_BACK;
			}
			break;
		case AR1_BACK:
			// turns counterclockwise enough degrees to face boiler
			if (forward(RED_1_CASE5_BACK)) {
				modeState = AR1_TURN;
				ahrs->ZeroYaw();
			}
			break;
		case AR1_TURN:
			//change to timed drive
			//go forward 7-ish feet to run into boiler
			if (autonTurn(RED_1_CASE6_TURN)) {
				modeState = AR1_FWD2;
				resetEncoder();
				ahrs->ZeroYaw();
			}
			break;
		case AR1_FWD2:
			//change to timed drive
			//go forward 7-ish feet to run into boiler
			if (forward(RED_1_CASE7_FWD)) {
				modeState = AR1_SHOOT;
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
			resetEncoder();
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
#define AR3_GEAR_WAIT 5
#define AR3_GEAR 6
#define AR3_BACK 7
#define AR3_TURN2 8
#define AR3_SHOOT 9
#define AR3_END 10
	void autoRed3(void) {
		//red three
		//puts gear onto side of airship and shoots
		switch (modeState) {
		case AR3_INIT:
			GearDeflector->Set(true);
			FloorIntakeArm->Set(true);
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
				resetEncoder();
				ahrs->ZeroYaw();
			}
			break;
		case AR3_STR8:
			// go forward to put gear on peg
			if (forward(RED_3_CASE3_STR8)) {
				modeState = AR3_GEAR_WAIT;
				AutonTimer.Reset();
			}
			break;
		case AR3_GEAR_WAIT:
			// wait for gear to be lined up
			if (AutonTimer.Get() > 0.5) {
				modeState = AR3_GEAR;
			}
			break;
		case AR3_GEAR:
			//deploy gear
			if (1) {
				modeState = AR3_BACK;
				ahrs->ZeroYaw();
				resetEncoder();
			}
			break;
		case AR3_BACK:
			//back up to radius
			if (forward(RED_3_CASE6_BACK)) {
				modeState = AR3_TURN2;
				ahrs->ZeroYaw();
			}
			break;
		case AR3_TURN2:
			//turn to face boiler
			if (autonTurn(RED_3_CASE7_TURN)) {
				modeState = AR3_SHOOT;
			}
			break;
		case AR3_SHOOT:
			//shoot
			if (shoot()) {
				modeState = AR3_END;
			}
			break;
		default:
			stopMotors();
		}
		return;
	}

#define AB4_INIT 1
#define AB4_FWD 2
#define AB4_TURN 3
#define AB4_STR8 4
#define AB4_GEAR_WAIT 5
#define AB4_GEAR 6
#define AB4_BACK 7
#define AB4_END 8
	void autoGearLeft(void) {
		//puts gear on pin on right side of airship
		switch (modeState) {
		case AB4_INIT:
			modeState = AB3_FWD;
			break;
		case AB4_FWD:
			// go forward 7 ft
			if (forward(BLUE_4_CASE1_FWD)) {
				modeState = AB4_TURN;
				ahrs->ZeroYaw();
			}
			break;
		case AB4_TURN:
			// turn 90 degrees counterclockwise
			if (autonTurn(BLUE_4_CASE2_TURN)) {
				modeState = AB4_STR8;
				resetEncoder();
				ahrs->ZeroYaw();
			}
			break;
		case AB4_STR8:
			// go forward
			//timed drive
			if (forward(BLUE_4_CASE3_STR8)) {
				modeState = AB4_GEAR_WAIT;
				resetEncoder();
			}
			break;
		case AB4_GEAR_WAIT:
			// go forward
			//timed drive
			//AutonTimer.Get() > 1.5
			if (timedDrive(1.0, -0.2, -0.2)) {
				modeState = AB4_GEAR;
			}
			break;
		case AB4_GEAR:
			// deploy gear
			if (1) {
				modeState = AB4_BACK;
				resetEncoder();
				GearOut->Set(true);
			}
			break;
		case AB4_BACK:
			//back up to the shooting radius
			//forward(BLUE_4_CASE6_BACK)
			if (timedDrive(4.0, 0.3, 0.3)) {
				GearOut->Set(false);
				ahrs->ZeroYaw();
				modeState = AB4_END;
			}
			break;
		default:
			stopMotors();
		}
	}

#define AR4_INIT 1
#define AR4_FWD 2
#define AR4_TURN 3
#define AR4_STR8 4
#define AR4_GEAR_WAIT 5
#define AR4_GEAR 6
#define AR4_BACK 7
#define AR4_END 8
	void autoGearRight(void) {
		//puts gear onto left side of airship
		switch (modeState) {
		case AR4_INIT:
			modeState = AR4_FWD;
			break;
		case AR4_FWD:
			// go forward 7 ft
			if (forward(RED_4_CASE1_FWD)) {
				modeState = AR4_TURN;
				ahrs->ZeroYaw();
			}
			break;
		case AR4_TURN:
			// turn 60 degrees clockwise
			if (autonTurn(RED_4_CASE2_TURN)) {
				modeState = AR4_STR8;
				resetEncoder();
				ahrs->ZeroYaw();
			}
			break;
		case AR4_STR8:
			// go forward to put gear on peg
			if (forward(RED_4_CASE3_STR8)) {
				modeState = AR4_GEAR_WAIT;
				AutonTimer.Reset();
			}
			break;
		case AR4_GEAR_WAIT:
			// wait for gear to be lined up
			if (timedDrive(1.0, -0.2, -0.2)) {
				modeState = AR4_GEAR;
			}
			break;
		case AR4_GEAR:
			//deploy gear
			if (1) {
				modeState = AR4_BACK;
				ahrs->ZeroYaw();
				resetEncoder();
			}
			break;
		case AR4_BACK:
			//back up to radius
			//forward(RED_4_CASE6_BACK)
			if (timedDrive(4.0, 0.3, 0.3)) {
				modeState = AR4_END;
				ahrs->ZeroYaw();
			}
			break;
		default:
			stopMotors();
		}
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
		SmartDashboard::PutBoolean("Shooter Running", shooterOn);
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
		SmartDashboard::PutNumber("Shooter Motor1  Output", Shooter1.Get());
		SmartDashboard::PutNumber("Winch Motor0 Output", Winch0.Get());
		SmartDashboard::PutNumber("Winch Motor1 Output", Winch1.Get());
		SmartDashboard::PutNumber("Kicker Motor Output", KickerWheel.Get());
		SmartDashboard::PutNumber("Agitator Motor0 Output", Agitator0.Get());
		SmartDashboard::PutNumber("Agitator Motor1 Output", Agitator1.Get());
		SmartDashboard::PutNumber("Floor Intake Motor Output",
				FloorIntakeRoller.Get());

//		//chooser code for manip in open/closed loop
		ShooterClosedLoop = (chooseShooter.GetSelected() == chooserClosedLoop);
		KickerClosedLoop = (chooseKicker.GetSelected() == chooserClosedLoop);

	}

	void motorSpeed(double leftMotor, double rightMotor) {
		DriveLeft0.Set(leftMotor * -1);
		DriveLeft1.Set(leftMotor * -1);
		DriveLeft2.Set(leftMotor * -1);
		DriveRight0.Set(rightMotor);
		DriveRight1.Set(rightMotor);
		DriveRight2.Set(rightMotor);
	}

	int forward(double targetDistance) {
		double encoderDistance = readEncoder();
		double encoderError = encoderDistance - targetDistance;
		double driveCommandLinear = encoderError * KP_LINEAR;
		//limits max drive speed
		if (driveCommandLinear > LINEAR_MAX_DRIVE_SPEED) {
			driveCommandLinear = LINEAR_MAX_DRIVE_SPEED;
		} else if (driveCommandLinear < -1 * LINEAR_MAX_DRIVE_SPEED) { 								/////***** "-1" is a "magic number." At least put a clear comment in here.
			driveCommandLinear = -1 * LINEAR_MAX_DRIVE_SPEED;
		}
		//gyro values that make the robot drive straight
		double gyroAngle = ahrs->GetAngle();
		double driveCommandRotation = gyroAngle * KP_ROTATION;
		//encdoer check
		if (EncoderCheckTimer.Get() > MAX_DRIVE_TIME) {
			motorSpeed(0.0, 0.0);
			DriverStation::ReportError("(forward) Encoder Max Drive Time Exceeded");
		} else {
			//calculates and sets motor speeds
			motorSpeed(driveCommandLinear + driveCommandRotation,
					driveCommandLinear - driveCommandRotation);
		}
		//routine helps prevent the robot from overshooting the distance
		if (isWaiting == 0) {
			if (abs(encoderError) < LINEAR_TOLERANCE) {
				isWaiting = 1;
				AutonTimer.Reset();
			}
		}
		//timed wait
		else {
			float currentTime = AutonTimer.Get();
			if (abs(encoderError) > LINEAR_TOLERANCE) {
				isWaiting = 0;
			} else if (currentTime > LINEAR_SETTLING_TIME) {
				isWaiting = 0;
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

	//------------- Start Code for Running Encoders --------------
	double readEncoder() {
		double usableEncoderData;
		double r = EncoderRight.GetDistance();
		double l = EncoderLeft.GetDistance();
		//If a encoder is disabled switch l or r to each other.
		if (l > 0) {
			usableEncoderData = fmax(r, l);
		} else if (l == 0) {
			usableEncoderData = r;
		} else {
			usableEncoderData = fmin(r, l);
		}
		return usableEncoderData;
	}

	void resetEncoder() {
		EncoderLeft.Reset();
		EncoderRight.Reset();
		EncoderCheckTimer.Reset();
	}
	//------------- End Code for Running Encoders --------------------

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
			Shooter0.Set(ShootCommandPWM); // positive so they turn the correct way.
		}

		return 1;
	}

	void TestPeriodic() {
		lw->Run();
	}

private:
	RobotDrive Adrive;
	frc::LiveWindow* lw = LiveWindow::GetInstance();
	frc::SendableChooser<std::string> chooseAutonSelector, chooseDriveEncoder,
			chooseKicker, chooseShooter;
	const std::string AutonNameSwitch = "Use Switch";
	const std::string autonNameOFF = "0 OFF";
	const std::string autonNameBlue1 = "Blue Middle Gear Shoot";
	const std::string autonNameBlue2 = "Middle Gear Eject";
	const std::string autonNameBlue3 = "Blue Left Side Gear Shoot";
	const std::string autonNameBlue4 = "Right Side Gear";
	const std::string autonNameRed1 = "Red Middle Gear Shoot";
	const std::string autonNameRed2 = "Go Forward Only";
	const std::string autonNameRed3 = "Red Right Side Gear Shoot";
	const std::string autonNameRed4 = "Left Side Gear";
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
	Timer EncoderCheckTimer;
	Encoder EncoderLeft;
	Encoder EncoderRight;
	std::shared_ptr<NetworkTable> table;
	AHRS *ahrs;
//tells us what state we are in in each auto mode
	int modeState;bool AutonOverride;
	int AutoVal;
	float OutputX, OutputY;
	int isWaiting = 0;			/////***** Divide this into 2 variables.

	Solenoid *driveSolenoid = new Solenoid(0);
	// create pdp variable
	PowerDistributionPanel *pdp = new PowerDistributionPanel();

//manipulator
	VictorSP Winch0, Winch1;
	VictorSP Shooter0, Shooter1;
	VictorSP Agitator0, Agitator1;
	VictorSP FloorIntakeRoller;
	VictorSP KickerWheel;
	Solenoid *GearOut = new Solenoid(1);
	Solenoid *GearIn = new Solenoid(3);
	Solenoid *GearDeflector = new Solenoid(5);
	Solenoid *FloorIntakeArm = new Solenoid(2);
	Encoder EncoderKicker;
	Encoder EncoderShoot;
	double IntakeCommandPWM, AgitatorCommandPWM, ConvCommandPWM,
			ShootCommandRPM, ShootCommandPWM, KickerCommandRPM,
			KickerCommandPWM, ShootKP, ShootKI, ShootKD, ShootKF;

	bool useRightEncoder;bool KickerClosedLoop;bool ShooterClosedLoop;

	PIDController KickerPID, ShooterPID;

	bool driveRightTriggerPrev = false;bool driveButtonYPrev = false;bool operatorRightTriggerPrev =
	false;bool intakeDeployed = false;bool shooterOn = false;

	double autoBackupDistance;

}
;

START_ROBOT_CLASS(Robot)
