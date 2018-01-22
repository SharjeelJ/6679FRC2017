package org.usfirst.frc.team6679.robot;

import com.ctre.CANTalon;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.VictorSP;

public class ElectronicsInitialization
{
	// Initialize a global integer value to specify which robot the components are being initialized for (1 = competition, 2 = practice)
	private int robotNumber = 0;

	// Initialize global variables for the competition robot
	// PWM
	private VictorSP r1LeftDriveMotor1;
	private VictorSP r1LeftDriveMotor2;
	private VictorSP r1RightDriveMotor1;
	private VictorSP r1RightDriveMotor2;
	// RoboRio configuration page
	private CANTalon r1FlywheelMotor;
	private CANTalon r1IndexerMotor;
	private CANTalon r1ClimberMotor;
	private CANTalon r1GearPickupMotor;
	// PCM
	private Solenoid r1GearIntakeSolenoid;
	private Solenoid r1GearOuttakeSolenoid;
	private Solenoid r1GearPickupSolenoid1;
	private Solenoid r1GearPickupSolenoid2;
	// DIO
	private Ultrasonic r1UltrasonicSensor;
	private Encoder r1DrivetrainEncoderPrimary;
	private Encoder r1DrivetrainEncoderBackup;
	private DigitalInput r1GearPickupButton1;
	private DigitalInput r1GearPickupButton2;
	private DigitalInput r1GearPickupButton3;
	private DigitalInput r1GearPickupButton4;

	// Initialize global variables for the practice robot
	// PWM
	private Talon r2LeftDriveMotor1;
	private Talon r2LeftDriveMotor2;
	private Talon r2RightDriveMotor1;
	private Talon r2RightDriveMotor2;
	// RoboRio configuration page
	private CANTalon r2FlywheelMotor;
	private CANTalon r2IndexerMotor;
	private CANTalon r2ClimberMotor;
	private CANTalon r2GearPickupMotor;
	// PCM
	private Solenoid r2GearIntakeSolenoid;
	private Solenoid r2GearOuttakeSolenoid;
	private Solenoid r2GearPickupSolenoid1;
	private Solenoid r2GearPickupSolenoid2;
	// DIO
	private Ultrasonic r2UltrasonicSensor;
	private Encoder r2DrivetrainEncoderPrimary;
	private Encoder r2DrivetrainEncoderBackup;
	private DigitalInput r2GearPickupButton1;
	private DigitalInput r2GearPickupButton2;
	private DigitalInput r2GearPickupButton3;
	private DigitalInput r2GearPickupButton4;

	// Initialize global variables for the shared components
	// Driverstation USB
	private Joystick primaryXboxController = new Joystick(0);
	private Joystick secondaryXboxController = new Joystick(1);
	// RoboRio SPI
	private AHRS navX = new AHRS(SPI.Port.kMXP);
	// RoboRio configuration page
	private Compressor pneumaticCompressor = new Compressor(0);

	// Function to initialize all the components depending on which robot the code is running on
	public ElectronicsInitialization(int inputRobotNumber) 
	{
		// Stores the identifier for which robot the code is being run for
		robotNumber = inputRobotNumber;

		// Checks to see if the components being initialized are for the competition robot or practice robot and calls the appropriate code
		if (robotNumber == 1)
		{
			// Initializes the components for the competition robot
			r1RightDriveMotor1 = new VictorSP(1);
			r1RightDriveMotor2 = new VictorSP(2);
			r1LeftDriveMotor1 = new VictorSP(3);
			r1LeftDriveMotor2 = new VictorSP(4);
			r1FlywheelMotor = new CANTalon(0);
			r1IndexerMotor = new CANTalon(1);
			r1ClimberMotor = new CANTalon(2);
			r1GearPickupMotor = new CANTalon(3);
			r1GearIntakeSolenoid = new Solenoid(0);
			r1GearOuttakeSolenoid = new Solenoid(1);
			r1GearPickupSolenoid1 = new Solenoid(2);
			r1GearPickupSolenoid2 = new Solenoid(3);
			r1UltrasonicSensor = new Ultrasonic(0, 1);
			r1DrivetrainEncoderPrimary = new Encoder(2, 3);
			r1DrivetrainEncoderBackup = new Encoder(4, 5);
			r1GearPickupButton1 = new DigitalInput(6);
			r1GearPickupButton2 = new DigitalInput(7);
			r1GearPickupButton3 = new DigitalInput(8);
			r1GearPickupButton4 = new DigitalInput(9);			
		}
		else if (robotNumber == 2)
		{
			// initializes the components for the practice robot
			r2RightDriveMotor1 = new Talon(1);
			r2RightDriveMotor2 = new Talon(2);
			r2LeftDriveMotor1 = new Talon(3);
			r2LeftDriveMotor2 = new Talon(4);
			r2FlywheelMotor = new CANTalon(0);
			r2IndexerMotor = new CANTalon(1);
			r2ClimberMotor = new CANTalon(2);
			r2GearPickupMotor = new CANTalon(3);
			r2GearIntakeSolenoid = new Solenoid(0);
			r2GearOuttakeSolenoid = new Solenoid(1);
			r2GearPickupSolenoid1 = new Solenoid(2);
			r2GearPickupSolenoid2 = new Solenoid(3);
			r2UltrasonicSensor = new Ultrasonic(0, 1);
			r2DrivetrainEncoderPrimary = new Encoder(2, 3);
			r2DrivetrainEncoderBackup = new Encoder(4, 5);
			r2GearPickupButton1 = new DigitalInput(6);
			r2GearPickupButton2 = new DigitalInput(7);
			r2GearPickupButton3 = new DigitalInput(8);
			r2GearPickupButton4 = new DigitalInput(9);			

		}
	}

	// Function to return the Primary Xbox controller
	public Joystick getPrimaryXboxController()
	{
		// Returns the controller
		return primaryXboxController;
	}

	// Function to return the Secondary Xbox controller
	public Joystick getSecondaryXboxController()
	{
		// Returns the controller
		return secondaryXboxController;
	}

	// Function to return the navX
	public AHRS getNavX()
	{
		// Returns the navX
		return navX;
	}

	// Function to return the Pneumatic compressor
	public Compressor getPneumaticCompressor()
	{
		// Returns the compressor
		return pneumaticCompressor;
	}

	// Function to return the Left Drive motor 1
	public Object getLeftDriveMotor1()
	{
		// Returns the appropriate motor based on which robot the code is being initialized for
		if (robotNumber == 1)
			return r1LeftDriveMotor1;
		else if (robotNumber == 2)
			return r2LeftDriveMotor1;
		else
			return null;
	}

	// Function to return the Left Drive motor 2
	public Object getLeftDriveMotor2()
	{
		// Returns the appropriate motor based on which robot the code is being initialized for
		if (robotNumber == 1)
			return r1LeftDriveMotor2;
		else if (robotNumber == 2)
			return r2LeftDriveMotor2;
		else
			return null;
	}

	// Function to return the Right Drive motor 1
	public Object getRightDriveMotor1()
	{
		// Returns the appropriate motor based on which robot the code is being initialized for
		if (robotNumber == 1)
			return r1RightDriveMotor1;
		else if (robotNumber == 2)
			return r2RightDriveMotor1;
		else
			return null;
	}

	// Function to return the Right Drive motor 2
	public Object getRightDriveMotor2()
	{
		// Returns the appropriate motor based on which robot the code is being initialized for
		if (robotNumber == 1)
			return r1RightDriveMotor2;
		else if (robotNumber == 2)
			return r2RightDriveMotor2;
		else
			return null;
	}

	// Function to return the Flywheel motor
	public CANTalon getFlywheelMotor()
	{
		// Returns the appropriate motor based on which robot the code is being initialized for
		if (robotNumber == 1)
			return r1FlywheelMotor;
		else if (robotNumber == 2)
			return r2FlywheelMotor;
		else
			return null;
	}

	// Function to return the Indexer motor
	public CANTalon getIndexerMotor()
	{
		// Returns the appropriate motor based on which robot the code is being initialized for
		if (robotNumber == 1)
			return r1IndexerMotor;
		else if (robotNumber == 2)
			return r2IndexerMotor;
		else
			return null;
	}

	// Function to return the Climber motor
	public CANTalon getClimberMotor()
	{
		// Returns the appropriate motor based on which robot the code is being initialized for
		if (robotNumber == 1)
			return r1ClimberMotor;
		else if (robotNumber == 2)
			return r2ClimberMotor;
		else
			return null;
	}

	// Function to return the Gear Pickup motor
	public CANTalon getGearPickupMotor()
	{
		// Returns the appropriate motor based on which robot the code is being initialized for
		if (robotNumber == 1)
			return r1GearPickupMotor;
		else if (robotNumber == 2)
			return r2GearPickupMotor;
		else
			return null;
	}

	// Function to return the Gear Intake solenoid
	public Solenoid getGearIntakeSolenoid()
	{
		// Returns the appropriate solenoid based on which robot the code is being initialized for
		if (robotNumber == 1)
			return r1GearIntakeSolenoid;
		else if (robotNumber == 2)
			return r2GearIntakeSolenoid;
		else
			return null;
	}

	// Function to return the Gear Outtake solenoid
	public Solenoid getGearOuttakeSolenoid()
	{
		// Returns the appropriate solenoid based on which robot the code is being initialized for
		if (robotNumber == 1)
			return r1GearOuttakeSolenoid;
		else if (robotNumber == 2)
			return r2GearOuttakeSolenoid;
		else
			return null;
	}

	// Function to return the Gear Pickup solenoid 1
	public Solenoid getGearPickupSolenoid1()
	{
		// Returns the appropriate solenoid based on which robot the code is being initialized for
		if (robotNumber == 1)
			return r1GearPickupSolenoid1;
		else if (robotNumber == 2)
			return r2GearPickupSolenoid1;
		else
			return null;
	}

	// Function to return the Gear Pickup solenoid 2
	public Solenoid getGearPickupSolenoid2()
	{
		// Returns the appropriate solenoid based on which robot the code is being initialized for
		if (robotNumber == 1)
			return r1GearPickupSolenoid2;
		else if (robotNumber == 2)
			return r2GearPickupSolenoid2;
		else
			return null;
	}

	// Function to return the Ultrasonic sensor
	public Ultrasonic getUltrasonicSensor()
	{
		// Returns the appropriate sensor based on which robot the code is being initialized for
		if (robotNumber == 1)
			return r1UltrasonicSensor;
		else if (robotNumber == 2)
			return r2UltrasonicSensor;
		else
			return null;
	}

	// Function to return the Drivetrain Primary encoder
	public Encoder getDrivetrainEncoder()
	{
		// Returns the appropriate encoder based on which robot the code is being initialized for
		if (robotNumber == 1)
			return r1DrivetrainEncoderPrimary;
		else if (robotNumber == 2)
			return r2DrivetrainEncoderPrimary;
		else
			return null;
	}

	// Function to return the Drivetrain Backup encoder
	public Encoder getDrivetrainBackupEncoder()
	{
		// Returns the appropriate encoder based on which robot the code is being initialized for
		if (robotNumber == 1)
			return r1DrivetrainEncoderBackup;
		else if (robotNumber == 2)
			return r2DrivetrainEncoderBackup;
		else
			return null;
	}

	// Function to return the Gear Pickup button 1
	public DigitalInput getGearPickupButton1()
	{
		// Returns the appropriate sensor based on which robot the code is being initialized for
		if (robotNumber == 1)
			return r1GearPickupButton1;
		else if (robotNumber == 2)
			return r2GearPickupButton1;
		else
			return null;
	}

	// Function to return the Gear Pickup button 2
	public DigitalInput getGearPickupButton2()
	{
		// Returns the appropriate sensor based on which robot the code is being initialized for
		if (robotNumber == 1)
			return r1GearPickupButton2;
		else if (robotNumber == 2)
			return r2GearPickupButton2;
		else
			return null;
	}

	// Function to return the Gear Pickup button 3
	public DigitalInput getGearPickupButton3()
	{
		// Returns the appropriate sensor based on which robot the code is being initialized for
		if (robotNumber == 1)
			return r1GearPickupButton3;
		else if (robotNumber == 2)
			return r2GearPickupButton3;
		else
			return null;
	}

	// Function to return the Gear Pickup button 4
	public DigitalInput getGearPickupButton4()
	{
		// Returns the appropriate sensor based on which robot the code is being initialized for
		if (robotNumber == 1)
			return r1GearPickupButton4;
		else if (robotNumber == 2)
			return r2GearPickupButton4;
		else
			return null;
	}
}
