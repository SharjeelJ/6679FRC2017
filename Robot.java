package org.usfirst.frc.team6679.robot;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PWMSpeedController;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot
{
	// Initialize a global static integer to define which robot the code is being run on (1 = competition, 2 = practice)
	private static final int robotNumber = 1;

	// Initialize static constants that will be used to configure the camera(s)
	// Width x Height (compatible with vision analysis) = 640x480, 320x240
	// Brightness = 0, Exposure = 0, FPS = 30
	private static final int CAMERA_WIDTH = 320;
	private static final int CAMERA_HEIGHT = 240;
	private static final int CAMERA_BRIGHTNESS = 0;
	private static final int CAMERA_EXPOSURE = 0;
	private static final int CAMERA_FPS = 10;
	private static final int CAMERA2_FPS = 30;

	// Initialize global values that will be displayed to and can be adjusted from the SmartDashboard
	private String targetOutlineSize = "";
	private double intakeClimberMotorPower = 40;
	private double indexerMotorPower = 20;
	private double flywheelMotorPower = 28;
	private double flywheelP = 0.2;
	private double flywheelI = 0.0001;
	private double flywheelD = 0;
	private double flywheelF = 0.0276;
	private double flywheelRampRate = 1;
	private double flywheelMotorMaxSpeed = 40000;
	private int flywheelAcceptableError = 0;
	private double drivingKp = 0.015;

	// Autonomous encoder values
	private double auto1Value1 = 59;
	private double auto2Value1 = 59;
	private double auto3Value1 = 57;
	private double auto3Value2 = 76;
	private double auto4Value1 = 55;
	private double auto4Value2 = 78;
	private double auto5Value1 = 55;
	private double auto5Value2 = 83;
	private double auto6Value1 = 60;
	private double auto6Value2 = 40;	
	private double auto7Value1 = 40;
	private double auto8Value1 = 60;
	private double auto8Value2 = 50;

	// Initialize global integers to keep track of the autonomous routine requested and the stage of the routine the robot is currently on
	private int autonomousRoutine = 0;
	private int autonomousRoutineStage = 0;

	// Initialize a global boolean to keep track of whether or not the rumble motors on the primary joystick have been triggered
	private boolean hasTriggeredRumble = false;

	// Initialize a global boolean array for the xbox controller's buttons
	private boolean[] stick1BtnPressed = new boolean[10];

	// Initialize the electronics for the robot (1 = competition robot, 2 = practice robot)
	private ElectronicsInitialization robotElectronics = new ElectronicsInitialization(robotNumber);

	// Initialize a global variable for the encoder ticks to distance scale (inches) and a RobotDrive variable to control the robot's drivetrain
	private double encoderDistanceScale = 0.005813953488372; // 127 ticks = 1 inch
	private RobotDrive robotDrive = new RobotDrive((PWMSpeedController) robotElectronics.getLeftDriveMotor2(), (PWMSpeedController) robotElectronics.getLeftDriveMotor1(), (PWMSpeedController) robotElectronics.getRightDriveMotor2(), (PWMSpeedController) robotElectronics.getRightDriveMotor1());

	// Function that is run once when the robot is first powered on
	public void robotInit() 
	{
		// Turns off motor safety and sets the sensitivity to 100% and sets the encoder distance scale
		robotDrive.setSafetyEnabled(false);
		robotDrive.setSensitivity(1);
		robotElectronics.getDrivetrainEncoder().setDistancePerPulse(encoderDistanceScale);

		// Inverts the directions of all the drivetrain motors
		robotDrive.setInvertedMotor(MotorType.kFrontLeft, true);
		robotDrive.setInvertedMotor(MotorType.kRearLeft, true);
		robotDrive.setInvertedMotor(MotorType.kFrontRight, true);
		robotDrive.setInvertedMotor(MotorType.kRearRight, true);

		// Enables the ultrasonic sensor to calculate distances
		robotElectronics.getUltrasonicSensor().setEnabled(true);
		robotElectronics.getUltrasonicSensor().setAutomaticMode(true);

		// Sets the flywheel motor's minimum and maximum voltages for going forwards and backwards
		robotElectronics.getFlywheelMotor().configNominalOutputVoltage(+0.0f, -0.0f);
		robotElectronics.getFlywheelMotor().configPeakOutputVoltage(+12.0f, -0.0f);

		// Sets the flywheel motor's PID profile and changes the PID type to be a speed based PID
		robotElectronics.getFlywheelMotor().setProfile(0);
		robotElectronics.getFlywheelMotor().changeControlMode(TalonControlMode.Speed);

		// Sets the default positions for the solenoids
		robotElectronics.getGearIntakeSolenoid().set(false);
		robotElectronics.getGearOuttakeSolenoid().set(false);
		robotElectronics.getGearPickupSolenoid1().set(false);
		robotElectronics.getGearPickupSolenoid2().set(true);

		// Initializes and starts up a new thread for the image analysis camera
		new Thread(() -> {
			// Instantiates the vision analysis class as an object
			VisionAnalysis visionAnalysis = new VisionAnalysis();

			// Instantiates a UsbCamera object from the CameraServer for the first camera for image analysis (starts the SmartDashboard's camera stream)
			UsbCamera camera1 = CameraServer.getInstance().startAutomaticCapture("Microsoft LifeCam HD-3000 (Image Analysis)", 0);

			// Instantiates a UsbCamera object from the CameraServer for the second camera for POV driving (starts the SmartDashboard's camera stream)
			UsbCamera camera2 = CameraServer.getInstance().startAutomaticCapture("Microsoft LifeCam NX-6000 (POV Driving)", 1);

			// Sets the properties for the first camera object
			camera1.setResolution(CAMERA_WIDTH, CAMERA_HEIGHT);
			camera1.setBrightness(CAMERA_BRIGHTNESS);
			camera1.setExposureManual(CAMERA_EXPOSURE);
			camera1.setFPS(CAMERA_FPS);

			// Sets the properties for the second camera object
			camera2.setResolution(CAMERA_WIDTH, CAMERA_HEIGHT);
			camera2.setFPS(CAMERA2_FPS);

			// Gets a CvSink that will capture Mats from the camera
			CvSink cvSink = CameraServer.getInstance().getVideo(camera1);

			// Sets up a CvSource that will send the processed input back to the SmartDashboard
			CvSource outputStream = CameraServer.getInstance().putVideo("Image Analysis", CAMERA_WIDTH, CAMERA_HEIGHT);

			// Initialize a Mat to capture a frame from the camera for processing
			Mat sourceFrame = new Mat();

			// Runs the thread as long as the robot code is running
			while(!Thread.interrupted())
			{
				// Captures a frame from the camera input
				cvSink.grabFrame(sourceFrame);

				// Performs the vision analysis code on the captured frame
				visionAnalysis.process(sourceFrame);

				// Checks to see if any contours were even found otherwise skips over the vision code
				if (visionAnalysis.convexHullsOutput().size() > 0)
				{
					// Initializes a Rectangle object that will store the largest contour's outline to be added to the processed image to identify the target
					Rect targetOutline = Imgproc.boundingRect(visionAnalysis.convexHullsOutput().get(0));

					// Loops through all the possible bounding contours and filters out the largest one
					for (int counter = 1; counter < visionAnalysis.convexHullsOutput().size(); counter++)
					{
						// Checks to see if the current contour is larger than the previous one and stores it
						if (targetOutline.size().area() < Imgproc.boundingRect(visionAnalysis.convexHullsOutput().get(counter)).area())
						{
							targetOutline = Imgproc.boundingRect(visionAnalysis.convexHullsOutput().get(counter));
						}
					}

					// Draws the target's outline onto the source
					Imgproc.rectangle(sourceFrame, new Point(targetOutline.x, targetOutline.y), new Point(targetOutline.x + targetOutline.width, targetOutline.y + targetOutline.height), new Scalar(0, 0, 255), 2);

					// Draws the hard coded ideal target location onto the source
					Imgproc.rectangle(sourceFrame, new Point(130, 50), new Point(180, 60), new Scalar(255, 255, 255), 2);

					// Stores the dimensions of the target's outline
					targetOutlineSize = targetOutline.size().toString();

					// Puts the processed frame on the driverstation as a labeled stream
					outputStream.putFrame(sourceFrame);
				}
				else
				{
					// Puts the unprocessed source frame on the driverstation as a labeled stream
					outputStream.putFrame(sourceFrame);
				}
			}
		}).start();

		// Calls the function to update the SmartDashboard window's values
		updateSmartDashboard();
	}

	// Function that is called periodically during test mode
	public void testPeriodic() 
	{
		LiveWindow.run();
	}

	// Function that is called periodically during disabled mode
	public void disabledPeriodic()
	{
		// Grabs the input values from the driverstation SmartDashboard window
		getSmartDashboardValues();

		// Calls the function to update the SmartDashboard window's values
		updateSmartDashboard();
	}

	// Function that is run once each time the robot enters autonomous mode
	public void autonomousInit()
	{
		// Resets the autonomous stage counter
		autonomousRoutineStage = 0;

		// Turns off the pneumatic compressor
		robotElectronics.getPneumaticCompressor().setClosedLoopControl(false);	

		// Resets the navX
		robotElectronics.getNavX().reset();

		// Resets the drivetrain encoder
		robotElectronics.getDrivetrainEncoder().reset();

		// Resets the flywheel encoder's accumulator value
		robotElectronics.getFlywheelMotor().clearIAccum();
	}

	// Function that is run periodically during autonomous mode
	public void autonomousPeriodic() 
	{
		// Checks to see which autonomous routine has been requested for and calls it
		switch (autonomousRoutine)
		{
		case 1: // Moves forward to the peg station and releases the gear (middle blue station)
			if (robotElectronics.getDrivetrainEncoder().getDistance() <= auto1Value1 && autonomousRoutineStage == 0)
			{
				// Moves forward in a straight line
				robotDrive.drive(-0.5, -robotElectronics.getNavX().getAngle() * drivingKp);
			}
			else if (autonomousRoutineStage == 0)
			{
				// Stops the robot
				autonomousRoutineStage = 1;
				robotDrive.stopMotor();
				Timer.delay(0.5);
			}
			else if (autonomousRoutineStage == 1)
			{
				// Releases the gear onto the peg
				robotElectronics.getGearIntakeSolenoid().set(true);
				robotElectronics.getGearOuttakeSolenoid().set(true);
				autonomousRoutineStage = 99;
			}
			break;
		case 2: // Moves forward to the peg station and releases the gear (middle red station)
			if (robotElectronics.getDrivetrainEncoder().getDistance() <= auto2Value1 && autonomousRoutineStage == 0)
			{
				// Moves forward in a straight line
				robotDrive.drive(-0.5, -robotElectronics.getNavX().getAngle() * drivingKp);
			}
			else if (autonomousRoutineStage == 0)
			{
				// Stops the robot
				autonomousRoutineStage = 1;
				robotDrive.stopMotor();
				Timer.delay(0.5);
			}
			else if (autonomousRoutineStage == 1)
			{
				// Releases the gear onto the peg
				robotElectronics.getGearIntakeSolenoid().set(true);
				robotElectronics.getGearOuttakeSolenoid().set(true);
				autonomousRoutineStage = 99;
			}
			break;
		case 3: // Moves forward then turns right towards the peg station and releases the gear (left blue station)
			if (robotElectronics.getDrivetrainEncoder().getDistance() <= auto3Value1 && autonomousRoutineStage == 0)
			{
				// Moves forward in a straight line
				robotDrive.drive(-0.5, -robotElectronics.getNavX().getAngle() * drivingKp);
			}
			else if (autonomousRoutineStage == 0)
			{
				// Stops the robot
				autonomousRoutineStage = 1;
				robotDrive.stopMotor();
				Timer.delay(0.25);
				robotElectronics.getDrivetrainEncoder().reset();
			}
			else if(robotElectronics.getDrivetrainEncoder().getDistance() <= auto3Value2 && autonomousRoutineStage == 1)
			{
				// Moves forward in a curve towards the peg station
				robotDrive.drive(-0.5, -(robotElectronics.getNavX().getAngle() - 62) * drivingKp);
			}
			else if (autonomousRoutineStage == 1)
			{
				// Stops the robot
				autonomousRoutineStage = 2;
				robotDrive.stopMotor();
				Timer.delay(0.25);
			}
			else if (autonomousRoutineStage == 2)
			{
				// Releases the gear onto the peg
				robotElectronics.getGearIntakeSolenoid().set(true);
				robotElectronics.getGearOuttakeSolenoid().set(true);
				autonomousRoutineStage = 99;
			}
			break;
		case 4: // Moves forward then turns left towards the peg station and releases the gear (right blue station)
			if (robotElectronics.getDrivetrainEncoder().getDistance() <= auto4Value1 && autonomousRoutineStage == 0)
			{
				// Moves forward in a straight line
				robotDrive.drive(-0.5, -robotElectronics.getNavX().getAngle() * drivingKp);
			}
			else if (autonomousRoutineStage == 0)
			{
				// Stops the robot
				autonomousRoutineStage = 1;
				robotDrive.stopMotor();
				Timer.delay(0.25);
				robotElectronics.getDrivetrainEncoder().reset();
			}
			else if (robotElectronics.getDrivetrainEncoder().getDistance() <= auto4Value2 && autonomousRoutineStage == 1)
			{
				// Moves forward in a curve towards the peg station
				robotDrive.drive(-0.5, -(robotElectronics.getNavX().getAngle() + 62) * drivingKp);
			}
			else if (autonomousRoutineStage == 1)
			{
				// Stops the robot
				autonomousRoutineStage = 2;
				robotDrive.stopMotor();
				Timer.delay(0.25);
			}
			else if (autonomousRoutineStage == 2)
			{
				// Releases the gear onto the peg
				robotElectronics.getGearIntakeSolenoid().set(true);
				robotElectronics.getGearOuttakeSolenoid().set(true);
				autonomousRoutineStage = 99;
			}
			break;
		case 5: // Moves forward then turns right towards the peg station and releases the gear (left red station)
			if (robotElectronics.getDrivetrainEncoder().getDistance() <= auto5Value1 && autonomousRoutineStage == 0)
			{
				// Moves forward in a straight line
				robotDrive.drive(-0.5, -robotElectronics.getNavX().getAngle() * drivingKp);
			}
			else if (autonomousRoutineStage == 0)
			{
				// Stops the robot
				autonomousRoutineStage = 1;
				robotDrive.stopMotor();
				Timer.delay(0.25);
				robotElectronics.getDrivetrainEncoder().reset();
			}
			else if(robotElectronics.getDrivetrainEncoder().getDistance() <= auto5Value2 && autonomousRoutineStage == 1)
			{
				// Moves forward in a curve towards the peg station
				robotDrive.drive(-0.5, -(robotElectronics.getNavX().getAngle() - 62) * drivingKp);
			}
			else if (autonomousRoutineStage == 1)
			{
				// Stops the robot
				autonomousRoutineStage = 2;
				robotDrive.stopMotor();
				Timer.delay(0.25);
			}
			else if (autonomousRoutineStage == 2)
			{
				// Releases the gear onto the peg
				robotElectronics.getGearIntakeSolenoid().set(true);
				robotElectronics.getGearOuttakeSolenoid().set(true);
				autonomousRoutineStage = 99;
			}
			break;
		case 6: // Moves forward then turns left towards the peg station and releases the gear (right red station)
			if (robotElectronics.getDrivetrainEncoder().getDistance() <= auto6Value1 && autonomousRoutineStage == 0)
			{
				// Moves forward in a straight line
				robotDrive.drive(-0.5, -robotElectronics.getNavX().getAngle() * drivingKp);
			}
			else if (autonomousRoutineStage == 0)
			{
				// Stops the robot
				autonomousRoutineStage = 1;
				robotDrive.stopMotor();
				Timer.delay(0.25);
				robotElectronics.getDrivetrainEncoder().reset();
			}
			else if (robotElectronics.getDrivetrainEncoder().getDistance() <= auto6Value2 && autonomousRoutineStage == 1)
			{
				// Moves forward in a curve towards the peg station
				robotDrive.drive(-0.5, -(robotElectronics.getNavX().getAngle() + 62) * drivingKp);
			}
			else if (autonomousRoutineStage == 1)
			{
				// Stops the robot
				autonomousRoutineStage = 2;
				robotDrive.stopMotor();
				Timer.delay(0.25);
			}
			else if (autonomousRoutineStage == 2)
			{
				// Releases the gear onto the peg
				robotElectronics.getGearIntakeSolenoid().set(true);
				robotElectronics.getGearOuttakeSolenoid().set(true);
				autonomousRoutineStage = 99;
			}
			break;
		case 7: // Shoots 10 balls into the boiler then turns and moves into the autonomous zone (blue alliance)
			if (autonomousRoutineStage == 0)
			{
				// Turns on the flywheel
				robotElectronics.getFlywheelMotor().set(flywheelMotorMaxSpeed * (flywheelMotorPower / 100));

				// Lets the flywheel speed up and stabilize for 1 second before turning on the indexer
				Timer.delay(1);
				robotElectronics.getIndexerMotor().set(indexerMotorPower / 100);

				// Shoots balls for 5 seconds then stops the flywheel and indexer
				Timer.delay(5);
				robotElectronics.getFlywheelMotor().set(0);
				robotElectronics.getIndexerMotor().set(0);
				autonomousRoutineStage = 1;
			}
			else if (autonomousRoutineStage == 1 && robotElectronics.getDrivetrainEncoder().getDistance() <= auto7Value1)
			{
				// Attempts to rotate and face away from the boiler
				robotDrive.drive(0.5, -(robotElectronics.getNavX().getAngle() + 90) * drivingKp);
			}
			else if (autonomousRoutineStage == 1)
			{
				// Stops the robot
				autonomousRoutineStage = 2;
				robotDrive.stopMotor();
				Timer.delay(0.25);
				robotElectronics.getDrivetrainEncoder().reset();
			}
			else if (robotElectronics.getDrivetrainEncoder().getDistance() >= -120 && autonomousRoutineStage == 2)
			{
				// Moves forward in a curve into the autonomous zone
				robotDrive.drive(0.5, (robotElectronics.getNavX().getAngle() - 90) * drivingKp);
			}
			else if (autonomousRoutineStage == 2)
			{
				// Stops the robot
				autonomousRoutineStage = 99;
				robotDrive.stopMotor();
				Timer.delay(0.25);
			}
			break;
		case 8: // Shoots 10 balls into the boiler then turns and moves towards the peg station and releases the gear (red alliance)
			if (autonomousRoutineStage == 0)
			{
				// Turns on the flywheel
				robotElectronics.getFlywheelMotor().set(flywheelMotorMaxSpeed * (flywheelMotorPower / 100));

				// Lets the flywheel speed up and stabilize for 1 second before turning on the indexer
				Timer.delay(1);
				robotElectronics.getIndexerMotor().set(indexerMotorPower / 100);

				// Shoots balls for 7 seconds then stops the flywheel and indexer
				Timer.delay(7);
				robotElectronics.getFlywheelMotor().set(0);
				robotElectronics.getIndexerMotor().set(0);
				autonomousRoutineStage = 1;
			}
			else if (autonomousRoutineStage == 1 && robotElectronics.getDrivetrainEncoder().getDistance() <= auto8Value1)
			{
				// Moves forward in a curve towards the peg station
				robotDrive.drive(-0.5, -(robotElectronics.getNavX().getAngle() - 30) * drivingKp);
			}
			else if (autonomousRoutineStage == 1)
			{
				// Stops the robot
				autonomousRoutineStage = 2;
				robotDrive.stopMotor();
				Timer.delay(0.25);
				robotElectronics.getDrivetrainEncoder().reset();
			}
			else if (autonomousRoutineStage == 2 && robotElectronics.getDrivetrainEncoder().getDistance() <= auto8Value2)
			{
				// Moves forward in a curve towards the peg station
				robotDrive.drive(-0.5, -(robotElectronics.getNavX().getAngle() + 15) * drivingKp);
			}
			else if (autonomousRoutineStage == 2)
			{
				// Stops the robot
				autonomousRoutineStage = 3;
				robotDrive.stopMotor();
				Timer.delay(0.5);
			}
			else if (autonomousRoutineStage == 3)
			{
				// Releases the gear onto the peg
				robotElectronics.getGearIntakeSolenoid().set(true);
				robotElectronics.getGearOuttakeSolenoid().set(true);
				autonomousRoutineStage = 99;
			}
			break;
		case 9: // Shoots the pre-loaded balls into the boiler
			if (autonomousRoutineStage == 0)
			{
				// Turns on the flywheel
				robotElectronics.getFlywheelMotor().set(flywheelMotorMaxSpeed * (flywheelMotorPower / 100));

				// Lets the flywheel speed up and stabilize for 1 second before turning on the indexer
				Timer.delay(1);
				robotElectronics.getIndexerMotor().set(indexerMotorPower / 100);

				// Shoots balls for 10 seconds then stops the flywheel and indexer
				Timer.delay(10);
				robotElectronics.getFlywheelMotor().set(0);
				robotElectronics.getIndexerMotor().set(0);
				autonomousRoutineStage = 99;
			}
			break;
		default: // Do nothing
			robotDrive.stopMotor();
			break;
		}

		// Gets the values from the SmartDashboard
		getSmartDashboardValues();

		// Calls the function to update the SmartDashboard window's values
		updateSmartDashboard();
	}

	// Function that is called once each time the robot enters tele-operated mode
	public void teleopInit() 
	{
		// Sets the allowable margin of error for the flywheel
		robotElectronics.getFlywheelMotor().setAllowableClosedLoopErr(flywheelAcceptableError);

		// Sets the latest PID values for the flywheel
		robotElectronics.getFlywheelMotor().setPID(flywheelP, flywheelI, flywheelD, flywheelF, 0, flywheelRampRate, 0);

		// Resets the navX
		robotElectronics.getNavX().reset();

		// Resets the drivetrain encoder
		robotElectronics.getDrivetrainEncoder().reset();

		// Zeroes out the flywheel's encoder values
		robotElectronics.getFlywheelMotor().setEncPosition(0);

		// Turns on the pneumatic compressor
		robotElectronics.getPneumaticCompressor().setClosedLoopControl(true);
	}

	// Function that is called periodically during tele-operated mode
	public void teleopPeriodic() 
	{
		// If the Start button is pressed on the primary xbox controller runs the specified autonomous routine
		if (robotElectronics.getPrimaryXboxController().getRawButton(8) && stick1BtnPressed[8] == false)
		{
			stick1BtnPressed[8] = true;
			/*autonomousInit();
			while (autonomousRoutineStage != 99)
			{
				autonomousPeriodic();
			}*/
		}
		// Resets the button's status
		else if (robotElectronics.getPrimaryXboxController().getRawButton(8) == false && stick1BtnPressed[8])
		{
			stick1BtnPressed[8] = false;
		}

		// If the Back button is pressed on the primary xbox controller toggles the indexer motor (backwards and off)
		if (robotElectronics.getPrimaryXboxController().getRawButton(7) && stick1BtnPressed[7] == false)
		{
			stick1BtnPressed[7] = true;
			if (Math.round(robotElectronics.getIndexerMotor().get() * 100) != -indexerMotorPower)
			{
				robotElectronics.getIndexerMotor().set(-indexerMotorPower / 100);
			}
			else if (Math.round(robotElectronics.getIndexerMotor().get() * 100) == -indexerMotorPower)
			{
				robotElectronics.getIndexerMotor().set(0);
			}
		}
		// Resets the button's status
		else if (robotElectronics.getPrimaryXboxController().getRawButton(7) == false && stick1BtnPressed[7])
		{
			stick1BtnPressed[7] = false;
		}

		// If the X button is pressed on the primary xbox controller opens both the gear intake and outtake solenoids (closes gear pickup solenoid)
		if (robotElectronics.getPrimaryXboxController().getRawButton(3) && stick1BtnPressed[3] == false)
		{	
			stick1BtnPressed[3] = true;
			robotElectronics.getGearPickupMotor().set(0);
			robotElectronics.getGearPickupSolenoid1().set(false);
			robotElectronics.getGearPickupSolenoid2().set(true);
			robotElectronics.getGearIntakeSolenoid().set(true);
			robotElectronics.getGearOuttakeSolenoid().set(true);
		}
		// Resets the button's status
		else if (robotElectronics.getPrimaryXboxController().getRawButton(3) == false && stick1BtnPressed[3])
		{
			stick1BtnPressed[3] = false;
		}

		// If the Y button is pressed on the primary xbox controller opens the gear intake solenoid and closes the gear outtake solenoid (closes gear pickup solenoid)
		if (robotElectronics.getPrimaryXboxController().getRawButton(4) && stick1BtnPressed[4] == false)
		{	
			stick1BtnPressed[4] = true;
			robotElectronics.getGearPickupMotor().set(0);
			robotElectronics.getGearPickupSolenoid1().set(false);
			robotElectronics.getGearPickupSolenoid2().set(true);
			robotElectronics.getGearIntakeSolenoid().set(true);
			robotElectronics.getGearOuttakeSolenoid().set(false);
		}
		// Resets the button's status
		else if (robotElectronics.getPrimaryXboxController().getRawButton(4) == false && stick1BtnPressed[4])
		{
			stick1BtnPressed[4] = false;
		}

		// If the B button is pressed on the primary xbox controller closes both the gear intake and outtake solenoids
		if (robotElectronics.getPrimaryXboxController().getRawButton(2) && stick1BtnPressed[2] == false)
		{	
			stick1BtnPressed[2] = true;
			robotElectronics.getGearIntakeSolenoid().set(false);
			robotElectronics.getGearOuttakeSolenoid().set(false);
		}
		// Resets the button's status
		else if (robotElectronics.getPrimaryXboxController().getRawButton(2) == false && stick1BtnPressed[2])
		{
			stick1BtnPressed[2] = false;
		}

		// If the A button is pressed on the primary xbox controller toggles the gear pickup (outtake)
		if (robotElectronics.getPrimaryXboxController().getRawButton(1) && stick1BtnPressed[1] == false)
		{	
			stick1BtnPressed[1] = true;
			if (robotElectronics.getGearPickupSolenoid1().get() == true)
			{
				robotElectronics.getGearPickupSolenoid1().set(false);
				robotElectronics.getGearPickupSolenoid2().set(true);
				robotElectronics.getGearPickupMotor().set(0);
			}
			else
			{
				robotElectronics.getGearIntakeSolenoid().set(false);
				robotElectronics.getGearOuttakeSolenoid().set(false);
				robotElectronics.getGearPickupSolenoid1().set(true);
				robotElectronics.getGearPickupSolenoid2().set(false);
				robotElectronics.getGearPickupMotor().set(100);
			}
		}
		// Resets the button's status
		else if (robotElectronics.getPrimaryXboxController().getRawButton(1) == false && stick1BtnPressed[1])
		{
			stick1BtnPressed[1] = false;
		}

		// If the Left Bumper button is pressed on the primary xbox controller toggles the indexer motor (forwards and off)
		if (robotElectronics.getPrimaryXboxController().getRawButton(5) && stick1BtnPressed[5] == false)
		{
			stick1BtnPressed[5] = true;
			if (Math.round(robotElectronics.getIndexerMotor().get() * 100) != indexerMotorPower)
			{
				robotElectronics.getIndexerMotor().set(indexerMotorPower / 100);
			}
			else if (Math.round(robotElectronics.getIndexerMotor().get() * 100) == indexerMotorPower)
			{
				robotElectronics.getIndexerMotor().set(0);
			}
		}
		// Resets the button's status
		else if (robotElectronics.getPrimaryXboxController().getRawButton(5) == false && stick1BtnPressed[5])
		{
			stick1BtnPressed[5] = false;
		}

		// If the Right Bumper button is pressed on the primary xbox controller toggles the gear pickup (intake)
		if (robotElectronics.getPrimaryXboxController().getRawButton(6) && stick1BtnPressed[6] == false)
		{
			stick1BtnPressed[6] = true;
			if (robotElectronics.getGearPickupSolenoid1().get() == true)
			{
				robotElectronics.getGearPickupSolenoid1().set(false);
				robotElectronics.getGearPickupSolenoid2().set(true);
				robotElectronics.getGearPickupMotor().set(0);
				hasTriggeredRumble = false;
				robotElectronics.getPrimaryXboxController().setRumble(RumbleType.kLeftRumble, 0);
				robotElectronics.getPrimaryXboxController().setRumble(RumbleType.kRightRumble, 0);
			}
			else
			{
				robotElectronics.getGearIntakeSolenoid().set(false);
				robotElectronics.getGearOuttakeSolenoid().set(false);
				robotElectronics.getGearPickupSolenoid1().set(true);
				robotElectronics.getGearPickupSolenoid2().set(false);
				robotElectronics.getGearPickupMotor().set(-100);
			}
		}
		// Resets the button's status
		else if (robotElectronics.getPrimaryXboxController().getRawButton(6) == false && stick1BtnPressed[6])
		{
			stick1BtnPressed[6] = false;
		}

		// If the Down D-Pad button is pressed on the primary xbox controller sets the flywheel's speed to 0%
		if (robotElectronics.getPrimaryXboxController().getPOV(0) == 180)
		{
			robotElectronics.getFlywheelMotor().set(flywheelMotorMaxSpeed * 0);
			robotElectronics.getFlywheelMotor().clearIAccum();
		}

		// If the Left D-Pad button is pressed on the primary xbox controller sets the flywheel's speed to 60%
		if (robotElectronics.getPrimaryXboxController().getPOV(0) == 270)
		{
			robotElectronics.getFlywheelMotor().set(flywheelMotorMaxSpeed * 0.60);
			robotElectronics.getFlywheelMotor().clearIAccum();
		}

		// If the Up D-Pad button is pressed on the primary xbox controller sets the flywheel's speed to 70%
		if (robotElectronics.getPrimaryXboxController().getPOV(0) == 0)
		{
			robotElectronics.getFlywheelMotor().set(flywheelMotorMaxSpeed * 0.70);
			robotElectronics.getFlywheelMotor().clearIAccum();
		}

		// If the Right D-Pad button is pressed on the primary xbox controller sets the flywheel's speed to the SmartDashboard value
		if (robotElectronics.getPrimaryXboxController().getPOV(0) == 90)
		{
			robotElectronics.getFlywheelMotor().set(flywheelMotorMaxSpeed * (flywheelMotorPower / 100));
			robotElectronics.getFlywheelMotor().clearIAccum();
		}

		// Sends the Right Trigger button's relevant power value to the intake + climbing motor
		robotElectronics.getClimberMotor().set(robotElectronics.getPrimaryXboxController().getRawAxis(3));

		// Checks to see if the primary joystick's rumble motors needs to be toggled
		if (hasTriggeredRumble == false && (robotElectronics.getGearPickupSolenoid1().get() == true || robotElectronics.getGearPickupSolenoid2().get() == false) && (!robotElectronics.getGearPickupButton1().get() || !robotElectronics.getGearPickupButton2().get() || !robotElectronics.getGearPickupButton3().get() || !robotElectronics.getGearPickupButton4().get()))
		{
			hasTriggeredRumble = true;
			robotElectronics.getPrimaryXboxController().setRumble(RumbleType.kLeftRumble, 1);
			robotElectronics.getPrimaryXboxController().setRumble(RumbleType.kRightRumble, 1);
		}

		// Gets the values from the SmartDashboard
		getSmartDashboardValues();

		// Calls the function to update the SmartDashboard window's values
		updateSmartDashboard();

		// Calls the function to move / drive the robot
		robotDrive.arcadeDrive(robotElectronics.getPrimaryXboxController(), 5, robotElectronics.getPrimaryXboxController(), 0, true);
	}

	// Function to update the visually presented data in the SmartDashboard window
	public void updateSmartDashboard()
	{
		SmartDashboard.putNumber("Indexer Motor Power (%)", indexerMotorPower);
		SmartDashboard.putNumber("Intake + Climber Motor Power (%)", intakeClimberMotorPower);
		SmartDashboard.putNumber("Flywheel Motor Power (%)", flywheelMotorPower);
		SmartDashboard.putNumber("Flywheel P", flywheelP);
		SmartDashboard.putNumber("Flywheel I", flywheelI);
		SmartDashboard.putNumber("Flywheel D", flywheelD);
		SmartDashboard.putNumber("Flywheel F", flywheelF);
		SmartDashboard.putNumber("Flywheel Ramp Rate", flywheelRampRate);
		SmartDashboard.putNumber("Flywheel Max Speed", flywheelMotorMaxSpeed);
		SmartDashboard.putNumber("Flywheel Acceptable Error", flywheelAcceptableError);
		SmartDashboard.putNumber("Flywheel Encoder Value", robotElectronics.getFlywheelMotor().getEncPosition());
		SmartDashboard.putNumber("Flywheel Encoder Velocity", robotElectronics.getFlywheelMotor().getEncVelocity());
		SmartDashboard.putNumber("Flywheel Error", robotElectronics.getFlywheelMotor().getClosedLoopError());
		SmartDashboard.putString("Target Outline Dimensions", targetOutlineSize);
		SmartDashboard.putNumber("Ultrasonic Front (Inch)", robotElectronics.getUltrasonicSensor().getRangeInches());
		SmartDashboard.putNumber("navX Angle", robotElectronics.getNavX().getAngle());
		SmartDashboard.putNumber("navX Displacement X", robotElectronics.getNavX().getDisplacementX());
		SmartDashboard.putNumber("navX Displacement Y", robotElectronics.getNavX().getDisplacementY());
		SmartDashboard.putNumber("navX Displacement Z", robotElectronics.getNavX().getDisplacementZ());
		SmartDashboard.putNumber("navX Heading", robotElectronics.getNavX().getFusedHeading());
		SmartDashboard.putNumber("Autonomous Routine", autonomousRoutine);
		SmartDashboard.putNumber("Drivetrain Encoder Value", robotElectronics.getDrivetrainEncoder().get());
		SmartDashboard.putNumber("Drivetrain Encoder Distance", robotElectronics.getDrivetrainEncoder().getDistance());
		SmartDashboard.putNumber("Driving kP", drivingKp);
		SmartDashboard.putNumber("A1 V1", auto1Value1);
		SmartDashboard.putNumber("A2 V1", auto2Value1);
		SmartDashboard.putNumber("A3 V1", auto3Value1);
		SmartDashboard.putNumber("A3 V2", auto3Value2);
		SmartDashboard.putNumber("A4 V1", auto4Value1);
		SmartDashboard.putNumber("A4 V2", auto4Value2);
		SmartDashboard.putNumber("A5 V1", auto5Value1);
		SmartDashboard.putNumber("A5 V2", auto5Value2);
		SmartDashboard.putNumber("A6 V1", auto6Value1);
		SmartDashboard.putNumber("A6 V2", auto6Value2);
		SmartDashboard.putNumber("A7 V1", auto7Value1);
		SmartDashboard.putNumber("A8 V1", auto8Value1);
		SmartDashboard.putNumber("A8 V2", auto8Value2);
	}

	// Function to get the values from the SmartDashboard window
	public void getSmartDashboardValues()
	{
		indexerMotorPower = SmartDashboard.getNumber("Indexer Motor Power (%)", indexerMotorPower);
		intakeClimberMotorPower = SmartDashboard.getNumber("Intake + Climber Motor Power (%)", intakeClimberMotorPower);
		flywheelMotorPower = SmartDashboard.getNumber("Flywheel Motor Power (%)", flywheelMotorPower);
		flywheelP = SmartDashboard.getNumber("Flywheel P", flywheelP);
		flywheelI = SmartDashboard.getNumber("Flywheel I", flywheelI);
		flywheelD = SmartDashboard.getNumber("Flywheel D", flywheelD);
		flywheelF = SmartDashboard.getNumber("Flywheel F", flywheelF);
		flywheelRampRate = SmartDashboard.getNumber("Flywheel Ramp Rate", flywheelRampRate);
		flywheelMotorMaxSpeed = SmartDashboard.getNumber("Flywheel Max Speed", flywheelMotorMaxSpeed);
		flywheelAcceptableError = (int) SmartDashboard.getNumber("Flywheel Acceptable Error", flywheelAcceptableError);
		drivingKp = SmartDashboard.getNumber("Driving kP", drivingKp);
		autonomousRoutine = (int) SmartDashboard.getNumber("Autonomous Routine", autonomousRoutine);
		auto1Value1 = SmartDashboard.getNumber("A1 V1", auto1Value1);
		auto2Value1 = SmartDashboard.getNumber("A2 V1", auto2Value1);
		auto3Value1 = SmartDashboard.getNumber("A3 V1", auto3Value1);
		auto3Value2 = SmartDashboard.getNumber("A3 V2", auto3Value2);
		auto4Value1 = SmartDashboard.getNumber("A4 V1", auto4Value1);
		auto4Value2 = SmartDashboard.getNumber("A4 V2", auto4Value2);
		auto5Value1 = SmartDashboard.getNumber("A5 V1", auto5Value1);
		auto5Value2 = SmartDashboard.getNumber("A5 V2", auto5Value2);
		auto6Value1 = SmartDashboard.getNumber("A6 V1", auto6Value1);
		auto6Value2 = SmartDashboard.getNumber("A6 V2", auto6Value2);
		auto7Value1 = SmartDashboard.getNumber("A7 V1", auto7Value1);
		auto8Value1 = SmartDashboard.getNumber("A8 V1", auto8Value1);
		auto8Value2 = SmartDashboard.getNumber("A8 V2", auto8Value2);
	}
}
