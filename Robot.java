package org.usfirst.frc.team3238.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.IterativeRobot;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot
{

    Autonomous autonomous;
    Chassis chassis;
    PIController piControllerGrabber;
    PIController piControllerLifterLeft;
    PIController piControllerLifterRight;
    CANTalon leftFrontMotorController;
    CANTalon rightFrontMotorController;
    CANTalon leftRearMotorController;
    CANTalon rightRearMotorController;
    DigitalInput reflectSensorFront;
    DigitalInput reflectSensorRear;
    AnalogInput gyroSensor;
    BuiltInAccelerometer accelerometer;
    AnalogInput infraredSensor;
    AnalogInput sonarSensor;
    ToteLifter toteLifter;
    Grabber grabber;
    ArrayList<String> fileContents;

    double m_spinThreshold;
    double m_infraredDistanceTrigger;
    long m_timeIgnore;
    int m_timeDriveBack;
    int m_timeDriveAway;
    double m_moveBackSpeed;
    double m_grabberHorizontalI;
    double m_grabberHorizontalP;
    double m_grabberVerticalI;
    double m_grabberVerticalP;
    double m_accuracyThreshold;
    double m_openServoPosition;
    double m_closeServoPosition;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit()
    {
    	// Talon Ports
    		//Chassis
		final int CHASSISLEFTFRONTTALONID = 6;
		final int CHASSISLEFTREARTALONID = 8;
		final int CHASSISRIGHTFRONTTALONID = 7;
		final int CHASSISRIGHTREARTALONID = 3;
			//Grabber
		final int GRABBERVERTICALTALONID = 4;
		final int GRABBERHORIZONTALTALONID = 5;
			//Lifter
		final int LIFTERTALONID = 1;
		
		// Digital Inputs
		final int REFLECTSENSORFRONTPORT = 5;
		final int REFLECTSENSORREARPORT = 6;
		
		// Analog Inputs
		final int IRSENSORPORT = 7;
		final int SONARSENSORPORT = 1;
		final int GYROSENSORPORT = 6;
		
		// Servo Ports
		final int SERVOLEFTPORT = 2;
		final int SERVORIGHTPORT = 2;
		
		// Driver Station Inputs
		final int JOYSTICKPORT = 1;
    			
    			
    			
    			
        fileContents = FileReader.readFile("RobotConstants.txt");
        
        piControllerLifterLeft = new PIController(
                Double.parseDouble(fileContents.get(2)),
                Double.parseDouble(fileContents.get(2)));
        piControllerLifterRight = new PIController(
                Double.parseDouble(fileContents.get(2)),
                Double.parseDouble(fileContents.get(2)));
        
        m_grabberHorizontalI = Double.parseDouble(fileContents.get(2));
        m_grabberHorizontalP = Double.parseDouble(fileContents.get(2));
        m_grabberVerticalI = Double.parseDouble(fileContents.get(2));
        m_grabberVerticalP = Double.parseDouble(fileContents.get(2));
        
        m_accuracyThreshold = Double.parseDouble(fileContents.get(2));
        m_openServoPosition = Double.parseDouble(fileContents.get(2));
        m_closeServoPosition = Double.parseDouble(fileContents.get(2));
        
        m_spinThreshold = Double.parseDouble(fileContents.get(2));
        m_infraredDistanceTrigger = Double.parseDouble(fileContents.get(2));
        m_timeIgnore = Long.parseLong(fileContents.get(2));
        m_timeDriveBack = Integer.parseInt(fileContents.get(2));
        m_timeDriveAway = Integer.parseInt(fileContents.get(2));
        m_moveBackSpeed = Double.parseDouble(fileContents.get(2));
        

        leftFrontMotorController = new CANTalon(CHASSISLEFTFRONTTALONID);
        rightFrontMotorController = new CANTalon(CHASSISRIGHTFRONTTALONID);
        leftRearMotorController = new CANTalon(CHASSISLEFTREARTALONID);
        rightRearMotorController = new CANTalon(CHASSISRIGHTREARTALONID);
        
        reflectSensorFront = new DigitalInput(REFLECTSENSORFRONTPORT);
        reflectSensorRear = new DigitalInput(REFLECTSENSORREARPORT);
        gyroSensor = new AnalogInput(GYROSENSORPORT);
        infraredSensor = new AnalogInput(IRSENSORPORT);
        sonarSensor = new AnalogInput(SONARSENSORPORT);
        accelerometer = new BuiltInAccelerometer();
        
        
        chassis = new Chassis(leftFrontMotorController,
                rightFrontMotorController, leftRearMotorController,
                rightRearMotorController);

        grabber = new Grabber(GRABBERVERTICALTALONID, GRABBERHORIZONTALTALONID, 
        		3, 4, sonarSensor, m_grabberVerticalP, 
        		m_grabberVerticalI,  m_grabberHorizontalP, 
        		m_grabberHorizontalI);
        
        toteLifter = new ToteLifter(LIFTERTALONID, SERVORIGHTPORT, 
        		SERVOLEFTPORT, 4, 5, piControllerLifterLeft,
                piControllerLifterRight, m_accuracyThreshold, 
                m_openServoPosition, m_closeServoPosition);
        
        autonomous = new Autonomous(chassis, m_infraredDistanceTrigger, 
        		m_timeIgnore, m_timeDriveBack, m_timeDriveAway, 
        		m_moveBackSpeed);
    }

    /**
     * 
     */
    public void autonomousInit()
    {
        autonomous.init();
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic()
    {
<<<<<<< HEAD
        autonomous.idle(chassis, reflectSensorRear, reflectSensorFront,
                gyroSensor.getValue(), m_spinThreshold, accelerometer,
                infraredSensor.getVoltage(), toteLifter, grabber,
                piControllerLifterLeft, piControllerLifterRight);
=======
       
>>>>>>> ca1d62692512e4affa98c7780303b6c572ef0f57
    }

    /**
     * 
     */
    public void teleopInit()
    {

    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic()
    {

    }

    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic()
    {

    }
}
