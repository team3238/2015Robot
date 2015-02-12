package org.usfirst.frc.team3238.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;

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
    CANTalon leftFrontMotorController, rightFrontMotorController, 
            leftRearMotorController, rightRearMotorController;
    CANTalon grabberVerticalTalon, grabberHorizontalTalon;
    CANTalon leftLifterTalon, rightLifterTalon;
    DigitalInput reflectSensorFront;
    DigitalInput reflectSensorRear;
    AnalogInput gyroSensor;
    BuiltInAccelerometer accelerometer;
    AnalogInput infraredSensor;
    AnalogInput sonarSensor;
    ToteLifter toteLifter;
    Grabber grabber;
    ArrayList<String> fileContents;
    AnalogInput grabberVerticalPot;
    AnalogInput grabberHorizontalPot;
    AnalogInput lifterLeftPot;
    AnalogInput lifterRightPot;
    Joystick joystickZero, joystickOne;

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
    double m_canExtendHeight;
    double m_canGrabHeight;
    double m_toteExtendHeight;
    double m_toteGrabHeight;
    double m_stepCanExtendHeight;
    double m_stepCanGrabHeight;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit()
    {
    	// Talon Ports
    		//Chassis
		final int CHASSISLEFTFRONTTALONID = 2;
		final int CHASSISLEFTREARTALONID = 4;
		final int CHASSISRIGHTFRONTTALONID = 8;
		final int CHASSISRIGHTREARTALONID = 6;
			//Grabber
		final int GRABBERVERTICALTALONID = 1;
		final int GRABBERHORIZONTALTALONID = 3;
			//Lifter
		final int LIFTERLEFTTALONID = 5;
		final int LIFTERRIGHTTALONID = 7;
		
		// Digital Inputs
		final int REFLECTSENSORFRONTPORT = 5;
		final int REFLECTSENSORREARPORT = 6;
		
		// Analog Inputs
		final int IRSENSORPORT = 7;
		final int SONARSENSORPORT = 0;
		final int GYROSENSORPORT = 6;
		
		final int GRABBERVERTICALPOTPORT = 4;
		final int GRABBERHORIZONTALPOTPORT = 5;
		final int LIFTERLEFTPOTPORT = 2;
		final int LIFTERRIGHTPOTPORT = 3;
		
		// Servo Ports
		final int SERVOLEFTPORT = 2;
		final int SERVORIGHTPORT = 4;
		
		// Driver Station Inputs
		final int JOYSTICKPORT = 0;
    			
    			
		joystickZero = new Joystick(JOYSTICKPORT);
		joystickOne = new Joystick(1);
    			
        fileContents = FileReader.readFile("RobotConstants.txt");
        
        piControllerLifterLeft = new PIController(
                Double.parseDouble(fileContents.get(27)),
                Double.parseDouble(fileContents.get(30)));
        piControllerLifterRight = new PIController(
                Double.parseDouble(fileContents.get(33)),
                Double.parseDouble(fileContents.get(36)));
        
        m_grabberHorizontalI = Double.parseDouble(fileContents.get(59));
        m_grabberHorizontalP = Double.parseDouble(fileContents.get(56));
        m_grabberVerticalI = Double.parseDouble(fileContents.get(53));
        m_grabberVerticalP = Double.parseDouble(fileContents.get(50));
        
        m_accuracyThreshold = Double.parseDouble(fileContents.get(39));
        m_openServoPosition = Double.parseDouble(fileContents.get(45));
        m_closeServoPosition = Double.parseDouble(fileContents.get(42));
        
        m_spinThreshold = Double.parseDouble(fileContents.get(22));
        m_infraredDistanceTrigger = Double.parseDouble(fileContents.get(5));
        m_timeIgnore = Long.parseLong(fileContents.get(8));
        m_timeDriveBack = Integer.parseInt(fileContents.get(11));
        m_timeDriveAway = Integer.parseInt(fileContents.get(14));
        m_moveBackSpeed = Double.parseDouble(fileContents.get(17));

        m_canExtendHeight = Double.parseDouble(fileContents.get(62));
        m_canGrabHeight = Double.parseDouble(fileContents.get(65));
        m_toteExtendHeight = Double.parseDouble(fileContents.get(68));
        m_toteGrabHeight = Double.parseDouble(fileContents.get(71));
        m_stepCanExtendHeight = Double.parseDouble(fileContents.get(74)); 
        m_stepCanGrabHeight = Double.parseDouble(fileContents.get(77));

        leftFrontMotorController = new CANTalon(CHASSISLEFTFRONTTALONID);
        rightFrontMotorController = new CANTalon(CHASSISRIGHTFRONTTALONID);
        leftRearMotorController = new CANTalon(CHASSISLEFTREARTALONID);
        rightRearMotorController = new CANTalon(CHASSISRIGHTREARTALONID);
        
        grabberVerticalTalon = new CANTalon(GRABBERVERTICALTALONID);
        grabberHorizontalTalon = new CANTalon(GRABBERHORIZONTALTALONID);
        grabberVerticalPot = new AnalogInput(GRABBERVERTICALPOTPORT);
        grabberHorizontalPot = new AnalogInput(GRABBERHORIZONTALPOTPORT);
        lifterLeftPot = new AnalogInput(LIFTERLEFTPOTPORT);
        lifterRightPot = new AnalogInput(LIFTERRIGHTPOTPORT);
        
        reflectSensorFront = new DigitalInput(REFLECTSENSORFRONTPORT);
        reflectSensorRear = new DigitalInput(REFLECTSENSORREARPORT);
        gyroSensor = new AnalogInput(GYROSENSORPORT);
        infraredSensor = new AnalogInput(IRSENSORPORT);
        sonarSensor = new AnalogInput(SONARSENSORPORT);
        accelerometer = new BuiltInAccelerometer();
        
        leftLifterTalon = new CANTalon(LIFTERLEFTTALONID);
        rightLifterTalon = new CANTalon(LIFTERRIGHTTALONID);
        
        chassis = new Chassis(leftFrontMotorController,
                leftRearMotorController, rightFrontMotorController,
                rightRearMotorController);

        grabber = new Grabber(grabberVerticalTalon, grabberHorizontalTalon, 
        		grabberVerticalPot, grabberHorizontalPot, sonarSensor, 
        		m_grabberVerticalP, m_grabberVerticalI,  m_grabberHorizontalP, 
        		m_grabberHorizontalI, m_canExtendHeight, m_canGrabHeight, 
                m_toteExtendHeight, m_toteGrabHeight, m_stepCanExtendHeight, 
                m_stepCanGrabHeight);
        
        toteLifter = new ToteLifter(leftLifterTalon, rightLifterTalon, 
                SERVORIGHTPORT, SERVOLEFTPORT, lifterLeftPot, lifterRightPot, 
        		piControllerLifterLeft, piControllerLifterRight, 
        		m_accuracyThreshold, m_openServoPosition, m_closeServoPosition);
        
        autonomous = new Autonomous(chassis, m_infraredDistanceTrigger, 
        		m_timeIgnore, m_timeDriveBack, m_timeDriveAway, 
        		m_moveBackSpeed);
    }

    /**
     * 
     */
    public void autonomousInit()
    {        
        fileContents = FileReader.readFile("RobotConstants.txt");
        m_grabberHorizontalI = Double.parseDouble(fileContents.get(59));
        m_grabberHorizontalP = Double.parseDouble(fileContents.get(56));
        m_grabberVerticalI = Double.parseDouble(fileContents.get(53));
        m_grabberVerticalP = Double.parseDouble(fileContents.get(50));
        
        grabber.inputPIConstants(m_grabberVerticalP, m_grabberVerticalI, 
        		m_grabberHorizontalP, m_grabberHorizontalI);
        System.out.print(m_grabberVerticalP);
        System.out.println(" " + m_grabberVerticalI);
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic()
    {
        chassis.setJoystickData(0, 0, 0);
        chassis.idle();

        grabber.goToHeight(0.689);
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
        grabberVerticalTalon.set(-joystickZero.getY());
        grabberHorizontalTalon.set(joystickOne.getY());
//        System.out.println(sonarSensor.getVoltage());
//        rightLifterTalon.set(joystickOne.getY());
//        leftLifterTalon.set(joystickZero.getY());
        
//       grabberVerticalTalon.set(joystick.getY());
//        chassis.setJoystickData(0, 0, 0);
//        chassis.idle();
        double verticalPotVolt;
        double horizontalPotVolt;
        verticalPotVolt = grabberVerticalPot.getVoltage();
        horizontalPotVolt = grabberHorizontalPot.getVoltage();
        System.out.println(horizontalPotVolt);
//        
//        System.out.println(-0.2608156852*verticalPotVolt+1.421847684);
        
//        System.out.println(
//        		"Vert: "+verticalPotVolt+"  Hori: "+horizontalPotVolt);
//    	double x = joystick.getX();
//		double y = joystick.getY();
//		double twist = joystick.getTwist();
//		chassis.setJoystickData(-y, -x, twist);
//		chassis.idle();
    }
    
    public void disabledPeriodic()
    {
        double verticalPotVolt;
        double horizontalPotVolt;
        verticalPotVolt = grabberVerticalPot.getVoltage();
        horizontalPotVolt = grabberHorizontalPot.getVoltage();
        
//        System.out.println(sonarSensor.getVoltage());
        
        //System.out.println(
        //        "Vert: "+verticalPotVolt+"  Hori: "+horizontalPotVolt);
        //System.out.println(-0.2608156852*verticalPotVolt+1.421847684);
        System.out.println(horizontalPotVolt);
    }

    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic()
    {

    }
}
