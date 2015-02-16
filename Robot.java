package org.usfirst.frc.team3238.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;

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
    PIController piControllerLifterLeft, piControllerLifterRight;
    CANTalon leftFrontTalon, rightFrontTalon, 
            leftRearTalon, rightRearTalon;
    CANTalon grabberVerticalTalon, grabberHorizontalTalon;
    CANTalon leftLifterTalon, rightLifterTalon;
    DigitalInput reflectSensorFront, reflectSensorRear;
    AnalogInput gyroSensor;
    BuiltInAccelerometer accelerometer;
    AnalogInput infraredSensor, sonarSensor;
    ToteLifter toteLifter;
    Grabber grabber;
    ArrayList<String> fileContents;
    AnalogInput grabberVerticalPot, grabberHorizontalPot;
    AnalogInput lifterLeftPot, lifterRightPot;
    Joystick joystickZero, joystickOne, joystickTwo;
    Servo leftLifterServo, rightLifterServo;

    //Subsytem Constants
    double m_lifterLeftP;
    double m_lifterLeftI;
    double m_lifterRightP;
    double m_lifterRightI;
    double m_lifterAccuracyThreshold;
        
    double m_grabberHorizontalP;
    double m_grabberHorizontalI;
    double m_grabberVerticalP;
    double m_grabberVerticalI;
        
    double m_spinThreshold;
    double m_infraredDistanceTrigger;
    long m_timeIgnore;
    int m_timeDriveBack;
    int m_timeDriveAway;
    double m_moveBackSpeed;

    double m_canExtendHeight;
    double m_canGrabHeight;
    double m_toteExtendHeight;
    double m_toteGrabHeight;
    double m_stepCanExtendHeight; 
    double m_stepCanGrabHeight;
    double m_retractedLocation;
    double m_horizontalThreshold;
    double m_verticalThreshold;
    double m_pauseDistanceFromObject;
    double m_grabberHorizontalGentleP;
    double m_grabberHorizontalGentleI;
    double m_grabberHorizontalHome;
    double m_grabberVerticalHome;
        
    double m_lifterHomeHeight;
    double m_lifterWaitLiftPosition;
    double m_lifterOpenDogsLiftPosition;
    double m_lifterCloseDogsLiftPosition;
        
    double m_slowDownRetractThreshold;
    
    double m_homeOffset;
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit()
    {
    	//Talon Ports
    		//Chassis
		final int CHASSISLEFTFRONTTALONID = 2;
		final int CHASSISLEFTREARTALONID = 4;
		final int CHASSISRIGHTFRONTTALONID = 8;
		final int CHASSISRIGHTREARTALONID = 6;
			//Grabber
		final int GRABBERVERTICALTALONID = 1;
		final int GRABBERHORIZONTALTALONID = 3;
			//Lifter
		final int LIFTERLEFTTALONID = 7;
		final int LIFTERRIGHTTALONID = 5;
		
		// Digital Inputs
		final int REFLECTSENSORFRONTPORT = 5;
		final int REFLECTSENSORREARPORT = 6;
		
		// Analog Inputs
            //Autonomous
		final int ULTRASONICPORT = 0;
		final int IRSENSORPORT = 1;
		final int GYROSENSORPORT = 2;
		    //Grabber
        final int SONARSENSORPORT = 0;
		final int GRABBERVERTICALPOTPORT = 4;
		final int GRABBERHORIZONTALPOTPORT = 5;
		    //Lifter
		final int LIFTERLEFTPOTPORT = 7;
		final int LIFTERRIGHTPOTPORT = 6;
		
		// Servo Ports
		final int SERVOLEFTPORT = 0;
		final int SERVORIGHTPORT = 1;
		
		// Driver Station Inputs
		final int JOYSTICKPORT = 0;
    				
		joystickZero = new Joystick(JOYSTICKPORT);
		joystickOne = new Joystick(1);
		joystickTwo = new Joystick(2);
    			 
        leftFrontTalon = new CANTalon(CHASSISLEFTFRONTTALONID);
        rightFrontTalon = new CANTalon(CHASSISRIGHTFRONTTALONID);
        leftRearTalon = new CANTalon(CHASSISLEFTREARTALONID);
        rightRearTalon = new CANTalon(CHASSISRIGHTREARTALONID);
        
        grabberVerticalTalon = new CANTalon(GRABBERVERTICALTALONID);
        grabberHorizontalTalon = new CANTalon(GRABBERHORIZONTALTALONID);
        grabberVerticalPot = new AnalogInput(GRABBERVERTICALPOTPORT);
        grabberHorizontalPot = new AnalogInput(GRABBERHORIZONTALPOTPORT);
        grabberVerticalPot.setAverageBits(3);
        grabberHorizontalPot.setAverageBits(3);
        
        reflectSensorFront = new DigitalInput(REFLECTSENSORFRONTPORT);
        reflectSensorRear = new DigitalInput(REFLECTSENSORREARPORT);
        gyroSensor = new AnalogInput(GYROSENSORPORT);
        infraredSensor = new AnalogInput(IRSENSORPORT);
        sonarSensor = new AnalogInput(SONARSENSORPORT);
        accelerometer = new BuiltInAccelerometer();
        
        leftLifterTalon = new CANTalon(LIFTERLEFTTALONID);
        rightLifterTalon = new CANTalon(LIFTERRIGHTTALONID);
        lifterLeftPot = new AnalogInput(LIFTERLEFTPOTPORT);
        lifterRightPot = new AnalogInput(LIFTERRIGHTPOTPORT);
        lifterLeftPot.setAverageBits(2);
        lifterRightPot.setAverageBits(2);
        leftLifterServo = new Servo(SERVOLEFTPORT);
        rightLifterServo = new Servo(SERVORIGHTPORT);
   
        setSubsystemConstants();
      
        chassis = new Chassis(leftFrontTalon,
                leftRearTalon, rightFrontTalon,
                rightRearTalon);

        grabber = new Grabber(grabberVerticalTalon, grabberHorizontalTalon, 
        		grabberVerticalPot, grabberHorizontalPot, sonarSensor, 
        		m_grabberVerticalP, m_grabberVerticalI, m_grabberHorizontalP, 
        		m_grabberHorizontalI, m_grabberHorizontalGentleP, 
        		m_grabberHorizontalGentleI, m_horizontalThreshold, 
        		m_verticalThreshold, m_canExtendHeight, m_canGrabHeight, 
        		m_toteExtendHeight, m_toteGrabHeight, m_stepCanExtendHeight, 
        		m_stepCanGrabHeight, m_retractedLocation, 
                m_pauseDistanceFromObject, m_grabberHorizontalHome, 
                m_grabberVerticalHome, m_slowDownRetractThreshold);
        
        toteLifter = new ToteLifter(leftLifterTalon, rightLifterTalon, 
                leftLifterServo, rightLifterServo, lifterLeftPot, 
                lifterRightPot, m_lifterLeftP, m_lifterLeftI, m_lifterRightP, 
                m_lifterRightI, m_lifterAccuracyThreshold, m_lifterHomeHeight, 
                m_lifterWaitLiftPosition, m_lifterOpenDogsLiftPosition, 
                m_lifterCloseDogsLiftPosition, m_homeOffset);
        
        autonomous = new Autonomous(chassis, m_infraredDistanceTrigger, 
        		m_timeIgnore, m_timeDriveBack, m_timeDriveAway, 
        		m_moveBackSpeed);
    }

    public void setSubsystemConstants()
    {
        fileContents = FileReader.readFile("RobotConstants.txt");
        
        m_lifterLeftP = Double.parseDouble(fileContents.get(27));
        m_lifterLeftI = Double.parseDouble(fileContents.get(30));
        m_lifterRightP = Double.parseDouble(fileContents.get(33));
        m_lifterRightI = Double.parseDouble(fileContents.get(36));
        m_lifterAccuracyThreshold = 
                Double.parseDouble(fileContents.get(39));
        
        m_grabberHorizontalP = Double.parseDouble(fileContents.get(56));
        m_grabberHorizontalI = Double.parseDouble(fileContents.get(59));
        m_grabberVerticalP = Double.parseDouble(fileContents.get(50));
        m_grabberVerticalI = Double.parseDouble(fileContents.get(53));
        
        m_spinThreshold = Double.parseDouble(fileContents.get(22));
        m_infraredDistanceTrigger = 
                Double.parseDouble(fileContents.get(5));
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
        m_retractedLocation = Double.parseDouble(fileContents.get(80));
        m_horizontalThreshold = Double.parseDouble(fileContents.get(83));
        m_verticalThreshold = Double.parseDouble(fileContents.get(86));
        m_pauseDistanceFromObject = 
                Double.parseDouble(fileContents.get(89));
        m_grabberHorizontalGentleP = 
                Double.parseDouble(fileContents.get(92));
        m_grabberHorizontalGentleI = 
                Double.parseDouble(fileContents.get(95));
        m_grabberHorizontalHome = 
                Double.parseDouble(fileContents.get(98));
        m_grabberVerticalHome = Double.parseDouble(fileContents.get(101));
        
        m_lifterHomeHeight = Double.parseDouble(fileContents.get(104));
        m_lifterWaitLiftPosition = Double.parseDouble
                (fileContents.get(107));
        m_lifterOpenDogsLiftPosition = Double.parseDouble
                (fileContents.get(110));
        m_lifterCloseDogsLiftPosition = Double.parseDouble
                (fileContents.get(113));
        
        m_slowDownRetractThreshold = 
                Double.parseDouble(fileContents.get(116));
        m_homeOffset = Double.parseDouble(fileContents.get(119));
    }

    public void reinputConstants()
    {
        setSubsystemConstants();

        grabber.inputConstants(m_grabberVerticalP, m_grabberVerticalI, 
                m_grabberHorizontalP, m_grabberHorizontalI, 
                m_grabberHorizontalGentleP, m_grabberHorizontalGentleI, 
                m_horizontalThreshold, m_verticalThreshold, m_canExtendHeight, 
                m_canGrabHeight, m_toteExtendHeight, m_toteGrabHeight, 
                m_stepCanExtendHeight, m_stepCanGrabHeight, m_retractedLocation, 
                m_pauseDistanceFromObject, m_grabberHorizontalHome, 
                m_grabberVerticalHome, m_slowDownRetractThreshold);
        toteLifter.inputConstants(m_lifterLeftP, m_lifterLeftI,
                m_lifterRightP, m_lifterRightI, m_lifterAccuracyThreshold, 
                m_lifterHomeHeight, m_lifterWaitLiftPosition, 
                m_lifterOpenDogsLiftPosition, m_lifterCloseDogsLiftPosition, 
                m_homeOffset);
        autonomous.inputConstants(m_infraredDistanceTrigger, m_timeIgnore, 
                m_timeDriveBack, m_timeDriveAway, m_moveBackSpeed);
    }

    public void autonomousInit()
    {        
        reinputConstants();
        //System.out.println(toteLifter.m_homeHeight);
        //toteLifter.zeroPots();
        leftLifterServo.setAngle(0);
        rightLifterServo.setAngle(180);
        //toteLifter.reinitPIControllers();
        //toteLifter.zeroPots();
        grabber.zeroPots();
        grabber.grabStepCan();
        
        
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic()
    {
        //toteLifter.mapPots();
        //System.out.print("Left: " + toteLifter.m_leftHeight);
        //System.out.println(" Right: " + toteLifter.m_rightHeight);
        chassis.setJoystickData(0, 0, 0);
        chassis.idle();
        grabber.idle();
        //System.out.println(grabber.m_canHorizontalState);
        System.out.println(grabber.horizontalTalon.getOutputCurrent());
//        grabber.goToLength(0.689);
        
//        System.out.println(toteLifter.goToHeight(0.4));
    }

    /**
     * 
     */
    public void teleopInit()
    {
        reinputConstants();
        grabber.reset();
        System.out.println(grabber.m_horizontalP);
        System.out.println(grabber.m_horizontalI);
        toteLifter.disable();
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic()
    {
//        toteLifter.mapPots();
//        System.out.print("Left: " + toteLifter.m_leftHeight);
//        System.out.println(" Right: " + toteLifter.m_rightHeight);
          if(joystickZero.getRawButton(1))
          {
              toteLifter.dropTotes();
          }
          if(joystickZero.getRawButton(2))
          {
              toteLifter.addTote();
          }
//        if(joystickZero.getRawButton(3))
//        {
//            toteLifter.dropFullTotes();
//        }
//        if(joystickZero.getRawButton(4))
//        {
//            toteLifter.dropAllTotes();
//        }
        if(joystickZero.getRawButton(3))
        {
            grabber.grabTote();
        }
        if(joystickZero.getRawButton(4))
        {
            grabber.grabCan();
        }
        if(joystickZero.getRawButton(5))
        {
            grabber.grabStepCan();
        }
        if(joystickZero.getRawButton(10))
        {
            toteLifter.approachHome();
        }
        if(joystickZero.getRawButton(11))
        {
            grabber.zeroPots();
        }
        if(joystickZero.getRawButton(12))
        {
            grabber.goHome();
        }
        System.out.println(toteLifter.leftTalon.getOutputCurrent());
        //System.out.println(grabber.horizontalTalon.getOutputCurrent());
        toteLifter.idle();
        //System.out.println("Tote = "+ grabber.m_toteHorizontalState );
        //System.out.println("                                     Can = "+grabber.m_canHorizontalState);
        //System.out.println("                                                                         Vertical = "+grabber.m_verticalState);
        grabber.idle();
//        System.out.print("m_pauseDistanceFromObject: " + grabber.m_pauseDistanceFromObject);
//        System.out.print(" m_horizontalPotDistance: " + grabber.m_horizontalPotDistance);
//        System.out.println(" m_horizontalThreshold: " + grabber.m_horizontalThreshold);
//        chassis.setJoystickData(0, 0, 0);
//        chassis.idle();

//        System.out.print("LeftAmps: " + leftLifterTalon.getOutputCurrent());
//        System.out.println
//                (" RightAmps: " + rightLifterTalon.getOutputCurrent());
//        System.out.print(" AddSubstate: " + toteLifter.m_addSubstate);
//        System.out.println(toteLifter.m_dropFullState);
//        System.out.print(" Left: " + toteLifter.m_leftHeight);
//        System.out.println(" Right: " + toteLifter.m_rightHeight);
        
//        System.out.print("horizontalState: " + grabber.m_canHorizontalState);
//        System.out.print(" verticalState: " + grabber.m_verticalState);
//        System.out.print(" Horizontal: " + grabber.m_horizontalPotDistance);
//        System.out.print(" Vertical: " + grabber.m_verticalPotDistance);
//        System.out.println(" Sonar: " + grabber.m_sonarDistance);
        
    	double x = joystickZero.getX();
		double y = joystickZero.getY();
		double twist = joystickZero.getTwist();
		
		if(twist < 0)
        {
            if(twist > -0.25)
            {
                twist = 0;
            }
            else
            {
                twist += 0.25;
                twist = twist*(1/.75);
            }
        }
		
        /* Maps the value of the joystick using the current position of the
           throttle slider for safety and ease of driver control. */
		double throttleMapping = Math.abs((joystickZero.getThrottle() - 1));
		chassis.setJoystickData(y * throttleMapping, x * throttleMapping, 
		        twist * throttleMapping);
		chassis.idle();
    }
    
    public void disabledPeriodic()
    {
//        grabber.mapSensors();
//        System.out.print("Horizontal: " + grabber.m_horizontalPotDistance);
//        System.out.print(" Vertical: " + grabber.m_verticalPotDistance);
//        System.out.println(" Sonar: " + grabber.m_sonarDistance);
    }

    public void testInit()
    {
        reinputConstants();
        leftLifterServo.setAngle(0);
        rightLifterServo.setAngle(180);
    }

    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic()
    {
        grabberVerticalTalon.set(-joystickTwo.getY());
        grabberHorizontalTalon.set(joystickTwo.getX());
        System.out.println("leftCurrent : "+leftLifterTalon.getOutputCurrent() +"rightCurrent : "+rightLifterTalon.getOutputCurrent());
        double twist = joystickZero.getTwist();
        if(twist < 0)
        {
            if(twist > -0.35)
            {
                twist = 0;
            }
            else
            {
                twist += 0.35;
                twist = twist*(1/.65);
            }
        }
        //System.out.println(twist);
        leftLifterTalon.set(-joystickZero.getY());
        rightLifterTalon.set(joystickOne.getY());        
        if(joystickZero.getRawButton(1))
        {
            leftLifterServo.setAngle(0);
            rightLifterServo.setAngle(180);
        }
        if(joystickZero.getRawButton(2))
        {
            leftLifterServo.setAngle(170);
            rightLifterServo.setAngle(10);
        }
    }
}
