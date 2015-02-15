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
		final int LIFTERLEFTTALONID = 7;
		final int LIFTERRIGHTTALONID = 5;
		
		// Digital Inputs
		final int REFLECTSENSORFRONTPORT = 5;
		final int REFLECTSENSORREARPORT = 6;
		
		// Analog Inputs
		final int IRSENSORPORT = 2;
		final int SONARSENSORPORT = 0;
		final int GYROSENSORPORT = 1;
		
		final int GRABBERVERTICALPOTPORT = 4;
		final int GRABBERHORIZONTALPOTPORT = 5;
		
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
    			
        fileContents = FileReader.readFile("RobotConstants.txt");
        
        double lifterLeftP = Double.parseDouble(fileContents.get(27));
        double lifterLeftI = Double.parseDouble(fileContents.get(30));
        double lifterRightP = Double.parseDouble(fileContents.get(33));
        double lifterRightI = Double.parseDouble(fileContents.get(36));
        
        double grabberHorizontalP = Double.parseDouble(fileContents.get(56));
        double grabberHorizontalI = Double.parseDouble(fileContents.get(59));
        double grabberVerticalP = Double.parseDouble(fileContents.get(50));
        double grabberVerticalI = Double.parseDouble(fileContents.get(53));
        
        double accuracyThreshold = Double.parseDouble(fileContents.get(39));
        
        double spinThreshold = Double.parseDouble(fileContents.get(22));
        double infraredDistanceTrigger = 
                Double.parseDouble(fileContents.get(5));
        long timeIgnore = Long.parseLong(fileContents.get(8));
        int timeDriveBack = Integer.parseInt(fileContents.get(11));
        int timeDriveAway = Integer.parseInt(fileContents.get(14));
        double moveBackSpeed = Double.parseDouble(fileContents.get(17));

        double canExtendHeight = Double.parseDouble(fileContents.get(62));
        double canGrabHeight = Double.parseDouble(fileContents.get(65));
        double toteExtendHeight = Double.parseDouble(fileContents.get(68));
        double toteGrabHeight = Double.parseDouble(fileContents.get(71));
        double stepCanExtendHeight = Double.parseDouble(fileContents.get(74)); 
        double stepCanGrabHeight = Double.parseDouble(fileContents.get(77));
        double retractedLocation = Double.parseDouble(fileContents.get(80));
        double horizontalThreshold = Double.parseDouble(fileContents.get(83));
        double verticalThreshold = Double.parseDouble(fileContents.get(86));
        double pauseDistanceFromObject = 
                Double.parseDouble(fileContents.get(89));
        double grabberHorizontalGentleP = 
                Double.parseDouble(fileContents.get(92));
        double grabberHorizontalGentleI = 
                Double.parseDouble(fileContents.get(95));
        double grabberHorizontalHome = Double.parseDouble(fileContents.get(98));
        double grabberVerticalHome = Double.parseDouble(fileContents.get(101));
        
        double lifterHomeHeight = Double.parseDouble(fileContents.get(104));
        double lifterWaitLiftPosition = Double.parseDouble
                (fileContents.get(107));
        double lifterOpenDogsLiftPosition = Double.parseDouble
                (fileContents.get(110));
        double lifterCloseDogsLiftPosition = Double.parseDouble
                (fileContents.get(113));
        
        double slowDownRetractThreshold = 
                Double.parseDouble(fileContents.get(116));
        
        leftFrontTalon = new CANTalon(CHASSISLEFTFRONTTALONID);
        rightFrontTalon = new CANTalon(CHASSISRIGHTFRONTTALONID);
        leftRearTalon = new CANTalon(CHASSISLEFTREARTALONID);
        rightRearTalon = new CANTalon(CHASSISRIGHTREARTALONID);
        
        grabberVerticalTalon = new CANTalon(GRABBERVERTICALTALONID);
        grabberHorizontalTalon = new CANTalon(GRABBERHORIZONTALTALONID);
        grabberVerticalPot = new AnalogInput(GRABBERVERTICALPOTPORT);
        grabberHorizontalPot = new AnalogInput(GRABBERHORIZONTALPOTPORT);
        grabberVerticalPot.setAverageBits(2);
        grabberHorizontalPot.setAverageBits(2);
        
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
        
        chassis = new Chassis(leftFrontTalon,
                leftRearTalon, rightFrontTalon,
                rightRearTalon);

        grabber = new Grabber(grabberVerticalTalon, grabberHorizontalTalon, 
        		grabberVerticalPot, grabberHorizontalPot, sonarSensor, 
        		grabberVerticalP, grabberVerticalI,  grabberHorizontalP, 
        		grabberHorizontalI, grabberHorizontalGentleP, 
        		grabberHorizontalGentleI, horizontalThreshold, 
        		verticalThreshold, canExtendHeight, canGrabHeight, 
        		toteExtendHeight, toteGrabHeight, stepCanExtendHeight, 
        		stepCanGrabHeight, retractedLocation, pauseDistanceFromObject, 
        		grabberHorizontalHome, grabberVerticalHome,
        		slowDownRetractThreshold);
        
        toteLifter = new ToteLifter(leftLifterTalon, rightLifterTalon, 
                leftLifterServo, rightLifterServo, lifterLeftPot, 
                lifterRightPot, lifterLeftP, lifterLeftI, lifterRightP, 
                lifterRightI, accuracyThreshold, lifterHomeHeight, 
                lifterWaitLiftPosition, lifterOpenDogsLiftPosition, 
                lifterCloseDogsLiftPosition);
        
        autonomous = new Autonomous(chassis, infraredDistanceTrigger, 
        		timeIgnore, timeDriveBack, timeDriveAway, 
        		moveBackSpeed);
    }

    /**
     * 
     */
    public void autonomousInit()
    {        
        fileContents = FileReader.readFile("RobotConstants.txt");
        double grabberHorizontalI = Double.parseDouble(fileContents.get(59));
        double grabberHorizontalP = Double.parseDouble(fileContents.get(56));
        double grabberVerticalI = Double.parseDouble(fileContents.get(53));
        double grabberVerticalP = Double.parseDouble(fileContents.get(50));
        double grabberHorizontalGentleP = 
                Double.parseDouble(fileContents.get(92));
        double grabberHorizontalGentleI = 
                Double.parseDouble(fileContents.get(95));
        grabber.inputPIConstants(grabberVerticalP, grabberVerticalI, 
                grabberHorizontalP, grabberHorizontalI, 
                grabberHorizontalGentleP, grabberHorizontalGentleI);
        
        double lifterLeftP = Double.parseDouble(fileContents.get(27));
        double lifterLeftI = Double.parseDouble(fileContents.get(30));
        double lifterRightP = Double.parseDouble(fileContents.get(33));
        double lifterRightI = Double.parseDouble(fileContents.get(36));
        toteLifter.inputPIConstants(lifterLeftP, lifterLeftI, lifterRightP, 
                lifterRightI);
//        System.out.print(grabberHorizontalP + " " + grabberHorizontalI + " ");
//        System.out.println
//                (grabberHorizontalGentleP + " " + grabberHorizontalGentleI);
        
        toteLifter.zeroPots();
        leftLifterServo.setAngle(0);
        rightLifterServo.setAngle(180);
        toteLifter.reinitPIControllers();
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic()
    {
        chassis.setJoystickData(0, 0, 0);
        chassis.idle();

//        grabber.goToLength(0.689);
        
        System.out.println(toteLifter.goToHeight(0.4));
    }

    /**
     * 
     */
    public void teleopInit()
    {
        grabber.reset();
        fileContents = FileReader.readFile("RobotConstants.txt");
        double grabberHorizontalI = Double.parseDouble(fileContents.get(59));
        double grabberHorizontalP = Double.parseDouble(fileContents.get(56));
        double grabberVerticalI = Double.parseDouble(fileContents.get(53));
        double grabberVerticalP = Double.parseDouble(fileContents.get(50));
        double grabberHorizontalGentleP = 
                Double.parseDouble(fileContents.get(92));
        double grabberHorizontalGentleI = 
                Double.parseDouble(fileContents.get(95));
        
        grabber.inputPIConstants(grabberVerticalP, grabberVerticalI, 
                grabberHorizontalP, grabberHorizontalI, 
                grabberHorizontalGentleP, grabberHorizontalGentleI);
        System.out.print(grabberHorizontalP + " " + grabberHorizontalI + " ");
        System.out.println
                (grabberHorizontalGentleP + " " + grabberHorizontalGentleI);
        //grabber.zeroPots();
        toteLifter.reinit();
        toteLifter.zeroPots();
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic()
    {
        if(joystickZero.getRawButton(1))
        {
            toteLifter.dropTotes();
        }
        if(joystickZero.getRawButton(2))
        {
            toteLifter.addTote();
        }
        if(joystickZero.getRawButton(3))
        {
            toteLifter.dropFullTotes();
        }
        if(joystickZero.getRawButton(4))
        {
            toteLifter.dropAllTotes();
        }
//        if(joystickZero.getRawButton(3))
//        {
//            grabber.grabTote();
//        }
//        if(joystickZero.getRawButton(4))
//        {
//            grabber.grabCan();
//        }
//        if(joystickZero.getRawButton(5))
//        {
//            grabber.grabStepCan();
//        }
//        if(joystickZero.getRawButton(11))
//        {
//            grabber.zeroPots();
//        }
//        if(joystickZero.getRawButton(12))
//        {
//            grabber.goHome();
//        }
        toteLifter.idle();
        grabber.idle();
        chassis.setJoystickData(0, 0, 0);
        chassis.idle();
        //System.out.print("Left: " + leftLifterTalon.getOutputCurrent());
        //System.out.println(" Right: " + rightLifterTalon.getOutputCurrent());
//        System.out.print(toteLifter.m_addSubstate);
        System.out.println(toteLifter.m_dropFullState);
//        System.out.print(" Left: " + toteLifter.m_leftHeight);
//        System.out.println(" Right: " + toteLifter.m_rightHeight);
        
        //System.out.print("horizontalState: " + grabber.m_canHorizontalState);
        //System.out.print(" verticalState: " + grabber.m_verticalState);
//        System.out.print(" Horizontal: " + grabber.m_horizontalPotDistance);
        //System.out.print(" Vertical: " + grabber.m_verticalPotDistance);
        //System.out.println(" Sonar: " + grabber.m_sonarDistance);
        
//    	double x = joystickZero.getX();
//		double y = joystickZero.getY();
//		double twist = joystickZero.getTwist();
//		double throttleMapping = Math.abs((joystickZero.getThrottle() - 1));
//		chassis.setJoystickData(y * throttleMapping, x * throttleMapping, 
//		        twist * throttleMapping);
//		chassis.idle();
    }
    
    public void disabledPeriodic()
    {
//        grabber.mapSensors();
//        System.out.print("Horizontal: " + grabber.m_horizontalPotDistance);
//        System.out.print(" Vertical: " + grabber.m_verticalPotDistance);
//        System.out.println(" Sonar: " + grabber.m_sonarDistance);
    }

    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic()
    {
//        grabberVerticalTalon.set(-joystickTwo.getY());
//        grabberHorizontalTalon.set(joystickTwo.getX());
        
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
        toteLifter.mapPots();
        System.out.println(toteLifter.m_rightHeight);
    }
}
