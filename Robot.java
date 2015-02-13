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
    CANTalon leftFrontMotorController, rightFrontMotorController, 
            leftRearMotorController, rightRearMotorController;
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
    Joystick joystickZero, joystickOne;
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
		final int SERVOLEFTPORT = 0;
		final int SERVORIGHTPORT = 1;
		
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
        
        double grabberHorizontalI = Double.parseDouble(fileContents.get(59));
        double grabberHorizontalP = Double.parseDouble(fileContents.get(56));
        double grabberVerticalI = Double.parseDouble(fileContents.get(53));
        double grabberVerticalP = Double.parseDouble(fileContents.get(50));
        
        double accuracyThreshold = Double.parseDouble(fileContents.get(39));
        double openServoPosition = Double.parseDouble(fileContents.get(45));
        double closeServoPosition = Double.parseDouble(fileContents.get(42));
        
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

        leftFrontMotorController = new CANTalon(CHASSISLEFTFRONTTALONID);
        rightFrontMotorController = new CANTalon(CHASSISRIGHTFRONTTALONID);
        leftRearMotorController = new CANTalon(CHASSISLEFTREARTALONID);
        rightRearMotorController = new CANTalon(CHASSISRIGHTREARTALONID);
        
        grabberVerticalTalon = new CANTalon(GRABBERVERTICALTALONID);
        grabberHorizontalTalon = new CANTalon(GRABBERHORIZONTALTALONID);
        grabberVerticalPot = new AnalogInput(GRABBERVERTICALPOTPORT);
        grabberHorizontalPot = new AnalogInput(GRABBERHORIZONTALPOTPORT);
        
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
        leftLifterServo = new Servo(SERVOLEFTPORT);
        rightLifterServo = new Servo(SERVORIGHTPORT);
        
        chassis = new Chassis(leftFrontMotorController,
                leftRearMotorController, rightFrontMotorController,
                rightRearMotorController);

        grabber = new Grabber(grabberVerticalTalon, grabberHorizontalTalon, 
        		grabberVerticalPot, grabberHorizontalPot, sonarSensor, 
        		grabberVerticalP, grabberVerticalI,  grabberHorizontalP, 
        		grabberHorizontalI, horizontalThreshold, verticalThreshold, 
                canExtendHeight, canGrabHeight, toteExtendHeight, 
                toteGrabHeight, stepCanExtendHeight, stepCanGrabHeight, 
                retractedLocation, pauseDistanceFromObject);
        
        toteLifter = new ToteLifter(leftLifterTalon, rightLifterTalon, 
                leftLifterServo, rightLifterServo, lifterLeftPot, 
                lifterRightPot, piControllerLifterLeft, piControllerLifterRight, 
        		accuracyThreshold, openServoPosition, closeServoPosition);
        
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
        
        grabber.inputPIConstants(grabberVerticalP, grabberVerticalI, 
        		grabberHorizontalP, grabberHorizontalI);
        System.out.print(grabberHorizontalP);
        System.out.println(" " + grabberHorizontalI);
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic()
    {
        chassis.setJoystickData(0, 0, 0);
        chassis.idle();

        grabber.goToLength(0.689);
    }

    /**
     * 
     */
    public void teleopInit()
    {
        grabber.reset();
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic()
    {
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
        grabber.idle();
        chassis.setJoystickData(0, 0, 0);
        chassis.idle();
        System.out.print("Vertical: " + (-0.2608156852 * 
                grabberVerticalPot.getVoltage() + 1.421847684));
        System.out.print(" Horizontal: " + (-0.4364133427 * 
                grabberHorizontalPot.getVoltage() + 1.78356027));
        System.out.println(" Sonar: " + (2.2121617347 * sonarSensor.getVoltage() 
                + 0.0467578232));
//        grabberVerticalTalon.set(-joystickZero.getY());
//        grabberHorizontalTalon.set(joystickOne.getY());
//        System.out.println(sonarSensor.getVoltage());
//        rightLifterTalon.set(joystickOne.getY());
//        leftLifterTalon.set(joystickZero.getY());
        
//       grabberVerticalTalon.set(joystick.getY());
//        chassis.setJoystickData(0, 0, 0);
//        chassis.idle();
//        double verticalPotVolt;
//        double horizontalPotVolt;
//        verticalPotVolt = grabberVerticalPot.getVoltage();
//        horizontalPotVolt = grabberHorizontalPot.getVoltage();
//        System.out.println(horizontalPotVolt);
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
//        double verticalPotVolt;
//        double horizontalPotVolt;
//        verticalPotVolt = grabberVerticalPot.getVoltage();
//        horizontalPotVolt = grabberHorizontalPot.getVoltage();
        
//        System.out.println(sonarSensor.getVoltage());
        
        //System.out.println(
        //        "Vert: "+verticalPotVolt+"  Hori: "+horizontalPotVolt);
        //System.out.println(-0.2608156852*verticalPotVolt+1.421847684);
//        System.out.println(horizontalPotVolt);
    }

    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic()
    {
        grabberVerticalTalon.set(-joystickZero.getY());
        grabberHorizontalTalon.set(joystickOne.getY());
        System.out.print("Vertical: " + (-0.2608156852 * 
                grabberVerticalPot.getVoltage() + 1.421847684));
        System.out.print(" Horizontal: " + (-0.4364133427 * 
                grabberHorizontalPot.getVoltage() + 1.78356027));
        System.out.println(" Sonar: " + (2.2121617347 * sonarSensor.getVoltage() 
                + 0.0467578232));
//        leftLifterTalon.set(joystickZero.getY());
//        rightLifterTalon.set(-joystickOne.getY());
//        System.out.print("Left: " + leftLifterTalon.getOutputCurrent());
//        System.out.println(" Right: " + rightLifterTalon.getOutputCurrent());
//        rightLifterTalon.getOutputCurrent();
//        if(joystickOne.getRawButton(1))
//        {
//            leftLifterServo.setAngle(0);
//            rightLifterServo.setAngle(180);
//        }
//        if(joystickOne.getRawButton(2))
//        {
//            leftLifterServo.setAngle(170);
//            rightLifterServo.setAngle(10);
//        }
    }
}
