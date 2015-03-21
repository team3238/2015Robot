package org.usfirst.frc.team3238.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
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
    DriverStation driverStation;
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
    
    double m_gyroPConstant;
    double m_gyroIConstant;

    int m_topHatValue;
    int m_previousTopHatValue;
    double m_previousJoystickOneYValue;
    double m_joystickOneYValue;
    
    long teleopStartTimestamp;
    
    boolean m_grabberHasHit;
    long m_nubbinTimePlaceholder;
    double m_previousDistance;
    
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
    				
		driverStation = DriverStation.getInstance();
		
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
        gyroSensor.setAverageBits(2);
        infraredSensor = new AnalogInput(IRSENSORPORT);
        infraredSensor.setAverageBits(2);
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
                m_grabberVerticalHome, m_slowDownRetractThreshold, 
                infraredSensor);
        
        toteLifter = new ToteLifter(leftLifterTalon, rightLifterTalon, 
                leftLifterServo, rightLifterServo, lifterLeftPot, 
                lifterRightPot, m_lifterLeftP, m_lifterLeftI, m_lifterRightP, 
                m_lifterRightI, m_lifterAccuracyThreshold, m_lifterHomeHeight, 
                m_lifterWaitLiftPosition, m_lifterOpenDogsLiftPosition, 
                m_lifterCloseDogsLiftPosition, m_homeOffset, infraredSensor);
        
        autonomous = new Autonomous(chassis);
        m_joystickOneYValue = 0;
        m_grabberHasHit = false;
        m_nubbinTimePlaceholder = 0;
        m_previousDistance = 0.0;
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
        
        m_gyroPConstant = Double.parseDouble(fileContents.get(122));
        m_gyroIConstant = Double.parseDouble(fileContents.get(125));
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
        //autonomous.inputConstants(m_infraredDistanceTrigger, m_timeIgnore, 
        //        m_timeDriveBack, m_timeDriveAway, m_moveBackSpeed);
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
        //grabber.zeroPots();
        //grabber.grabStepCan();
        grabber.reset();
        autonomous.init(toteLifter, accelerometer);
        grabber.zeroPots();
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic()
    {
        //toteLifter.mapPots();
        //System.out.print("Left: " + toteLifter.m_leftHeight);
        //System.out.println(" Right: " + toteLifter.m_rightHeight);
        //chassis.setJoystickData(0, 0, 0);
        autonomous.idle(chassis, gyroSensor.getAverageVoltage(), 
                m_spinThreshold, toteLifter, grabber, m_gyroPConstant, 
                m_gyroIConstant);
        chassis.idle();
        grabber.idle();
        //toteLifter.idle();
        //System.out.println(grabber.horizontalTalon.getOutputCurrent());
        //grabber.idle();
        //System.out.println(grabber.m_canHorizontalState);
        //System.out.println(grabber.horizontalTalon.getOutputCurrent());
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
        GyroDrive.reinit();
        teleopStartTimestamp = System.currentTimeMillis();
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic()
    {
        //      toteLifter.mapPots();
        toteLifter.mapDistance();
        //System.out.println(toteLifter.m_infraredDistance);
        //      System.out.print("Left: " + toteLifter.m_leftHeight);
        //      System.out.println(" Right: " + toteLifter.m_rightHeight);
        m_previousTopHatValue = m_topHatValue;
        m_topHatValue = joystickZero.getPOV();
        if(joystickZero.getRawButton(1))
        {
            toteLifter.m_goAllTheWayDownOnDrop = true;
            toteLifter.dropTotes();
        }
        if(joystickZero.getRawButton(2))
        {
            toteLifter.addTote();
        }
        if(joystickZero.getRawButton(7))
        {
            toteLifter.closeDogs();
        }
        if(joystickZero.getRawButton(8))
        {
            toteLifter.openDogs();
        }
        if(joystickZero.getRawButton(5))
        {
            toteLifter.dropFullTotes();
        }
        if(joystickZero.getRawButton(9))
        {
            grabber.goToAvoidTotePosition();
        }
        if(joystickZero.getRawButton(3))
        {
            grabber.grabTote();
        }
        if(joystickZero.getRawButton(4))
        {
            grabber.grabCan();
        }
        if(joystickZero.getRawButton(6))
        {
           //grabber.grabStepCan();
            grabber.grabStepCanAuto();
            grabber.m_stepCanDirection = -1.0;
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
        if(joystickOne.getRawButton(11))
        {
            toteLifter.goToHomeOnly();
        }
        if(joystickOne.getRawButton(12))
        {
            toteLifter.m_goAllTheWayDownOnDrop = false;
            toteLifter.dropTotes();
        }
        //System.out.println("grabber arm");
        //System.out.println(0.2680762026*Math.pow(infraredSensor.getAverageVoltage(),
        //        -1.130124285));
        ///System.out.println(toteLifter.leftTalon.getOutputCurrent());
        //System.out.println(toteLifter.leftTalon.getOutputCurrent());
        //System.out.println(grabber.horizontalTalon.getOutputCurrent());
        //System.out.println(grabber.m_horizontalPotDistance);
        //System.out.println("Tote = "+ grabber.m_toteHorizontalState );
        //System.out.println("                                     Can = "+grabber.m_canHorizontalState);
        //System.out.println("                                                                         Vertical = "+grabber.m_verticalState);
        toteLifter.m_manuelControl = false;
        if(m_topHatValue == -1)
        {
            m_grabberHasHit = false;
            m_nubbinTimePlaceholder = System.currentTimeMillis();
            if(m_previousTopHatValue != -1)
            {
                grabber.horizontalTalon.set(0);
            }
            else
            {
                m_previousJoystickOneYValue = m_joystickOneYValue;
                m_joystickOneYValue = joystickOne.getY();
                
                if(Math.abs(m_joystickOneYValue) >= 0.2)
                {
                    if(joystickOne.getThrottle() >= 0)
                    {
                        toteLifter.disable();
                        toteLifter.m_manuelControl = true;
                        toteLifter.m_manuelPosition += m_joystickOneYValue/100;
                        grabber.horizontalTalon.set(0);
                    }
                    else
                    {
                        grabber.horizontalTalon.set(m_joystickOneYValue);
                        toteLifter.leftTalon.set(0);
                        toteLifter.rightTalon.set(0);
                    }
                }
                else
                {
                    if(Math.abs(m_previousJoystickOneYValue) >= 0.2)
                    {
                        grabber.horizontalTalon.set(0);
                    }
                    grabber.idle();
                }
            }
        }
        else
        {
        	grabber.reset();
        	//grabber.horizontalTalon.set(0);
        	//grabber.verticalTalon.set(0);
        	//System.out.println(m_topHatValue);
        	//if(Math.abs(grabber.m_horizontalPotDistance - m_previousDistance) < 0.002 &&
        	//        System.currentTimeMillis() - m_nubbinTimePlaceholder > 100)
        	double coefficient = 1.0;
        	m_previousDistance = grabber.m_horizontalPotDistance;
        	grabber.mapSensors();
        	if(grabber.m_horizontalPotDistance < 1.0)
        	{
        	    coefficient = 0.7;
        	    if(System.currentTimeMillis() - m_nubbinTimePlaceholder > 100)
                {
        	        if(grabber.horizontalTalon.getOutputCurrent() > 14)
        	        {
        	            m_grabberHasHit = true;
        	        }
                }
        	}
        	if(m_grabberHasHit == true)
        	{
        	    coefficient = 0.0;
        	}
        	grabber.verticalTalon.set(1*Math.sin((m_topHatValue+90)*(Math.PI/180.0)));
        	grabber.horizontalTalon.set(coefficient*Math.cos((m_topHatValue+270)*(Math.PI/180.0)));
        	
        }
        
    	double x = joystickZero.getX();
		double y = joystickZero.getY();
		double twist = joystickZero.getTwist();
		
		double deadzone = 0.15;
		
        if(twist < 0)
        {
            if (twist > -deadzone)
            {
                twist = 0;
            }
            else
            {
                twist += deadzone;
                twist = twist*(1/(1-deadzone));
            }
        }
        else
        {
            if (twist < deadzone)
            {
                twist = 0;
            }
            else
            {
                twist -= deadzone;
                twist = twist*(1/(1-deadzone));
            }
        }
            
        /* Maps the value of the joystick using the current position of the
           throttle slider for safety and ease of driver control. */
		double throttleMapping = Math.abs((joystickZero.getThrottle() - 1));
		double adjustedRotation = GyroDrive.getAdjustedRotationValue(
		        x * throttleMapping, y * throttleMapping,
		        twist * throttleMapping, m_gyroPConstant, m_gyroIConstant, 
		        m_spinThreshold, gyroSensor.getAverageVoltage() - 2.37);
		chassis.setJoystickData(x * throttleMapping, y * throttleMapping, 
		        adjustedRotation);
		chassis.idle();
		if(driverStation.isFMSAttached() && 
				(System.currentTimeMillis() - teleopStartTimestamp > 133000))
		{
			toteLifter.goToHeight(toteLifter.m_waitLiftPosition);
		}
		toteLifter.idle();
		System.out.println("Left height:" + toteLifter.m_leftHeight + "           Right height:" + toteLifter.m_leftHeight + "            Difference = " + (toteLifter.m_leftHeight-toteLifter.m_rightHeight));
    }
    
    public void disabledPeriodic()
    {
//        grabber.mapSensors();
//        System.out.print("Horizontal: " + grabber.m_horizontalPotDistance);
//        System.out.print(" Vertical: " + grabber.m_verticalPotDistance);
//        System.out.println(" Sonar: " + grabber.m_sonarDistance);
         // System.out.println("Y = " + Double.toString(accelerometer.getY()).substring(0, 7)  + 
          //        "X = " + Double.toString(accelerometer.getX()).substring(0, 7) +
          //        "Z = " + Double.toString(accelerometer.getZ()).substring(0, 7));
        System.out.println("Y = " + Double.toString(accelerometer.getY()));
        System.out.println("                   X = " + Double.toString(accelerometer.getX()));
        System.out.println("                                     Z = " + Double.toString(accelerometer.getZ()));
        /*
        if (X < .01)
        {
            not on scoring platfrom
        }
        else
        {
            on scoring platform
        }*/
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
        //System.out.println(toteLifter.leftPot.getAverageVoltage());
        //System.out.println("Throttle: "+joystickZero.getThrottle());
    }
}
