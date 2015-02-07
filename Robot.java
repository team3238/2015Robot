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
    ToteLifter toteLifter;
    Grabber grabber;
    ArrayList<String> m_fileContents;

    double m_spinThreshold;
    double m_infraredDistance;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit()
    {
        m_fileContents = FileReader.readFile("RobotConstants.txt");
        m_spinThreshold = Double.parseDouble(m_fileContents.get(2));
        piControllerLifterLeft = new PIController(
                Double.parseDouble(m_fileContents.get(2)),
                Double.parseDouble(m_fileContents.get(2)));
        piControllerLifterRight = new PIController(
                Double.parseDouble(m_fileContents.get(2)),
                Double.parseDouble(m_fileContents.get(2)));

        leftFrontMotorController = new CANTalon(2);
        rightFrontMotorController = new CANTalon(4);
        leftRearMotorController = new CANTalon(6);
        rightRearMotorController = new CANTalon(8);

        chassis = new Chassis(leftFrontMotorController,
                rightFrontMotorController, leftRearMotorController,
                rightRearMotorController);

        grabber = new Grabber(1, 2, 3, 4, 5, 6, 7, 8, 9);
        autonomous = new Autonomous(chassis);
        reflectSensorFront = new DigitalInput(13);
        reflectSensorRear = new DigitalInput(14);
        gyroSensor = new AnalogInput(1);
        accelerometer = new BuiltInAccelerometer();
        toteLifter = new ToteLifter(1, 2, 3, 4, piControllerLifterLeft,
                piControllerLifterRight);
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
        autonomous.idle(chassis, reflectSensorRear, reflectSensorFront,
                gyroSensor.getValue(), m_spinThreshold, accelerometer,
                m_infraredDistance, toteLifter, grabber,
                piControllerLifterLeft, piControllerLifterRight);
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
