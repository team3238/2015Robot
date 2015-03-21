package org.usfirst.frc.team3238.robot;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;


/**
 *
 * @author Anders Sjoboen
 */
public class Autonomous
{
    int m_autoState;
    int m_collectCount;
    double m_infraredDistanceTrigger;
    long m_timeStamp;
    long m_timeIgnore;
    int m_timeDriveBack;
    int m_timeDriveAway;
    double m_moveBackSpeed;
    int m_moveForwardTime;

    public static class AutoState
    {
        public static final int driveOffRamp = 0, pickUpCan = 1,
                translateRight = 2, waitToTranslate = 3, done = 4,
                goBack = 5;
    }

    Autonomous(Chassis chassis)
    {
        
    }
    
    /**
     * Re-initializes the autonomous period so that the variables are set up to
     * be at the beginning of the autonomous
     * 
     */
    void init(ToteLifter toteLifter, BuiltInAccelerometer accelerometer)
    {
        m_timeStamp = System.currentTimeMillis();
        m_autoState = AutoState.driveOffRamp;
        //toteLifter.approachHome();
        if (accelerometer.getX() < .01)
        {
            m_moveForwardTime = 1500;
        }
        else
        {
            m_moveForwardTime = 500;
        }
    }

    /**
     * Handles all of the cases for the autonomous. This must be called every
     * loop for the autonomous to operate
     * 
     * @param chassis
     *            Chassis object to move the robot
     * @param reflectSensorRear
     *            The reflectivity sensor in the back of the robot
     * @param reflectSensorFront
     *            The reflectivity sensor in the front of the back of the robot
     * @param gyroValue
     *            The value for how fast the robot is turning
     * @param spinThreshold
     *            Value for the threshold for how fast you need to be going for
     *            the corrector to not correct anymore
     * @param accelerometer
     *            The accelerometer object built in to the roboRIO
     * @param infraredDistance
     *            The distance that the infra sensor is reading
     */
    void idle(Chassis chassis, double gyroValue,
            double spinThreshold, ToteLifter toteLifter, Grabber grabber, 
            double gyroPConstant, double gyroIConstant)
    {
        
        switch(m_autoState)
        {
            case (AutoState.driveOffRamp):
                if(System.currentTimeMillis() - m_timeStamp < m_moveForwardTime)
                {
                    chassis.setJoystickData(0, -0.5, 0);
                }
                else
                {
                    chassis.setJoystickData(0, 0, 0);
                    m_autoState = AutoState.pickUpCan;
                }
                break;
                
            case (AutoState.pickUpCan):
                grabber.grabStepCanAuto();
                grabber.m_stepCanDirection = 1.0;
                
                //OPTIONAL 
                //m_autoState = AutoState.done;
                
                //REGULAR
                m_autoState = AutoState.waitToTranslate;
                break;
                
            case (AutoState.waitToTranslate):
                chassis.setJoystickData(0, 0, 0);
                if(grabber.m_finishedAutoGrab)
                {
                    System.out.println("*****************************************");
                    m_autoState = AutoState.goBack;
                    m_timeStamp = System.currentTimeMillis();
                }
                break;
                
            case (AutoState.goBack):
                //chassis.setJoystickData(0, 0.5, 0);
                if(System.currentTimeMillis() - m_timeStamp > 250)
                {
                    m_autoState = AutoState.translateRight;
                    m_timeStamp = System.currentTimeMillis();
                }
                break;
                
            case (AutoState.translateRight):
                System.out.println("Trying to translate!!!!!!!!!!!!!!!!!!!!!!!!!!");
                double adjustedRotation = GyroDrive.getAdjustedRotationValue(
                        0.9, 0.1, 0, gyroPConstant, gyroIConstant, 
                        spinThreshold, gyroValue - 2.37);
                //chassis.setJoystickData(0.9, 0.1, adjustedRotation);
                if(System.currentTimeMillis() - m_timeStamp > 1000)
                {
                    m_autoState = AutoState.done;
                }
                
                break;
                
            case (AutoState.done):
                chassis.setJoystickData(0, 0, 0);
                break;
                
            default:
                chassis.setJoystickData(0, 0, 0);
                break;
        }
    }
}
