package org.usfirst.frc.team3238.robot;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DigitalInput;

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

    public static class AutoState
    {
        public static final int collectingTote = 0, moveRightLineTrack = 1,
                moveRightAccelAssist = 2, moveBackwards = 3, dropTotes = 4,
                driveAway = 5, done = 6;
    }

    Autonomous(Chassis chassis, double infraredDistanceTrigger, 
    		long timeIgnore, int timeDriveBack, int timeDriveAway, 
    		double moveBackSpeed)
    {
        m_infraredDistanceTrigger = infraredDistanceTrigger;
        m_timeIgnore = timeIgnore;
        m_timeDriveBack = timeDriveBack;
        m_timeDriveAway = timeDriveAway;
        m_moveBackSpeed = moveBackSpeed;
    }

    /**
     * Re-initializes the autonomous period so that the variables are set up to
     * be at the beginning of the autonomous
     * 
     */
    void init()
    {
        m_collectCount = 0;
        m_timeStamp = System.currentTimeMillis();
        m_autoState = AutoState.collectingTote;
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
    void idle(Chassis chassis, DigitalInput reflectSensorRear,
            DigitalInput reflectSensorFront, int gyroValue,
            double spinThreshold, BuiltInAccelerometer accelerometer,
            double infraredDistance, ToteLifter toteLifter, Grabber grabber,
            PIController piContLifterLeft, PIController piContLifterRight)
    {
        switch(m_autoState)
        {
            case AutoState.collectingTote:
                chassis.setJoystickData(0, 0, 0);
                if(grabber.doneCollecting())
                // Grabber.collecting should return true when it
                // finishes collecting
                {
                    toteLifter.addTote();
                    // ToteLifter.storeTote tells the ToteLifter to start
                    // lifting the tote,
                    switch(m_collectCount)
                    {
                        case 0:
                            m_autoState = AutoState.moveRightLineTrack;
                            m_timeStamp = System.currentTimeMillis();
                            break;

                        case 1:
                            m_autoState = AutoState.moveRightAccelAssist;
                            m_timeStamp = System.currentTimeMillis();
                            break;

                        case 2:
                            m_autoState = AutoState.moveBackwards;
                            m_timeStamp = System.currentTimeMillis();
                            break;

                        default:
                            break;
                    }
                    m_collectCount += 1;
                }
                break;

            case AutoState.moveRightLineTrack:
                if((m_timeStamp >= m_timeIgnore)
                        && (infraredDistance < m_infraredDistanceTrigger))
                {
                    m_autoState = AutoState.collectingTote;
                    m_timeStamp = System.currentTimeMillis();
                    break;
                }
                LineTrack.lineTrack(reflectSensorFront, reflectSensorRear,
                        chassis, spinThreshold, gyroValue);
                break;

            case AutoState.moveRightAccelAssist:
                if((m_timeStamp >= m_timeIgnore)
                        && (infraredDistance < m_infraredDistanceTrigger))
                {
                    m_autoState = AutoState.collectingTote;
                    m_timeStamp = System.currentTimeMillis();
                    break;
                }
                chassis.setJoystickData(.5, 0, 0);
                break;

            case AutoState.moveBackwards:
                if(System.currentTimeMillis() - m_timeStamp >= m_timeDriveBack)
                {
                    m_autoState = AutoState.dropTotes;
                    toteLifter.dropTotes();
                    m_timeStamp = System.currentTimeMillis();
                    break;
                }
                chassis.setJoystickData(0, -m_moveBackSpeed, 0);
                break;

            case AutoState.dropTotes:
                chassis.setJoystickData(0, 0, 0);

                if(toteLifter.getTotesDropped())
                {
                    m_autoState = AutoState.driveAway;
                    m_timeStamp = System.currentTimeMillis();
                    break;
                }
                break;

            case AutoState.driveAway:
                if(System.currentTimeMillis() - m_timeStamp >= m_timeDriveAway)
                {
                    m_autoState = AutoState.done;
                    m_timeStamp = System.currentTimeMillis();
                    break;
                }
                chassis.setJoystickData(0, -m_moveBackSpeed, 0);
                break;

            case AutoState.done:
                chassis.setJoystickData(0, 0, 0);
                break;

            default:
                break;
        }
    }
}
