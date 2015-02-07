package org.usfirst.frc.team3238.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.AnalogInput;
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

    public static class AutoState
    {
        public static final int collectingTote = 0, moveRightLineTrack = 1,
                moveRightAccelAssist = 2, moveBackwards = 3, dropTotes = 4,
                driveAway = 5, done = 6;
    }

    Autonomous(Chassis chassis)
    {
        m_infraredDistanceTrigger = 0.5; //half a meter
        m_timeIgnore = 1000;
    }

    void init(Chassis chassis)
    {
        chassis.setJoystickData(0, 0, 0);
        m_collectCount = 0;
        m_timeStamp = System.currentTimeMillis();
        m_autoState = AutoState.collectingTote;
    }

    void idle(Chassis chassis, DigitalInput reflectSensorRear, 
            DigitalInput reflectSensorFront, AnalogInput irSensor, 
                int gyroValue, double spinThreshold, BuiltInAccelerometer accelerometer, double infraredDistance )
    {
        switch(m_autoState)
        {
            case AutoState.collectingTote:
				chassis.setJoystickData(0, 0, 0);
				if(Collector.collecting())
                //Collector.collecting should return true when it finishes collecting
                {
                    switch (m_collectCount)
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
                if((m_timeStamp >= m_timeIgnore) && (infraredDistance < m_infraredDistanceTrigger))
                {
                	m_autoState = AutoState.collectingTote;
                	m_timeStamp = System.currentTimeMillis();
                	break;
                }
                LineTrack.lineTrack(reflectSensorFront, reflectSensorRear, chassis, 
                        spinThreshold, gyroValue);
                break;

            case AutoState.moveRightAccelAssist:
            	if((m_timeStamp >= m_timeIgnore) && (infraredDistance < m_infraredDistanceTrigger))
                {
                	m_autoState = AutoState.collectingTote;
                	m_timeStamp = System.currentTimeMillis();
                	break;
                }
				AccelAssist.moveRight(chassis, accelerometer);
                break;

            case AutoState.moveBackwards:
                if(System.currentTimeMillis() - m_timeStamp >= 2000)
                {
                    m_autoState = AutoState.dropTotes;
                    m_timeStamp = System.currentTimeMillis();
                    break;
                }
                chassis.setJoystickData(0,-.5,0);
                break;

            case AutoState.dropTotes:
	            if(Collector.dropTotes)
				//Collector.dropTotes should return true when the totes have been dropped
	            {
	                m_autoState = AutoState.driveAway;
	                m_timeStamp = System.currentTimeMillis();
	                break;
	            }
                break;

            case AutoState.driveAway:
                if(System.currentTimeMillis() - m_timeStamp >= 1000)
                {
                    m_autoState = AutoState.done;
                    m_timeStamp = System.currentTimeMillis();
                    break;
                }
				chassis.setJoystickData(0,-.5,0);
                break;

            case AutoState.done:
            	chassis.setJoystickData(0,0,0);
                break;

            default:
                break;
        }
    }
}
