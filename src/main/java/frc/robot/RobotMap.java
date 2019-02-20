/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
  public static final int LEFT_MASTER_MOTOR_ID = 12;
  public static final int LEFT_FOLLOWER_1_ID = 3;
  public static final int LEFT_FOLLOWER_2_ID = 4;

  public static final int RIGHT_MASTER_MOTOR_ID = 8;
  public static final int RIGHT_FOLLOWER_1_ID = 10;
  public static final int RIGHT_FOLLOWER_2_ID = 11;

  public static final int FRONT_LIFT_MOTOR_ID = 5;
  public static final int FRONT_DRIVE_MOTOR_ID = 13;

  public static final int REAR_LIFT_MOTOR_ID = 2;
  public static final int REAR_DRIVE_MOTOR_ID = 9;
  
  public static final int PDP_ID = 6;
  public static final int PIDGEON_IMU_ID = 7;
  public static final int PCM_ID = 1;
  public static final int ROBO_RIO_ID = 0;

  public static final int GEARSHIFT_PCM_PORT = 0;
  public static final int HATCH_HINGE_EXTEND_PCM_PORT = 1;
  public static final int HATCH_HINGE_RETRACT_PCM_PORT = 2;
  public static final int HATCH_EXTEND_PCM_PORT = 3;
  public static final int HATCH_RETRACT_PCM_PORT = 4;
  public static final int HATCH_PUSHOFF_PCM_PORT = 5;
  public static final int LIFT_BRAKE_PCM_PORT = 6;

  public static final int HINGE_EXTEND_DIO_PORT = 0;
  public static final int HINGE_RETRACT_DIO_PORT = 1;

  public static final int PRESSURE_GAUGE_ANALOGUE_INPUT = 0;
}
