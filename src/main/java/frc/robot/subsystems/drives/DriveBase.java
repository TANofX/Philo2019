/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.drives;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.commands.drives.DefaultJoystickDrive;

/**
 * Add your docs here.
 */
public class DriveBase extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private DriveMotor leftMasterMotor;
  private DriveMotor leftFollower1;
  private DriveMotor leftFollower2;

  private DriveMotor rightMasterMotor;
  private DriveMotor rightFollower1;
  private DriveMotor rightFollower2;

  private DifferentialDrive driveBase;

  public static int SPEED_CONTROL = 1;
	public static int POSITION_CONTROL = 0;
	
	private static double MAX_POSITION_ERROR = 125.0;
	private static double MAX_VELOCITY_ERROR = 50.0;

  public DriveBase( int leftMasterCANId
                  , int leftFollower1CANId
                  , int leftFollower2CANId
                  , int rightMasterCANId
                  , int rightFollower1CANId
                  , int rightFollower2CANId) {
    /* Configure left side drive base motors.  We may need to create a subclass like we have used
       in prior years in order to use speed control */
    leftMasterMotor = new DriveMotor(leftMasterCANId);
    leftFollower1 = new DriveMotor(leftFollower1CANId);
    leftFollower2 = new DriveMotor(leftFollower2CANId);

    /* Configure right side drive base motors.  We may need to create a subclass like we have used
       in prior years in order to use speed control */
    rightMasterMotor = new DriveMotor(rightMasterCANId);
    rightFollower1 = new DriveMotor(rightFollower1CANId);
    rightFollower2 = new DriveMotor(rightFollower2CANId);

    leftFollower1.follow(leftMasterMotor);
    leftFollower2.follow(leftMasterMotor);

    rightFollower1.follow(rightMasterMotor);
    rightFollower2.follow(rightMasterMotor);

		//We generally start in low gear, so let's set the state machine to that
		//currentGearState = false;
		
		leftFollower1.setInverted(true);
		leftMasterMotor.configSelectedFeedbackSensor(com.ctre.phoenix.motorcontrol.FeedbackDevice.QuadEncoder, 0, 0);

		
		rightFollower1.setInverted(true);
		rightMasterMotor.configSelectedFeedbackSensor(com.ctre.phoenix.motorcontrol.FeedbackDevice.QuadEncoder, 0, 0);
		
		int timeoutMs = 10;
		
		leftMasterMotor.enableCurrentLimit(true);
		leftFollower1.enableCurrentLimit(true);
		leftFollower2.enableCurrentLimit(true);
		
		rightMasterMotor.enableCurrentLimit(true);
		rightFollower1.enableCurrentLimit(true);
		rightFollower2.enableCurrentLimit(true);
		
		leftMasterMotor.configAllowableClosedloopError(POSITION_CONTROL, (int)MAX_POSITION_ERROR, timeoutMs);
		leftMasterMotor.configAllowableClosedloopError(SPEED_CONTROL, (int)MAX_VELOCITY_ERROR, timeoutMs);
		rightMasterMotor.configAllowableClosedloopError(POSITION_CONTROL, (int)MAX_POSITION_ERROR, timeoutMs);
		rightMasterMotor.configAllowableClosedloopError(SPEED_CONTROL, (int)MAX_VELOCITY_ERROR, timeoutMs);
		
		leftMasterMotor.config_kP(POSITION_CONTROL, 1.0, timeoutMs);
		leftMasterMotor.config_kI(POSITION_CONTROL, 0.0, timeoutMs);
		leftMasterMotor.config_kD(POSITION_CONTROL, 0.0, timeoutMs);
		leftMasterMotor.config_kF(POSITION_CONTROL, 0.2379, timeoutMs);
		
		leftMasterMotor.config_kP(SPEED_CONTROL, 0.1, timeoutMs);
		leftMasterMotor.config_kI(SPEED_CONTROL, 0.0, timeoutMs);
		leftMasterMotor.config_kD(SPEED_CONTROL, 0.0, timeoutMs);
		leftMasterMotor.config_kF(SPEED_CONTROL, 0.2379, timeoutMs);
		
		rightMasterMotor.config_kP(POSITION_CONTROL, 1.0, timeoutMs);
		rightMasterMotor.config_kI(POSITION_CONTROL, 0.0, timeoutMs);
		rightMasterMotor.config_kD(POSITION_CONTROL, 0.0, timeoutMs);
		rightMasterMotor.config_kF(POSITION_CONTROL, 0.2379, timeoutMs);
		
		rightMasterMotor.config_kP(SPEED_CONTROL, 0.1, timeoutMs);
		rightMasterMotor.config_kI(SPEED_CONTROL, 0.0, timeoutMs);
		rightMasterMotor.config_kD(SPEED_CONTROL, 0.0, timeoutMs);
		rightMasterMotor.config_kF(SPEED_CONTROL, 0.2379, timeoutMs);
				
    driveBase = new DifferentialDrive(leftMasterMotor, rightMasterMotor);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new DefaultJoystickDrive(this));
  }

  public void stopMotors()
  {
    leftMasterMotor.set(0);
    rightMasterMotor.set(0);
  }

  public DifferentialDrive getDriveBase()
  {
    return driveBase;
  }

  public void driveMotors(double speed) {
    leftMasterMotor.set(speed);
    rightMasterMotor.set(speed);
  }
}
