/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.drives;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private Solenoid gearShift;

  private DifferentialDrive driveBase;

  private double currentInchesPerTick;

  public static int SPEED_CONTROL = 1;
	public static int POSITION_CONTROL = 0;
  
  public static double LOW_GEAR_RAMP_RATE = 0.5;
  public static double HIGHT_GEAR_RAMP_RATE = 1.0;

	private static double MAX_POSITION_ERROR = 125.0;
	private static double MAX_VELOCITY_ERROR = 50.0;

  private static final double PRIMARY_GEAR_RATIO = 42.0 / 11.0;
  private static final double SECONDARY_GEAR_RATIO_HIGH = 60.0 / 14.0;
  private static final double SECONDARY_GEAR_RATIO_LOW = 50.0 / 14.0;

  private static final int TICKS_PER_REVOLUTION_INPUT = 250 * 4; // Assuming quadrature encoding
  private static final double TICKS_PER_REVOLUTION_OUTPUT_HIGH = TICKS_PER_REVOLUTION_INPUT * SECONDARY_GEAR_RATIO_HIGH * PRIMARY_GEAR_RATIO;
  private static final double TICKS_PER_REVOLUTION_OUTPUT_LOW = TICKS_PER_REVOLUTION_INPUT * SECONDARY_GEAR_RATIO_LOW * PRIMARY_GEAR_RATIO;
  private static final double WHEEL_DIAMETER_INCHES = 6.0;
  private static final double WHEEL_CIRCUMFERENCE_INCHES = Math.PI * WHEEL_DIAMETER_INCHES;
  private static final double INCHES_PER_TICK_HIGH = WHEEL_CIRCUMFERENCE_INCHES / TICKS_PER_REVOLUTION_OUTPUT_HIGH;
  private static final double INCHES_PER_TICK_LOW = WHEEL_CIRCUMFERENCE_INCHES / TICKS_PER_REVOLUTION_OUTPUT_LOW;

  public DriveBase( int leftMasterCANId
                  , int leftFollower1CANId
                  , int leftFollower2CANId
                  , int rightMasterCANId
                  , int rightFollower1CANId
                  , int rightFollower2CANId
                  , int pcmId
                  , int gearShiftId) {
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

    gearShift = new Solenoid(pcmId, gearShiftId);

    leftFollower1.follow(leftMasterMotor);
    leftFollower2.follow(leftMasterMotor);

    rightFollower1.follow(rightMasterMotor);
    rightFollower2.follow(rightMasterMotor);

    //leftMasterMotor.selectProfileSlot(0, SPEED_CONTROL);
    //rightMasterMotor.selectProfileSlot(0, SPEED_CONTROL);

		//We generally start in low gear, so let's set the state machine to that
		//currentGearState = false;
		
		leftFollower1.setInverted(InvertType.OpposeMaster);
		leftMasterMotor.configSelectedFeedbackSensor(com.ctre.phoenix.motorcontrol.FeedbackDevice.QuadEncoder, 0, 0);

		
		rightFollower1.setInverted(InvertType.OpposeMaster);
		rightMasterMotor.configSelectedFeedbackSensor(com.ctre.phoenix.motorcontrol.FeedbackDevice.QuadEncoder, 0, 0);
		
		int timeoutMs = 10;
		
		// leftMasterMotor.enableCurrentLimit(true);
		// leftFollower1.enableCurrentLimit(true);
		// leftFollower2.enableCurrentLimit(true);
		
		// rightMasterMotor.enableCurrentLimit(true);
		// rightFollower1.enableCurrentLimit(true);
		// rightFollower2.enableCurrentLimit(true);
    leftMasterMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    rightMasterMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

		// leftMasterMotor.configAllowableClosedloopError(POSITION_CONTROL, (int)MAX_POSITION_ERROR, timeoutMs);
		// leftMasterMotor.configAllowableClosedloopError(SPEED_CONTROL, (int)MAX_VELOCITY_ERROR, timeoutMs);
		// rightMasterMotor.configAllowableClosedloopError(POSITION_CONTROL, (int)MAX_POSITION_ERROR, timeoutMs);
		// rightMasterMotor.configAllowableClosedloopError(SPEED_CONTROL, (int)MAX_VELOCITY_ERROR, timeoutMs);
		
		// leftMasterMotor.config_kP(POSITION_CONTROL, 1.0, timeoutMs);
		// leftMasterMotor.config_kI(POSITION_CONTROL, 0.0, timeoutMs);
		// leftMasterMotor.config_kD(POSITION_CONTROL, 0.0, timeoutMs);
		// leftMasterMotor.config_kF(POSITION_CONTROL, 0.2379, timeoutMs);
		
		// leftMasterMotor.config_kP(SPEED_CONTROL, 2.0, timeoutMs);
		// leftMasterMotor.config_kI(SPEED_CONTROL, 0.0, timeoutMs);
		// leftMasterMotor.config_kD(SPEED_CONTROL, 0.0, timeoutMs);
		// leftMasterMotor.config_kF(SPEED_CONTROL, 0.01, timeoutMs);
		
		// rightMasterMotor.config_kP(POSITION_CONTROL, 1.0, timeoutMs);
		// rightMasterMotor.config_kI(POSITION_CONTROL, 0.0, timeoutMs);
		// rightMasterMotor.config_kD(POSITION_CONTROL, 0.0, timeoutMs);
		// rightMasterMotor.config_kF(POSITION_CONTROL, 0.2379, timeoutMs);
		
		// rightMasterMotor.config_kP(SPEED_CONTROL, 2.0, timeoutMs);
		// rightMasterMotor.config_kI(SPEED_CONTROL, 0.0, timeoutMs);
		// rightMasterMotor.config_kD(SPEED_CONTROL, 0.0, timeoutMs);
		// rightMasterMotor.config_kF(SPEED_CONTROL, 0.01, timeoutMs);
        
    // leftMasterMotor.configMaxIntegralAccumulator(SPEED_CONTROL, 25000);
    // rightMasterMotor.configMaxIntegralAccumulator(SPEED_CONTROL, 25000);
    driveBase = new DifferentialDrive(leftMasterMotor, rightMasterMotor);
    //driveBase.setDeadband(0.09);

    lowGear();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new DefaultJoystickDrive(this));
  }

  public void stopMotors()
  {
    leftMasterMotor.set(ControlMode.PercentOutput, 0);
    rightMasterMotor.set(ControlMode.PercentOutput, 0);
  }

  public DifferentialDrive getDriveBase()
  {
    return driveBase;
  }

  public void driveMotors(double speed) {
    leftMasterMotor.set(ControlMode.PercentOutput, speed);
    rightMasterMotor.set(ControlMode.PercentOutput, -1 * speed);
  }

  public void climbSpeed() {
    // leftMasterMotor.set(ControlMode.Velocity, speedInchesPerSecond / currentInchesPerTick / 100.0);
    // rightMasterMotor.set(ControlMode.Velocity, -1 * speedInchesPerSecond / currentInchesPerTick / 100.0);
    //leftMasterMotor.set(ControlMode.Velocity, 24000.0);
    //rightMasterMotor.set(ControlMode.Velocity, -24000.0);
    driveBase.arcadeDrive(0.35, 0.0);
  }

  public void highGear() {
    leftMasterMotor.configClosedloopRamp(1.0);
    rightMasterMotor.configClosedloopRamp(1.0);
    gearShift.set(true);
    currentInchesPerTick = INCHES_PER_TICK_HIGH;
  }

  public void lowGear() {
    leftMasterMotor.configClosedloopRamp(0.5);
    rightMasterMotor.configClosedloopRamp(0.5);
    gearShift.set(false);
    currentInchesPerTick = INCHES_PER_TICK_LOW;
  }

  public void dashboardOutput() {
    SmartDashboard.putNumber("Left Velocity", leftMasterMotor.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Right Velocity", rightMasterMotor.getSelectedSensorVelocity());

    SmartDashboard.putNumber("Left Position", leftMasterMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Right Position", rightMasterMotor.getSelectedSensorPosition());
  }
}
