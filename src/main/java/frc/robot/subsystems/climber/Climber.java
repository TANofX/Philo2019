/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class Climber extends Subsystem {
  private static final double CLIMB_DURATION_SECONDS = 3.0;
  private static final double ALLOWED_HEIGHT_ERROR_INCHES = 0.25;
  private static final double ALLOWED_DRIVE_ERROR_INCHES = 1.00;
  private static final double CALIBRATION_SPEED = 0.25; //Driving motor in a positive direction goes down, driving negative goes up
  private static final double DRIVE_SCALE_FACTOR = -1.0; 

  private static final double MAXIMUM_LIFT_HEIGHT_INCHES = 22.0;
  private static final double LEAD_SCREW_PITCH = -1.0;  // Inches per revolution
  private static final double ENCODER_PULSE_PER_REVOLUTION = 4096.0; // VEX Planetary Encoder with 4096 CPR
  private static final double LIFT_ENCODER_PULSE_PER_INCH = ENCODER_PULSE_PER_REVOLUTION / LEAD_SCREW_PITCH;
  //private static final double LIFT_ALLOWED_ERROR_PULSES = LIFT_ENCODER_PULSE_PER_INCH * ALLOWED_HEIGHT_ERROR_INCHES;
  private static final int LIFT_CRUISE_VELOCITY = 2800;//3200;//2800;// Theoretically, this value is in RPM of the lead screw // Original value with kF = 0.249799967 and kP = 1.0 = (int)(6.0 * LIFT_ENCODER_PULSE_PER_INCH / 100.0);
  private static int LIFT_PROFILE = 0;
  private static int LIFT_ALLOWED_ERROR = (int)Math.abs(Math.round(LIFT_ENCODER_PULSE_PER_INCH * ALLOWED_HEIGHT_ERROR_INCHES));
  private static double LIFT_MAX_ACCUMULATOR = 40000.0;

  private static final double WHEEL_DIAMETER = 2.5;
  private static final double DRIVE_ENCODER_PULSE_PER_INCH = ENCODER_PULSE_PER_REVOLUTION / (WHEEL_DIAMETER * Math.PI);
  //private static final double DRIVE_ALLOWED_ERROR_PULSES = DRIVE_ENCODER_PULSE_PER_INCH * ALLOWED_DRIVE_ERROR_INCHES;
  private static final int DRIVE_CRUISE_VELOCITY = (400);
  private static final int DRIVE_ACCELERATION = (50);
  private static int DRIVE_PROFILE = 0;

  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private TalonSRX liftMotor;
  private TalonSRX driveMotor;
  private TalonSRX liftFollower = null;

  public Climber(int liftMotorCANId, int driveMotorId) {
    liftMotor = new TalonSRX(liftMotorCANId);
    driveMotor = new TalonSRX(driveMotorId);


    liftMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    liftMotor.setSensorPhase(true);
    liftMotor.setInverted(false);
    liftMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    liftMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    liftMotor.configPeakCurrentLimit(60, 0);
    liftMotor.configPeakCurrentDuration(100, 0);
    liftMotor.configContinuousCurrentLimit(39);
    liftMotor.enableCurrentLimit(true);
    liftMotor.configVoltageCompSaturation(8.0);
    liftMotor.enableVoltageCompensation(false);
    // liftMotor.config_kP(LIFT_PROFILE, 0.06, 0);
    // liftMotor.config_kI(LIFT_PROFILE, 0.01, 0);
    // liftMotor.config_kD(LIFT_PROFILE, 0.0, 0);
    // liftMotor.config_kF(LIFT_PROFILE, 0.0, 0);
    // liftMotor.configAllowableClosedloopError(LIFT_PROFILE, LIFT_ALLOWED_ERROR);
    // liftMotor.configMaxIntegralAccumulator(LIFT_PROFILE, LIFT_MAX_ACCUMULATOR);
    liftMotor.configMotionAcceleration((int)(LIFT_CRUISE_VELOCITY * 6));
    liftMotor.configMotionCruiseVelocity(LIFT_CRUISE_VELOCITY);
    // liftMotor.configClearPositionOnLimitR(true, 0);

    driveMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    driveMotor.configMotionAcceleration(DRIVE_ACCELERATION);
    driveMotor.config_kP(DRIVE_PROFILE, 2.0, 0);
    driveMotor.config_kI(DRIVE_PROFILE, 0.0, 0);
    driveMotor.config_kD(DRIVE_PROFILE, 0.0, 0);
    driveMotor.config_kF(DRIVE_PROFILE, 3.0, 0);
    driveMotor.configMotionCruiseVelocity(DRIVE_CRUISE_VELOCITY);
  }

  public Climber(int liftMotorCANId, int driveMotorId, int liftFollowerCANId) {
    this(liftMotorCANId, driveMotorId);
    liftFollower = new TalonSRX(liftFollowerCANId);
    liftFollower.configPeakCurrentLimit(60, 0);
    liftFollower.configPeakCurrentDuration(100, 0);
    liftFollower.configContinuousCurrentLimit(39);
    liftFollower.enableCurrentLimit(true);
    liftFollower.follow(liftMotor);
  }    

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public double getCurrentHeightInches() {
    return getCurrentHeightTicks() / LIFT_ENCODER_PULSE_PER_INCH;
  }

  public int getCurrentHeightTicks() {
    return liftMotor.getSelectedSensorPosition();
  }

  public void zeroClimber() {
    liftMotor.setSelectedSensorPosition((int)(0.5 * ENCODER_PULSE_PER_REVOLUTION));
   }

  public boolean getDriveLimitSwitch() {
    return driveMotor.getSensorCollection().isRevLimitSwitchClosed();
  }

  public void calibrateLift() {
    liftMotor.set(ControlMode.PercentOutput, CALIBRATION_SPEED);
  }

  public void stopLift() {
    liftMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public void stopDrive() {
    driveMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public boolean goToHeightInches(double heightInches) {
    return goToHeightTicks((int)Math.round(heightInches * LIFT_ENCODER_PULSE_PER_INCH));
  }

  private boolean goToHeightTicks(int encoderPosition) {
    liftMotor.set(ControlMode.MotionMagic, encoderPosition);
    return true;
  }

  public boolean getTopLimitSwitch() {
    return liftMotor.getSensorCollection().isFwdLimitSwitchClosed(); //Note to self, limit switches don't need code to run
  }

  public boolean getBottomLimitSwitch() {
    return liftMotor.getSensorCollection().isRevLimitSwitchClosed();
  }

  public void clearStickyFaults() {

  }

  public double getDriveLocation() {
    return driveMotor.getSelectedSensorPosition();
  }

  public double getDriveLocationInches() {
    return (getDriveLocation() * DRIVE_ENCODER_PULSE_PER_INCH);
  }

  public boolean goToDistanceInches(Double distInches) {
    return goToDistanceTicks((int)Math.round(distInches * DRIVE_ENCODER_PULSE_PER_INCH));
  }

  private boolean goToDistanceTicks(int encoderPosition) {
    driveMotor.set(ControlMode.MotionMagic, encoderPosition);
    return true;
  }

  public void driveSpeed(double speedInchesPerSec) {
    double pulsesPer100MilliSec = DRIVE_SCALE_FACTOR * speedInchesPerSec * DRIVE_ENCODER_PULSE_PER_INCH / 10.0;
    driveMotor.set(ControlMode.Velocity, pulsesPer100MilliSec);
  }

  public void climbSpeed(boolean reverse) {
    double speed = 6.0;
    if (reverse) speed *= -1.0;

    driveSpeed(speed);
  }

  public boolean isAtHeightInches(double targetHeight) {
    if (Math.abs(targetHeight - getCurrentHeightInches()) <= ALLOWED_HEIGHT_ERROR_INCHES) {
      return true;
    }
    else{
      return false;
    }
  }

  public boolean isAtPosInches(double targetPos) {
   if (Math.abs(targetPos - getDriveLocationInches()) <= ALLOWED_DRIVE_ERROR_INCHES) {
     return true;
   } else {
     return false;
   } 
  }

  public void liftPercent(double percentSpeed) {
    liftMotor.set(ControlMode.PercentOutput, percentSpeed);
  }
  
  public double motorVoltage() {
    return liftMotor.getMotorOutputVoltage();
  }

  public double motorCurrent() {
    return liftMotor.getOutputCurrent();
  }

  public void enableLiftCompensation() {
    liftMotor.enableVoltageCompensation(true);
  }

  public double getLiftVelocityTarget() {
    return liftMotor.getActiveTrajectoryVelocity();
  }

  public double getLiftVelocity() {
    return liftMotor.getSelectedSensorVelocity();
  }
}
