/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberBrake;
import frc.robot.subsystems.pidgeonimu.PidgeonIMU;

public class AngleLimitedClimberDrive extends Command {
  private static final double ANGLE_LIMIT = 3.0;
  private static final double MIN_SPEED = 0.2;
  private static final double ADJUSTED_RANGE = 0.5;

  private Climber driveClimber;
  private ClimberBrake brakeSystem;
  private PidgeonIMU imuSystem;
  private double speed;
  private boolean opposite;

  public AngleLimitedClimberDrive(Climber theClimber, ClimberBrake theBrake, PidgeonIMU theIMU, double driveSpeed, boolean oppositeImpact) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    driveClimber = theClimber;
    brakeSystem = theBrake;
    imuSystem = theIMU;
    speed = driveSpeed;
    opposite = oppositeImpact;

    requires(theClimber);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double currentAngle = imuSystem.getpitch();
    int direction = (int)Math.signum(speed * currentAngle);
    double currentSpeed = 0.0;

    if (opposite) direction *= -1;

    if ((direction > 0) && (Math.abs(currentAngle) > ANGLE_LIMIT)) {
      driveClimber.liftPercent(0.0);
    } else {
      currentSpeed = speed;

      double adjustment = (ANGLE_LIMIT - Math.abs(currentAngle)) / ADJUSTED_RANGE;

      if (adjustment < 1.0) {
        currentSpeed = Math.signum(speed) * (MIN_SPEED + (adjustment * (Math.abs(speed) - MIN_SPEED)));
      }

      brakeSystem.releaseBrake();
      driveClimber.liftPercent(currentSpeed);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;  //(driveClimber.getTopLimitSwitch() || driveClimber.getBottomLimitSwitch());
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    interrupted();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    brakeSystem.setBrake();
    driveClimber.stopLift();
  }
}
