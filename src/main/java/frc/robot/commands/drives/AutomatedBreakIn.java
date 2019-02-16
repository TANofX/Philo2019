/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drives;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.drives.DriveBase;

public class AutomatedBreakIn extends Command {
  private DriveBase drives;
  private double minP;
  private double maxP;
  private double duration;
  private double currentP;
  private double scaleFactor = 1.0;

  private double rampRate;

  public AutomatedBreakIn(DriveBase driveSubsystem, double minPercent, double maxPercent, double startDirection, double durationSeconds) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    drives = driveSubsystem;
    requires(drives);

    minP = minPercent;
    maxP = maxPercent;
    duration = durationSeconds;

    rampRate = (maxP - minP) * 2.0 / durationSeconds;

    scaleFactor = startDirection;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    drives.stopMotors();

    if (scaleFactor < 0) {
      currentP = maxP;
    } else {
      currentP = minP;
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    drives.driveMotors(currentP);

    currentP += rampRate * scaleFactor;
    if (currentP > maxP) {
      scaleFactor = -1.0;
      currentP = minP;
    } else if (currentP < minP) {
      scaleFactor = 1.0;
      currentP = minP;
    }

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return timeSinceInitialized() > duration;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    drives.stopMotors();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    drives.stopMotors();
  }
}
