/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.climber.Climber;

/***
 * This Command sends the appropriate signals to the specified climber subsystem
 * to move to a given height (up or down) form it's current position.
 */
public class HoldPosition extends Command {
  private Climber climberSubsystem;
  private double targetHeight;

  public HoldPosition(Climber subsystem) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    climberSubsystem = subsystem;
    requires(climberSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    targetHeight = climberSubsystem.getCurrentHeightInches();
    if (targetHeight < 0) {
      targetHeight = 0;
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    climberSubsystem.goToHeightInches(targetHeight);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
