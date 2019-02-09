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
public class ClimbToHeight extends Command {
  private double heightToClimb;
  private Climber climberSubsystem;

  public ClimbToHeight(Climber subsystem, double heightInInches) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    climberSubsystem = subsystem;
    requires(climberSubsystem);
    
    heightToClimb = heightInInches;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    climberSubsystem.goToHeightInches(heightToClimb);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (climberSubsystem.isAtHeightInches(heightToClimb)) {
      return true;
    } else {
      return false;
    }
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
