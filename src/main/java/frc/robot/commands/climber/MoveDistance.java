/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drives.DriveBase;

/***
 * This Command sends the appropriate signals to the specified climber subsystem
 * to move to a given height (up or down) form it's current position.
 */
public class MoveDistance extends Command {
  private Climber climberSubsystem;
  private Climber climberSubsystem2;
  private DriveBase driveBaseSubsystem;
  private double distanceToMove;
  private double startingDist1;
  private double startingDist2;

  public MoveDistance(Climber subsystem, Climber subsystem2, DriveBase dbsubsystem, double distanceToMoveInches) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    climberSubsystem = subsystem;
    requires(climberSubsystem);

    climberSubsystem2 = subsystem2;
    requires(climberSubsystem2); 
    
    driveBaseSubsystem = dbsubsystem;
    requires(dbsubsystem);

    distanceToMove = distanceToMoveInches;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    startingDist1 = climberSubsystem.getDriveLocationInches();
    startingDist2 = climberSubsystem2.getDriveLocationInches();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //driveBaseSubsystem.driveSpeed(1);
    climberSubsystem.driveSpeed(1);
    climberSubsystem2.driveSpeed(1);
    driveBaseSubsystem.driveSpeed(1);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if ((climberSubsystem.isAtPosInches(distanceToMove + startingDist1) && climberSubsystem2.isAtPosInches(distanceToMove + startingDist2))
    || (climberSubsystem.getDriveLimitSwitch())
    || (climberSubsystem2.getDriveLimitSwitch())) {
      return true;
    } else {
      return false; 
    }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    driveBaseSubsystem.stopMotors();
    climberSubsystem.stopDrive();
    climberSubsystem2.stopDrive();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
