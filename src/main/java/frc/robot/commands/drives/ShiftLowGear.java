/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drives;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.drives.DriveBase;

public class ShiftLowGear extends Command {
  private DriveBase driveBase;
  
  public ShiftLowGear(DriveBase dBase) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    driveBase = dBase;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
      driveBase.lowGear();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    //driveBase.lowGear();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
