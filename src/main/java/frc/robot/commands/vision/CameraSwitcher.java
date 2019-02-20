/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.vision.Limelight;

public class CameraSwitcher extends Command {
  private Limelight visionControl;

  public CameraSwitcher(Limelight subsystem) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);

    visionControl = subsystem;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    switch (visionControl.getStreamMode()) {
      case PIP_SECONDARY:
        visionControl.setStreamMode(Limelight.StreamMode.PIP_MAIN);
        break;
      default:
        visionControl.setStreamMode(Limelight.StreamMode.PIP_SECONDARY);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
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
