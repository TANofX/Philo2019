/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.hatch;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.hatch.HatchCollector;

public class HatchHingeToggle extends Command {
  private HatchCollector collector;
  private boolean extendHinge = false;

  public HatchHingeToggle(HatchCollector hatch) {
    super("hinge deploy", 1);
    collector = hatch;
    requires(collector);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if (collector.getHingePosition() == 1) {
      extendHinge = false;
    } else if (collector.getHingePosition() == -1) {
      extendHinge = true;
    }

    collector.hingeExtend(extendHinge);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (((extendHinge) && (collector.getHingePosition() == 1)) || ((!extendHinge) && (collector.getHingePosition() == -1))) {
      return true;
    }

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
