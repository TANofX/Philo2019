/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberBrake;

/***
 * This Command sends the appropriate signals to the specified climber subsystem
 * to move to a given height (up or down) form it's current position.
 */
public class ReverseCalibrateClimber extends Command {
  private Climber climberSubsystem;
  private Climber frontClimber;
  private ClimberBrake climberBrake;

  public ReverseCalibrateClimber(Climber front, Climber subsystem, ClimberBrake brake) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    //super("Calibrate Climber", 0.5);
    frontClimber = front;
    requires(frontClimber);
    climberSubsystem = subsystem;
    requires(climberSubsystem);
    climberBrake = brake;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    climberBrake.releaseBrake();
    climberSubsystem.liftPercent(-1.0);;
    frontClimber.liftPercent(-1.0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    SmartDashboard.putNumber("Front Target", frontClimber.getLiftVelocityTarget());
    SmartDashboard.putNumber("Rear Target", climberSubsystem.getLiftVelocityTarget());
    SmartDashboard.putNumber("Front Velocity", frontClimber.getLiftVelocity());
    SmartDashboard.putNumber("Rear Velocity", climberSubsystem.getLiftVelocity());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (climberSubsystem.getBottomLimitSwitch() && frontClimber.getBottomLimitSwitch());
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    climberSubsystem.stopLift();
    frontClimber.stopLift();
    climberBrake.setBrake();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
