/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drives;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.ButtonMap;
import frc.robot.Robot;
import frc.robot.subsystems.drives.DriveBase;

public class DefaultJoystickDrive extends Command {
  private DriveBase driveSubsystem;

  public DefaultJoystickDrive(DriveBase subsystem) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    driveSubsystem = subsystem;
    requires(driveSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double driveRate = -1.0 * Robot.m_oi.getXbox().getRawAxis(ButtonMap.DRIVE_AXIS);
    double turnRate = Robot.m_oi.getXbox().getRawAxis(ButtonMap.TURN_AXIS);

    // We want to adjust the turn rate based on the right analog trigger input
    // In this case, we want the turn rate cut in half if the trigger is not pulled 
    // and allowed to go to the maximum value when the trigger is pulled
    // Since the trigger registers values from 0 to 1 instead of -1 to 1, we will
    // Add one and divide by 2 to that the value returned goes from 0.5 to 1, we will then
    // use this factor to adjust the turnRate value from the primary turn axis
     turnRate *= (Robot.m_oi.getXbox().getRawAxis(ButtonMap.TURN_ADJUST_AXIS) + 1.0) / 2.0;
    // driveRate *= (Robot.m_oi.getXbox().getRawAxis(ButtonMap.TURN_ADJUST_AXIS) + 1.0) / 2.0;
    // driveSubsystem.getDriveBase().arcadeDrive(  driveRate
    //                                           , turnRate
    //                                           , false);
    SmartDashboard.putNumber("driveRate", driveRate);
    SmartDashboard.putNumber("turnRate", turnRate);
    double rampRate = SmartDashboard.getNumber("NewRampRate", 0.5);
    driveSubsystem.AdjustRampRate(rampRate);
    driveSubsystem.getDriveBase().curvatureDrive(driveRate, turnRate, Math.abs(driveRate) < 0.15);
    //driveSubsystem.getDriveBase().arcadeDrive(driveRate, turnRate, true);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
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
    driveSubsystem.stopMotors();
  }
}
