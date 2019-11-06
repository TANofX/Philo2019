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
import frc.robot.subsystems.drives.DriveBase;
import frc.robot.subsystems.pidgeonimu.PidgeonIMU;

/***
 * This Command sends the appropriate signals to the specified climber subsystem
 * to move to a given height (up or down) form it's current position.
 */
public class ClimbToHeightWithDrive extends Command {
  private double frontHeight;
  private double rearHeight;
  private Climber frontClimber;
  private Climber backClimber;
  private ClimberBrake climberBrake;
  private DriveBase driveBaseSubsystem;
  private PidgeonIMU imu;
  private static final double COMPENSATION = 50.0;

  public ClimbToHeightWithDrive(Climber Front, Climber Back, ClimberBrake Brake, PidgeonIMU Pigeon,DriveBase dbsubsystem, double frontHeightInInches, double backHeightInInches) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    frontClimber = Front;
    requires(frontClimber);
    backClimber = Back;
    requires(backClimber);
    climberBrake = Brake;
    requires(climberBrake);
    driveBaseSubsystem = dbsubsystem;
    requires(driveBaseSubsystem);
    this.imu = Pigeon;
    
    frontHeight = frontHeightInInches;
    rearHeight = backHeightInInches;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    climberBrake.releaseBrake();
    frontClimber.goToHeightInches(frontHeight);
    backClimber.goToHeightInches(rearHeight);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // double pitch = imu.getpitch();
    // backClimber.liftPercent(pitch * COMPENSATION);
    SmartDashboard.putNumber("Front Target", frontClimber.getLiftVelocityTarget());
    SmartDashboard.putNumber("Rear Target", backClimber.getLiftVelocityTarget());
    SmartDashboard.putNumber("Front Velocity", frontClimber.getLiftVelocity());
    SmartDashboard.putNumber("Rear Velocity", backClimber.getLiftVelocity());


    driveBaseSubsystem.driveMotors(0.03);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
//    if (frontClimber.isAtHeightInches(heightToClimb)) {
    if (frontClimber.isAtHeightInches(frontHeight) && backClimber.isAtHeightInches(rearHeight)) {
        return true;
    } else {
      return false;
    }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    backClimber.stopLift();
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
