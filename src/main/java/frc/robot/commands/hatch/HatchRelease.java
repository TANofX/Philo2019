/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.hatch;




import edu.wpi.first.wpilibj.command.TimedCommand;
import frc.robot.subsystems.hatch.HatchCollector;
import frc.robot.subsystems.led.LEDLights;

public class HatchRelease extends TimedCommand {
  private HatchCollector collector;

  public HatchRelease(HatchCollector hatch) {
  super(0.25);
    collector = hatch;
  requires(collector); 
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  //                                                                                                 Called just before this Command runs the first time
  @Override
  protected void initialize() {
    collector.hatchRelease(true);
    //collector.hatchExtend(true);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //if(timeSinceInitialized() > 500){
    // collector.hatchExtend(false);
    //}
    //else if(timeSinceInitialized() > 250){
    //  collector.hatchRelease(true);
    //}
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    //return timeSinceInitialized() > 600;
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    collector.hatchRelease(false);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
     collector.hatchRelease(false);
     collector.hatchExtend(false);
  }
}
