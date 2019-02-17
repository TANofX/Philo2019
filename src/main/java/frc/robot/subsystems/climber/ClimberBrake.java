/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class ClimberBrake extends Subsystem {
  private Solenoid brakeSolenoid;
  
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public ClimberBrake(int pcmID, int brakeId) {
    brakeSolenoid = new Solenoid(pcmID, brakeId);
  }

  public void releaseBrake() {
    brakeSolenoid.set(true);
  }

  public void setBrake() {
    brakeSolenoid.set(false);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
