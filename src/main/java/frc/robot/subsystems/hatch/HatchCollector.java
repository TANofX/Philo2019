/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.hatch;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class HatchCollector extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private Compressor compressorControl;
  private DoubleSolenoid hatchLift;
  private DoubleSolenoid hatchExtend;
  private Solenoid hatchRelease;
  private boolean started = false;

  public HatchCollector(  int pcmCANId
                        , int liftIdA
                        , int liftIdB
                        , int extendIdA
                        , int extendIdB
                        , int releaseId) {
    compressorControl = new Compressor(pcmCANId);
    hatchLift = new DoubleSolenoid(pcmCANId, liftIdA, liftIdB);
    hatchExtend = new DoubleSolenoid(pcmCANId, extendIdA, extendIdB);
    hatchRelease = new Solenoid(pcmCANId, releaseId);
  }

  public void startup() {
    compressorControl.setClosedLoopControl(true);
    hatchLift.set(DoubleSolenoid.Value.kOff);
    hatchExtend.set(DoubleSolenoid.Value.kOff);
    hatchRelease.set(false);
    started = true;
  }

  public void shutdown() {
    hatchLift.set(DoubleSolenoid.Value.kReverse);
    hatchExtend.set(DoubleSolenoid.Value.kReverse);
    hatchRelease.set(false);
    compressorControl.setClosedLoopControl(false);
    started = false;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
