/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.hatch;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.subsystems.led.LEDLights;

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
  private DigitalInput extendSwitch;
  private DigitalInput retractSwitch;

  private LEDLights lights = null;

  public HatchCollector(  int pcmCANId
                        , int liftIdA
                        , int liftIdB
                        , int extendIdA
                        , int extendIdB
                        , int releaseId
                        , int extendIO
                        , int retractIO) {
    compressorControl = new Compressor(pcmCANId);
    hatchLift = new DoubleSolenoid(pcmCANId, liftIdA, liftIdB);
    hatchExtend = new DoubleSolenoid(pcmCANId, extendIdA, extendIdB);
    hatchRelease = new Solenoid(pcmCANId, releaseId);
    extendSwitch = new DigitalInput(extendIO);
    retractSwitch = new DigitalInput(retractIO);
  }

  public HatchCollector(  int pcmCANId
                        , int liftIdA
                        , int liftIdB
                        , int extendIdA
                        , int extendIdB
                        , int releaseId
                        , int extendIO
                        , int retractIO
                        , LEDLights lightSubsystem) {
                          this(pcmCANId, liftIdA, liftIdB, extendIdA, extendIdB, releaseId, extendIO, retractIO);

                          lights = lightSubsystem;
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
  public void hingeExtend(boolean hingeState) {
    hatchLift.set(hingeState ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
  }
  public void hatchRelease(boolean hingeState) {
    hatchRelease.set(hingeState);
    // Originally, this took a boolean, now we take a value
    if (lights != null) lights.blue(hingeState ? 255 : 0);
  }
  public void hatchExtend(boolean hingeState) {
    hatchExtend.set(hingeState ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
  }
  public int getHingePosition(){
    switch (hatchLift.get()) {
      case kForward:
        return 1;
      case kReverse:
        return -1;
      default:
        return 0;
    }
  }
  public int getExtendPosition(){
    switch (hatchExtend.get()) {
      case kForward:
        return 1;
      case kReverse:
        return -1;
      default:
        return 0;
    }
  }
  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
