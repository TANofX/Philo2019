/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.pidgeonimu;

import com.ctre.phoenix.sensors.PigeonIMU;
import frc.robot.commands.climber.DefaultPigeon;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class PidgeonIMU extends Subsystem {
  private PigeonIMU imu;
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
public PidgeonIMU(int pidgeonID) {
  imu = new PigeonIMU(pidgeonID);
}

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new DefaultPigeon(this));
  }

  public double getYaw() {
    double [] ypr = new double [3];
    imu.getYawPitchRoll(ypr);
    return ypr[0];
  }

  public double getpitch() {
    double [] ypr = new double [3];
    imu.getYawPitchRoll(ypr);
    return ypr[2];
  }
  
  public void zeroyaw() {
    imu.setYaw(0.0);
  }

  public double [] getState() {
    double [] ypr = new double [3];
    imu.getYawPitchRoll(ypr);
    return ypr;
  }

  
  }
