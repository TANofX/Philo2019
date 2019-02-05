/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.drives;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.drives.DefaultJoystickDrive;

/**
 * Add your docs here.
 */
public class DriveBase extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private TalonSRX leftMasterMotor;
  private TalonSRX leftFollower1;
  private TalonSRX leftFollower2;

  private TalonSRX rightMasterMotor;
  private TalonSRX rightFollower1;
  private TalonSRX rightFollower2;

  public DriveBase( int leftMasterCANId
                  , int leftFollower1CANId
                  , int leftFollower2CANId
                  , int rightMasterCANId
                  , int rightFollower1CANId
                  , int rightFollower2CANId) {
    /* Configure left side drive base motors.  We may need to create a subclass like we have used
       in prior years in order to use speed control */
    leftMasterMotor = new TalonSRX(leftMasterCANId);
    leftFollower1 = new TalonSRX(leftFollower1CANId);
    leftFollower2 = new TalonSRX(leftFollower2CANId);

    /* Configure right side drive base motors.  We may need to create a subclass like we have used
       in prior years in order to use speed control */
    rightMasterMotor = new TalonSRX(rightMasterCANId);
    rightFollower1 = new TalonSRX(rightFollower1CANId);
    rightFollower2 = new TalonSRX(rightFollower2CANId);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new DefaultJoystickDrive(this));
  }
}
