/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberBrake;
import frc.robot.subsystems.drives.DriveBase;
import frc.robot.subsystems.pidgeonimu.PidgeonIMU;

public class ClimbToLevel extends CommandGroup {
  /**
   * Add your docs here.
   */
  public ClimbToLevel(Climber frontClimber, Climber rearClimber, ClimberBrake brake, PidgeonIMU pigeon, double heightInInches, DriveBase driveBase) {
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
    addSequential(new ClimbToHeight(frontClimber, rearClimber, brake, pigeon, heightInInches, heightInInches));
    addSequential(new MoveDistance(frontClimber, rearClimber, driveBase, 15));
    addSequential(new ClimbToHeight(frontClimber, rearClimber, brake, pigeon, 0.0, heightInInches));
    addSequential(new MoveDistance(frontClimber, rearClimber, driveBase, 12));
    // addParallel(new MoveDistance(frontClimber, rearClimber, driveBase, 999));
    addSequential(new ClimbToHeightWithDrive(frontClimber, rearClimber, brake, pigeon,driveBase, 0.0, 0.0));  
    // addSequential(new ClimbToHeight(frontClimber, rearClimber, brake, pigeon, 0.0, 0.0));  
    addSequential(new MoveDistance(frontClimber, rearClimber, driveBase, 3));
  }
}
