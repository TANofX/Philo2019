/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drives.DriveBase;

public class ClimbToLevel extends CommandGroup {
  /**
   * Add your docs here.
   */
  public ClimbToLevel(Climber frontClimber, Climber rearClimber, double heightInInches, DriveBase driveBase) {
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
    addSequential(new SynchronizedClimb(frontClimber, rearClimber, heightInInches));
    addSequential(new MoveDistance(frontClimber, rearClimber, driveBase, 15));
    addSequential(new ClimbToHeight(frontClimber, 0));
    addSequential(new MoveDistance(frontClimber, rearClimber, driveBase, 12));
    addSequential(new ClimbToHeight(rearClimber, 0));
    addSequential(new MoveDistance(frontClimber, rearClimber, driveBase, 3));
  }
}
