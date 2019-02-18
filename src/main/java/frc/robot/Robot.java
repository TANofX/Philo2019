/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.climber.CalibrateClimber;
import frc.robot.commands.climber.ClimbToHeight;
import frc.robot.commands.climber.MoveDistance;
import frc.robot.commands.climber.SynchronizedClimb;
import frc.robot.commands.drives.AutomatedBreakIn;
import frc.robot.commands.drives.AutomatedBreakInComplete;
import frc.robot.commands.drives.DriveForward;
import frc.robot.commands.drives.ShiftHighGear;
import frc.robot.commands.hatch.HatchHingeToggle;
import frc.robot.commands.hatch.HatchExtendToggle;
import frc.robot.commands.hatch.HatchRelease;
import frc.robot.commands.vision.CameraSwitcher;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberBrake;
import frc.robot.subsystems.drives.DriveBase;
import frc.robot.subsystems.hatch.HatchCollector;
import frc.robot.subsystems.pidgeonimu.PidgeonIMU;
import frc.robot.subsystems.vision.Limelight;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static DriveBase m_drives = new DriveBase(RobotMap.LEFT_MASTER_MOTOR_ID
                                                  , RobotMap.LEFT_FOLLOWER_1_ID
                                                  , RobotMap.LEFT_FOLLOWER_2_ID
                                                  , RobotMap.RIGHT_MASTER_MOTOR_ID
                                                  , RobotMap.RIGHT_FOLLOWER_1_ID
                                                  , RobotMap.RIGHT_FOLLOWER_2_ID
                                                  , RobotMap.PCM_ID
                                                  , RobotMap.GEARSHIFT_PCM_PORT);
  public static Climber m_frontClimber = new Climber(RobotMap.FRONT_LIFT_MOTOR_ID
                                                    , RobotMap.FRONT_DRIVE_MOTOR_ID);
  public static Climber m_rearClimber = new Climber(RobotMap.REAR_LIFT_MOTOR_ID
                                                  , RobotMap.REAR_DRIVE_MOTOR_ID);
  public static HatchCollector m_hatch = new HatchCollector(RobotMap.PCM_ID
                                                          , RobotMap.HATCH_HINGE_EXTEND_PCM_PORT
                                                          , RobotMap.HATCH_HINGE_RETRACT_PCM_PORT
                                                          , RobotMap.HATCH_EXTEND_PCM_PORT
                                                          , RobotMap.HATCH_RETRACT_PCM_PORT
                                                          , RobotMap.HATCH_PUSHOFF_PCM_PORT
                                                          , RobotMap.HINGE_EXTEND_DIO_PORT
                                                          , RobotMap.HINGE_RETRACT_DIO_PORT);
  public static PidgeonIMU m_pigeon = new PidgeonIMU(RobotMap.PIDGEON_IMU_ID);
  public static OI m_oi;
  public static ClimberBrake m_brake = new ClimberBrake(RobotMap.PCM_ID, RobotMap.LIFT_BRAKE_PCM_PORT);
  public static Limelight m_vision = new Limelight();

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_oi = new OI();
  //  m_chooser.setDefaultOption("Default Auto", new ExampleCommand());
    // chooser.addOption("My Auto", new MyAutoCommand());
  //  SmartDashboard.putData("Auto mode", m_chooser);
  //  m_oi.frontClimbButton.whileHeld(new SynchronizedClimb(m_frontClimber, m_rearClimber, 4.0));
    m_oi.hatchHinge.whenPressed(new HatchHingeToggle(m_hatch));
    m_oi.hatchExtend.whenPressed(new HatchExtendToggle(m_hatch));
    m_oi.hatchPushOff.whenPressed(new HatchRelease(m_hatch));

    m_oi.gearShiftButton.whileHeld(new ShiftHighGear(m_drives));
    m_oi.cameraSwitchButton.whenPressed(new CameraSwitcher(m_vision));

    SmartDashboard.putData("Test Climb Drive", new MoveDistance(m_frontClimber, m_rearClimber, m_drives, 12.0));
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_chooser.getSelected();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
