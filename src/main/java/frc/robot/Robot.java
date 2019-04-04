/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.commands.vision.CameraSwitcher;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.climber.AngleLimitedClimberDrive;
import frc.robot.commands.climber.CalibrateClimber;
import frc.robot.commands.climber.ClimbToHeight;
import frc.robot.commands.climber.ClimbToLevel;
import frc.robot.commands.climber.MoveDistance;
import frc.robot.commands.climber.MotorOutput;
import frc.robot.commands.climber.ReleaseBrake;
import frc.robot.commands.climber.ReverseCalibrateClimber;
import frc.robot.commands.climber.SynchronizedClimb;
import frc.robot.commands.drives.AutomatedBreakIn;
import frc.robot.commands.drives.AutomatedBreakInComplete;
import frc.robot.commands.drives.DriveForward;
import frc.robot.commands.drives.ShiftHighGear;
import frc.robot.commands.hatch.HatchHingeToggle;
import frc.robot.commands.hatch.HatchExtendToggle;
import frc.robot.commands.CancelEverything;
import frc.robot.commands.hatch.HatchRelease;
import frc.robot.commands.vision.CameraSwitcher;
import frc.robot.commands.vision.FollowTarget;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberBrake;
import frc.robot.subsystems.drives.DriveBase;
import frc.robot.subsystems.hatch.HatchCollector;
import frc.robot.subsystems.hatch.PressureGauge;
import frc.robot.subsystems.led.LEDLights;
import frc.robot.subsystems.pidgeonimu.PidgeonIMU;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.commands.drives.ShiftLowGear;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // Using this version, we create a single light version that uses the PCM
  public static LEDLights m_lights = new LEDLights(RobotMap.PCM_ID, RobotMap.BLUE_LIGHT_PCM_PORT, RobotMap.GREEN_LIGHT_PCM_PORT);
  // Comment out the previous line and uncomment this next line to use the LightDrive on CAN
  //public static LEDLights m_lights = new LEDLights();

  public static DriveBase m_drives = new DriveBase(RobotMap.LEFT_MASTER_MOTOR_ID
                                                  , RobotMap.LEFT_FOLLOWER_1_ID
                                                  , RobotMap.LEFT_FOLLOWER_2_ID
                                                  , RobotMap.RIGHT_MASTER_MOTOR_ID
                                                  , RobotMap.RIGHT_FOLLOWER_1_ID
                                                  , RobotMap.RIGHT_FOLLOWER_2_ID
                                                  , RobotMap.PCM_ID
                                                  , m_lights
                                                  , RobotMap.GEARSHIFT_PCM_PORT);
  public static Climber m_frontClimber = new Climber(RobotMap.FRONT_LIFT_MOTOR_ID
                                                    , RobotMap.FRONT_DRIVE_MOTOR_ID
                                                    , RobotMap.FRONT_LIFT_FOLLOWER_ID);
  public static Climber m_rearClimber = new Climber(RobotMap.REAR_LIFT_MOTOR_ID
                                                  , RobotMap.REAR_DRIVE_MOTOR_ID);
  public static HatchCollector m_hatch = new HatchCollector(RobotMap.PCM_ID
                                                          , RobotMap.HATCH_HINGE_EXTEND_PCM_PORT
                                                          , RobotMap.HATCH_HINGE_RETRACT_PCM_PORT
                                                          , RobotMap.HATCH_EXTEND_PCM_PORT
                                                          , RobotMap.HATCH_RETRACT_PCM_PORT
                                                          , RobotMap.HATCH_PUSHOFF_PCM_PORT
                                                          , RobotMap.HINGE_EXTEND_DIO_PORT
                                                          , RobotMap.HINGE_RETRACT_DIO_PORT
                                                          , m_lights);
  public static PidgeonIMU m_pigeon = new PidgeonIMU(RobotMap.PIDGEON_IMU_ID);
  public static OI m_oi;
  public static ClimberBrake m_brake = new ClimberBrake(RobotMap.PCM_ID, RobotMap.LIFT_BRAKE_PCM_PORT);
  public static Limelight m_vision = new Limelight();
  public static PressureGauge m_pressureGuage = new PressureGauge(RobotMap.PRESSURE_GAUGE_ANALOGUE_INPUT);

  Command m_autonomousCommand;
  Command m_calibrateClimber = new CalibrateClimber(m_frontClimber, m_rearClimber, m_brake);
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    //m_frontClimber.enableLiftCompensation();
    m_oi = new OI();
  //  m_chooser.setDefaultOption("Default Auto", new ExampleCommand());
    // chooser.addOption("My Auto", new MyAutoCommand());
  //  SmartDashboard.putData("Auto mode", m_chooser);
  //  m_oi.frontClimbButton.whileHeld(new SynchronizedClimb(m_frontClimber, m_rearClimber, 4.0));
    m_oi.hatchHinge.whenPressed(new HatchHingeToggle(m_hatch));
    m_oi.hatchExtend.whenPressed(new HatchExtendToggle(m_hatch));
    m_oi.hatchPushOff.whileHeld(new HatchRelease(m_hatch));

    //m_oi.gearShiftButton.whileHeld(new ShiftHighGear(m_drives));
    m_oi.cameraSwitchButton.whenPressed(new CameraSwitcher(m_vision));

    m_oi.highGearButton.whenPressed(new ShiftHighGear(m_drives));
    m_oi.lowGearButton.whenPressed(new ShiftLowGear(m_drives));
    //confused on what to do for low gear
    //m_oi.lowGearButton.whileHeld(new ShiftLowGear(m_drives));
   
    m_oi.climbLevelThreeButton.whenPressed(new ClimbToLevel(m_frontClimber, m_rearClimber, m_brake, m_pigeon, 20.5, m_drives));
    m_oi.climblevelTwoButton.whenPressed( new ClimbToLevel(m_frontClimber, m_rearClimber, m_brake, m_pigeon, 7.0, m_drives));
    m_oi.zeroClimberButton.whenPressed(new ClimbToHeight(m_frontClimber, m_rearClimber, m_brake, m_pigeon, 0.0, 0.0));

    //SmartDashboard.putData("Test Climb Drive", new MoveDistance(m_frontClimber, m_rearClimber, m_drives, 12.0));
    SmartDashboard.putData("Calibrate Climber", new CalibrateClimber(m_frontClimber, m_rearClimber, m_brake));
    //SmartDashboard.putData("Do NOT Push", new ReverseCalibrateClimber(m_frontClimber, m_rearClimber, m_brake));

    // SmartDashboard.putData("Sync 19.5", new ClimbToHeight(m_frontClimber, m_rearClimber, m_brake, m_pigeon, 19.5, 19.5));
    // SmartDashboard.putData("Sync 14", new ClimbToHeight(m_frontClimber, m_rearClimber, m_brake, m_pigeon, 14.0, 14.0));
    // SmartDashboard.putData("Sync 8", new ClimbToHeight(m_frontClimber, m_rearClimber, m_brake, m_pigeon, 8.0, 8.0));
    // SmartDashboard.putData("Sync 4", new ClimbToHeight(m_frontClimber, m_rearClimber, m_brake, m_pigeon, 4.0, 4.0));
    // SmartDashboard.putData("Sync 0", new ClimbToHeight(m_frontClimber, m_rearClimber, m_brake, m_pigeon, 0.0, 0.0));

    // SmartDashboard.putData("Full Low Climb", new ClimbToLevel(m_frontClimber, m_rearClimber, m_brake, m_pigeon, 7.0, m_drives));
    // SmartDashboard.putData("Full High Climb", new ClimbToLevel(m_frontClimber, m_rearClimber, m_brake, m_pigeon, 20.5, m_drives));

    // SmartDashboard.putData("Breakin Drive System", new AutomatedBreakInComplete());
    // SmartDashboard.putData("Drive Forward", new DriveForward(m_drives));

    SmartDashboard.putData("Release Brake", new ReleaseBrake(m_brake));

    m_oi.backClimbDown.whileHeld(new AngleLimitedClimberDrive(m_rearClimber, m_brake, m_pigeon, 0.5, false));
    m_oi.backClimbUp.whileHeld(new AngleLimitedClimberDrive(m_rearClimber, m_brake, m_pigeon, -0.5, false));
    m_oi.frontClimbDown.whileHeld(new AngleLimitedClimberDrive(m_frontClimber, m_brake, m_pigeon, 0.25, true));
    m_oi.frontClimbUp.whileHeld(new AngleLimitedClimberDrive(m_frontClimber, m_brake, m_pigeon, -1.0, true));

    m_oi.climbMoveForward.whileHeld(new MoveDistance(m_frontClimber, m_rearClimber, m_drives, 15));
    m_oi.climbMoveBackwards.whileHeld(new MoveDistance(m_frontClimber, m_rearClimber, m_drives, -15));

 // Preferences for vision parameters
 double Kp = Preferences.getInstance().getDouble("Kp", 0.01);
 double Ki = Preferences.getInstance().getDouble("Ki", 0.0001);
 double sMin = Preferences.getInstance().getDouble("turn", 0.17);
 double driveKp = Preferences.getInstance().getDouble("driveP", 0.001);
 double driveKi = Preferences.getInstance().getDouble("driveI", 0.0);
 double dMin = Preferences.getInstance().getDouble("drive", 0.17);
 double tRange = Preferences.getInstance().getDouble("turnRange", 0.5);
 double dRange = Preferences.getInstance().getDouble("driveRange", 5);
 int tWidth = Preferences.getInstance().getInt("targetWidth", 350);

 Command followTarget = new FollowTarget(m_vision
 , m_drives
 , Kp
 , Ki
 , sMin
 , driveKp
 , driveKi
 , dMin
 , tWidth
 , tRange
 , dRange);

 m_oi.visionButton.whenPressed(followTarget);
 m_oi.altVisionButton.whenPressed(followTarget);

 m_oi.cancelButton.whenPressed(new CancelEverything(m_drives, null, null));
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
    //m_autonomousCommand = m_chooser.getSelected();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.start();
    // }
    m_calibrateClimber.start();
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
