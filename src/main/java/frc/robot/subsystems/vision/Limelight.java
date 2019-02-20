/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class Limelight extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private static final String NETWORK_TABLE_NAME = "limelight";
  private static final String LIMELIGHT_LED_MODE = "ledMode";
  private static final String LIMELIGHT_CAMERA_MODE = "camMode";
  private static final String LIMELIGHT_STREAM = "stream";
  private static final String LIMELIGHT_SNAPSHOT = "snapshot";
  private static final String LIMELIGHT_PIPELINE = "pipeline";

  private final NetworkTable limelightTable;

  public Limelight() {
    limelightTable = NetworkTableInstance.getDefault().getTable(NETWORK_TABLE_NAME);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public double getTargetingValue(String targetValueId) {
    return limelightTable.getEntry(targetValueId).getDouble(Double.MAX_VALUE);
  } 

  public void setLEDMode(LEDMode newMode) {
    limelightTable.getEntry(LIMELIGHT_LED_MODE).setNumber(newMode.tableValue());
  }

  public LEDMode getLEDMode() {
    double value = limelightTable.getEntry(LIMELIGHT_LED_MODE).getDouble(LEDMode.DEFAULT.tableValue());
  
    for (LEDMode mode: LEDMode.values()) {
      if (mode.tableValue() == (int)value) {
        return mode;
      }
    }

    return LEDMode.DEFAULT;
  }

  public void setCameraMode(CamMode newMode) {
    limelightTable.getEntry(LIMELIGHT_CAMERA_MODE).setNumber(newMode.tableValue());
  }

  public CamMode getCameraMode() {
    double value = limelightTable.getEntry(LIMELIGHT_CAMERA_MODE).getDouble(CamMode.VISION.tableValue());
  
    for (CamMode mode: CamMode.values()) {
      if (mode.tableValue() == (int)value) {
        return mode;
      }
    }

    return CamMode.VISION;
  }

  public void setStreamMode(StreamMode newMode) {
    limelightTable.getEntry(LIMELIGHT_STREAM).setNumber(newMode.tableValue());
  }

  public StreamMode getStreamMode() {
    double value = limelightTable.getEntry(LIMELIGHT_STREAM).getDouble(StreamMode.DEFAULT.tableValue());
  
    for (StreamMode mode: StreamMode.values()) {
      if (mode.tableValue() == (int)value) {
        return mode;
      }
    }

    return StreamMode.DEFAULT;
  }

  public void setSnapshotMode(SnapshotMode newMode) {
    limelightTable.getEntry(LIMELIGHT_SNAPSHOT).setNumber(newMode.tableValue());
  }

  public SnapshotMode getSnapshotMode() {
    double value = limelightTable.getEntry(LIMELIGHT_SNAPSHOT).getDouble(SnapshotMode.OFF.tableValue());
  
    for (SnapshotMode mode: SnapshotMode.values()) {
      if (mode.tableValue() == (int)value) {
        return mode;
      }
    }

    return SnapshotMode.OFF;
  }

  public void setPipeline(int pipelineId) {
    limelightTable.getEntry(LIMELIGHT_PIPELINE).setNumber(pipelineId);
  }

  public enum LEDMode {
    DEFAULT (0),
    OFF (1),
    BLINK (2),
    ON (3);
  
    private final int value;
  
    LEDMode(int enumVal) {
      value = enumVal;
    }
  
    public int tableValue() {
      return value;
    }
  }

  public enum CamMode {
    VISION (0),
    DRIVER (1);

    private final int value;

    CamMode(int camVal) {
      value = camVal;
    }

    public int tableValue() {
      return value;
    }
  }

  public enum StreamMode {
    DEFAULT (0),
    PIP_MAIN (1),
    PIP_SECONDARY (2);

    private int value;

    StreamMode(int val) {
      value = val;
    }

    public int tableValue() {
      return value;
    }
  }

  public enum SnapshotMode {
    OFF (0),
    ON (1);

    private int value;

    SnapshotMode(int val) {
      value = val;
    }

    public int tableValue() {
      return value;
    }
  }
}
