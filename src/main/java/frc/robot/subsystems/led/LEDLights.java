/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.led;

import java.awt.Color;

import com.mach.LightDrive.LightDriveCAN;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class LEDLights extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private Solenoid blueLights = null;
  private LightDriveCAN lightDrive = null;
  private Color currentColor = Color.BLACK;

  // This constructor creates a LightDrive component to control the lighting.
  // This allows more precise control of the actual lighting than the alternative
  // through the PCM.
  public LEDLights() {
    lightDrive = new LightDriveCAN();
  }

  public LEDLights(int pcmID, int blueLightsId)  {
    blueLights = new Solenoid(pcmID, blueLightsId);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void green(int value) {
    if (lightDrive != null) {
      currentColor = new Color(currentColor.getRed(), value, currentColor.getBlue());
      lightDrive.SetColor(1, currentColor);
    }
  }

  public void red(int value) {
    if (lightDrive != null) {
      currentColor = new Color(value, currentColor.getGreen(), currentColor.getBlue());
      lightDrive.SetColor(1, currentColor);
    }
  }

  public void blue(int value) {
    if (blueLights != null) {
      if (value > 0) {
        blueLights.set(true);
      } else {
        blueLights.set(false);
      }
    } else if (lightDrive != null) {
      currentColor = new Color(currentColor.getRed(), currentColor.getGreen(), value);
      lightDrive.SetColor(1, currentColor);
    }
  }
}
