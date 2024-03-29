/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */

public class OI {
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  Joystick stick = new Joystick(ButtonMap.STICK);
  Joystick xbox = new Joystick(ButtonMap.XBOX);

  //Buttons for Climb
  //I'm guessing below is just an example?
  //Button frontClimbButton = new JoystickButton(stick, 1);
  
  //Button climberAutolvl2 = new JoystickButton(xbox, xboxbuttonX);
  //Button climberAutolvl3 = new JoystickButton(xbox, xboxbuttonB);
  //"Xboxbutton" is a placeholder until I figure out what actually goes in there.

  Button frontClimbUp = new JoystickButton(stick, ButtonMap.FRONT_UP);
  Button backClimbUp = new JoystickButton(stick, ButtonMap.BACK_UP);

  Button frontClimbDown = new JoystickButton(stick, ButtonMap.FRONT_DOWN);
  Button backClimbDown = new JoystickButton(stick, ButtonMap.BACK_DOWN);

  Button climbMoveForward = new JoystickButton(stick, ButtonMap.DRIVE_FORWARD);
  Button climbMoveBackwards = new JoystickButton(stick, ButtonMap.DRIVE_BACKWARD);

  //Buttons for simple auto adjustments of the climber system
  Button climbLevelThreeButton = new JoystickButton (xbox, ButtonMap.LEVEL_THREE);
  Button climblevelTwoButton = new JoystickButton(xbox, ButtonMap.LEVEL_TWO);
  Button zeroClimberButton = new JoystickButton(xbox, ButtonMap.CLIMBER_ZERO);

  //Buttons relating to gearshifts
  Button lowGearButton = new JoystickButton(xbox, ButtonMap.LOW_GEAR);
  Button highGearButton =new JoystickButton(xbox, ButtonMap.HIGH_GEAR);
  //Buttons for drive
 // Button gearShiftButton = new JoystickButton(xbox, ButtonMap.GEAR_SHIFT);
  Button cameraSwitchButton = new JoystickButton(xbox, ButtonMap.CAMERA_SWITCHER);
  
  //Vision buttons
 public Button visionButton = new JoystickButton(xbox, ButtonMap.VISION_BUTTON); 
 public Button cancelButton = new JoystickButton(xbox, ButtonMap.CANCEL_BUTTON);
 public Button altVisionButton = new JoystickButton(stick, ButtonMap.ALT_VISION_BUTTON);
  //Buttons for hatch
  Button hatchHinge = new JoystickButton(stick, ButtonMap.RAISE_LOWER);
	Button hatchExtend = new JoystickButton(stick, ButtonMap.GRAB_HATCH);
  Button hatchPushOff = new JoystickButton(stick, ButtonMap.DETACH_HATCH);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ClimbToHeight());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());


  //Climb Triggers
  //climberAutolvl2.whenPressed(new ClimbToLevelCommand());
  //climberAutolvl3.whenPressed(new ClimbToLevelCommand());

  //frontClimbUp.whileHeld(new ClimbToHeightCommand());
  //backClimbUp.whileHeld(new ClimbToHeightCommand());

  //frontClimbDown.whileHeld(new ClimbToHeightCommand());
  //backClimbDown.whileHeld(new ClimbToHeightCommand());

  //climbMoveForward.whileHeld(new MoveDistanceCommand());
  //climbMoveBackwards.whileHeld(new MoveDistanceCommand());

  //Drive Triggers
  //drive.whileHeld(new placeholder());
  //turn.whileHeld(new placeholder());

  //highGear.whenPressed(new placeholder());
  //lowGear.whenPressed(new placeholder());

  //hatch Triggers
  //hatchExtend.whenActive(new placeholder());
  //hatchHinge.whenActive(new placeholder());
  //hatchPushOff.whenActive(new placeholder());

  public Joystick getJoystick(){
    return stick;
  }

  public Joystick getXbox(){
    return xbox;
  }

}
