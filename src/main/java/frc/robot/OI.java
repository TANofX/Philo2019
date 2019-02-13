/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.*;
import frc.robot.commands.climber.*;
import frc.robot.commands.drives.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.climber.ClimbToHeight;
import frc.robot.commands.climber.ClimbToLevel;

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
  Joystick stick = new Joystick(0);
  Joystick xbox = new Joystick(1);

  //Buttons for Climb
  //I'm guessing below is just an example?
  //Button frontClimbButton = new JoystickButton(stick, 1);
  
  //Button climberAutolvl2 = new JoystickButton(xbox, xboxbuttonX);
  //Button climberAutolvl3 = new JoystickButton(xbox, xboxbuttonB);
  //"Xboxbutton" is a placeholder until I figure out what actually goes in there.

  Button frontClimbUp = new JoystickButton(stick, 7);
  Button backClimbUp = new JoystickButton(stick, 8);

  Button frontClimbDown = new JoystickButton(stick, 9);
  Button backClimbDown = new JoystickButton(stick, 10);

  Button climbMoveForward = new JoystickButton(stick, 11);
  Button climbMoveBackwards = new JoystickButton(stick, 12);

  //Buttons for drive

  //Buttons for hatch
  //Consult later - how do triggers work?
  Button hatchExtend = new JoystickButton(stick, 5);
  Button hatchHinge = new JoystickButton(stick, 1);
  //also consult what "trigger" in line above is actually supposed to be
  Button hatchPushOff = new JoystickButton(stick, 2);

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
