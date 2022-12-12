// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.rambots4571.rampage.joystick.Controller;
import com.rambots4571.rampage.joystick.Gamepad;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Settings;

public class RobotContainer {

  public static final Controller<Gamepad.Button, Gamepad.Axis> driveController =
    Gamepad.make(Settings.driveController);

  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }


  private void configureButtonBindings() {}


  public Command getAutonomousCommand() {

    return null;
  }
}
