// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.rambots4571.rampage.joystick.Controller;
import com.rambots4571.rampage.joystick.Gamepad;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.Settings;
import frc.robot.commands.drive.ArcadeDriveCommand;
import frc.robot.commands.drive.TankDriveCommand;
import frc.robot.subsystems.DriveTrain;

public class RobotContainer {

  // Joysticks
  public static final Controller<Gamepad.Button, Gamepad.Axis> driveController =
      Gamepad.make(Settings.driveController);

  // Subsystems
  public final DriveTrain driveTrain;

  // Commands
  public final TankDriveCommand tankDriveCommand;
  public final ArcadeDriveCommand arcadeDriveCommand;

  public RobotContainer() {
    // Subsystems
    driveTrain = new DriveTrain();

    // Commands
    tankDriveCommand = new TankDriveCommand(driveTrain);
    arcadeDriveCommand = new ArcadeDriveCommand(driveTrain);

    driveTrain.setDefaultCommand(tankDriveCommand);

    configureButtonBindings();
  }

  private void configureButtonBindings() {}

  public Command getAutonomousCommand() {

    return null;
  }
}
