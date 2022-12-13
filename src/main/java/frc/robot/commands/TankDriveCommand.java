package frc.robot.commands;

import com.rambots4571.rampage.joystick.Gamepad.Axis;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class TankDriveCommand extends CommandBase {

  private final DriveTrain driveTrain;

  public TankDriveCommand(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);
  }

  @Override
  public void execute() {
    driveTrain.tankDrive(
        RobotContainer.driveController.getAxisValue(Axis.LeftYAxis),
        RobotContainer.driveController.getAxisValue(Axis.RightYAxis));
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
