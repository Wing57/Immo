package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.RobotContainer;
import frc.robot.commands.drive.DriveTrainRamsete;

public class FiveBall extends SequentialCommandGroup {

  private static final String PATH_1 = "paths/ExPath1.wpilib.json";
  private static final String PATH_2 = "paths/ExPath2.wpilib.json";
  private static final String PATH_3 = "paths/ExPath3.wpilib.json";
  private static final String PATH_4 = "paths/ExPath4.wpilib.json";

  public FiveBall(RobotContainer container) {
    addCommands(
        new DriveTrainRamsete(container.driveTrain, PATH_1).robotRelative(),
        new DriveTrainRamsete(container.driveTrain, PATH_2).fieldRelative(),
        new DriveTrainRamsete(container.driveTrain, PATH_3).fieldRelative(),
        new DriveTrainRamsete(container.driveTrain, PATH_4).fieldRelative());
  }
}
