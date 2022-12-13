package frc.robot.commands.drive;

import com.rambots4571.rampage.joystick.Gamepad.Axis;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class ArcadeDrive extends CommandBase {
    
    private final DriveTrain driveTrain;

    public ArcadeDrive(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
        addRequirements(driveTrain);
    }

    @Override
    public void execute() {
        driveTrain.arcadeDrive(
            RobotContainer.driveController.getAxisValue(Axis.LeftYAxis), 
            RobotContainer.driveController.getAxisValue(Axis.RightXAxis));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
