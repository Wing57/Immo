package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class IDriveTrain extends SubsystemBase {
    
    public abstract void tankDrive(double left, double right);

    public abstract void arcadeDrive(double xSpeed, double zRotation);

    public abstract void stopMotors();

    public abstract double getLeftDistance();

    public abstract double getRightDistance();

    public abstract double getDistance();
}
