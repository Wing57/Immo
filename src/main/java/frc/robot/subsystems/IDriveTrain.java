package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class IDriveTrain extends SubsystemBase {
    
    public abstract void tankDrive(double left, double right);

    public abstract void arcadeDrive(double xSpeed, double zRotation);

    public abstract void stopMotors();

    public abstract void resetEncoders();

    public abstract void zeroHeading();

    public abstract double getLeftDistance();

    public abstract double getRightDistance();

    public abstract double getLeftVelocity();

    public abstract double getRightVelocity();

    public abstract Rotation2d getRotation2d();

    public abstract Field2d getField();

    public abstract Pose2d getPose();

}
