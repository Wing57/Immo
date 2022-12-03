// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Settings;

public class DriveTrain extends SubsystemBase {

  private final List<CANSparkMax> allMotors;

  private final CANSparkMax leftMaster;
  private final CANSparkMax leftMotor2;
  private final CANSparkMax leftMotor3;

  private final CANSparkMax rightMaster;
  private final CANSparkMax rightMotor2;
  private final CANSparkMax rightMotor3;
  
  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;

  private final boolean rightMotorInvert;
  private final boolean leftMotorInvert;

  private final int currentLimit;
  private final double rampRate;

  private final DifferentialDrive drive;

  private final DifferentialDriveOdometry m_Odometry;

  private final Field2d field;

  private final AHRS navX;

  public DriveTrain() {
    leftMaster = new CANSparkMax(DriveConstants.LEFT_MOTOR_1, MotorType.kBrushless);
    leftMotor2 = new CANSparkMax(DriveConstants.LEFT_MOTOR_2, MotorType.kBrushless);
    leftMotor3 = new CANSparkMax(DriveConstants.LEFT_MOTOR_3, MotorType.kBrushless);

    rightMaster = new CANSparkMax(DriveConstants.LEFT_MOTOR_1, MotorType.kBrushless);
    rightMotor2 = new CANSparkMax(DriveConstants.LEFT_MOTOR_2, MotorType.kBrushless);
    rightMotor3 = new CANSparkMax(DriveConstants.LEFT_MOTOR_3, MotorType.kBrushless);

    //TODO: test ramprate
    currentLimit = 40;
    rampRate = 0.25;

    allMotors =
      Arrays.asList(leftMaster, leftMotor2, leftMotor3, rightMaster, rightMotor2, rightMotor3);
    
    allMotors.forEach(
        motor -> {

          // Factory Resets all Brushless NEOs
          motor.restoreFactoryDefaults();

          // Current limit to prevent breaker tripping. Approx at 150% of rated
          // current supply.
          motor.setSmartCurrentLimit(currentLimit);

          // Ramping motor output to prevent instantaneous directional changes
          // (Values need testing)
          motor.setClosedLoopRampRate(rampRate);

        }
    );
    
    //TODO: Verify inversions
    leftMotorInvert = true;
    rightMotorInvert = false;

    leftMaster.setInverted(leftMotorInvert);
    rightMaster.setInverted(rightMotorInvert);

    leftMotor2.follow(leftMaster, leftMotorInvert);
    leftMotor3.follow(leftMaster, leftMotorInvert);
    
    rightMotor2.follow(rightMaster, rightMotorInvert);
    rightMotor3.follow(rightMaster, rightMotorInvert);

    drive = new DifferentialDrive(leftMaster, rightMaster);

    leftEncoder = leftMaster.getEncoder();
    rightEncoder = rightMaster.getEncoder();

    navX = new AHRS(SPI.Port.kMXP);

    m_Odometry = new DifferentialDriveOdometry(navX.getRotation2d());
    field = new Field2d();
    resetOdometry(Settings.STARTING_POSITION);

    // Put field on the spot RUHEUHEUHEHE
    SmartDashboard.putData("Field", field);
  }

  // *****************************************
  // ************** Encoders *****************
  // *****************************************

  //TODO: Figure out if native unit conversion is needed
  public double getLeftDistance() {
    return leftEncoder.getPosition();
  }

  public double getRightDistance() {
    return rightEncoder.getPosition();
  }

  public double getDistance() {
    return (getLeftDistance() + getRightDistance()) / 2.0;
  }

  // Velocity
  public double getLeftVelocity() {
    return leftEncoder.getVelocity();
  }

  public double getRightVelocity() {
    return rightEncoder.getVelocity();
  }

  public double getVelocity() {
    return (getLeftVelocity() + getRightVelocity()) / 2.0;
  }

  // Reset
  public void resetEncoders() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  // *****************************************
  // ************* Robot Angle ***************
  // *****************************************

  public double getRawGyroAngle() {
    return navX.getAngle();
  }

  //TODO: test that the angle stuff checks out
  public double getGyroAngle() {
    return navX.getAngle() % 360;
  }

  public double getHeading() {
    return navX.getRotation2d().getDegrees();
  }

  public Rotation2d getRotation2d() {
    return navX.getRotation2d();
  }

  public void zeroHeading() {
    navX.reset();
  }

  // ********************************************
  // ************ Odometry Functions ************
  // ********************************************

  public void updateOdometry() {
    m_Odometry.update(
      getRotation2d(), 
      getLeftDistance(), 
      getRightDistance()
    );
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      getLeftVelocity(), 
      getRightVelocity()
      );
  }

  public Pose2d getPose() {
    updateOdometry();
    return m_Odometry.getPoseMeters();
  }

  public Field2d getField() {
    return field;
  }

  public void resetOdometry(Pose2d pose2d) {
    resetEncoders();
    zeroHeading();

    m_Odometry.resetPosition(pose2d, getRotation2d());
  }

  public void reset() {
    resetOdometry(getPose());
  }

  @Override
  public void periodic() {
    updateOdometry();
    field.setRobotPose(getPose());
  }
}
