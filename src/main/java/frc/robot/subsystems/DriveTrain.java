// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project. GOD I HATE JAVAHOME

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Settings;
import java.util.Arrays;
import java.util.List;

public class DriveTrain extends SubsystemBase {

  private final List<CANSparkMax> allMotors;

  private final CANSparkMax leftMaster, leftMotor2, leftMotor3;
  private final CANSparkMax rightMaster, rightMotor2, rightMotor3;

  private final CANSparkMax[] leftMotors;
  private final CANSparkMax[] rightMotors;

  private final RelativeEncoder leftEncoder, rightEncoder;
  private final boolean rightMotorInvert, leftMotorInvert;

  private final int currentLimit;
  private final double rampRate;

  private final DifferentialDrive drive;
  private final DifferentialDriveOdometry m_Odometry;

  private final Field2d field;

  private final AHRS navX;

  public DriveTrain() {
    // NEOs
    leftMaster = new CANSparkMax(DriveConstants.LEFT_MOTOR_1, MotorType.kBrushless);
    leftMotor2 = new CANSparkMax(DriveConstants.LEFT_MOTOR_2, MotorType.kBrushless);
    leftMotor3 = new CANSparkMax(DriveConstants.LEFT_MOTOR_3, MotorType.kBrushless);

    rightMaster = new CANSparkMax(DriveConstants.LEFT_MOTOR_1, MotorType.kBrushless);
    rightMotor2 = new CANSparkMax(DriveConstants.LEFT_MOTOR_2, MotorType.kBrushless);
    rightMotor3 = new CANSparkMax(DriveConstants.LEFT_MOTOR_3, MotorType.kBrushless);

    // Motor groups
    leftMotors = new CANSparkMax[] {leftMaster, leftMotor2, leftMotor3};

    rightMotors = new CANSparkMax[] {rightMaster, rightMotor2, rightMotor3};

    // TODO: test ramprate
    currentLimit = 40;
    rampRate = 0.25;

    // Universal Motor Config
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

          // Sets Motor to Brake/Coast
          motor.setIdleMode(IdleMode.kBrake);
        });

    // TODO: Verify inversions
    leftMotorInvert = true;
    rightMotorInvert = false;

    leftMaster.setInverted(leftMotorInvert);
    rightMaster.setInverted(rightMotorInvert);

    leftMotor2.follow(leftMaster, leftMotorInvert);
    leftMotor3.follow(leftMaster, leftMotorInvert);

    rightMotor2.follow(rightMaster, rightMotorInvert);
    rightMotor3.follow(rightMaster, rightMotorInvert);

    // DiffDrive jumbalaya
    drive = new DifferentialDrive(leftMotors[0], rightMotors[0]);

    // Encoder configs
    leftEncoder = leftMaster.getEncoder();
    rightEncoder = rightMaster.getEncoder();

    leftEncoder.setPositionConversionFactor(DriveConstants.POSITION_CONVERSION_FACTOR);
    rightEncoder.setPositionConversionFactor(DriveConstants.VELOCITY_CONVERSION_FACTOR);

    leftEncoder.setInverted(leftMotorInvert);
    rightEncoder.setInverted(rightMotorInvert);

    // navX init
    navX = new AHRS(SPI.Port.kMXP);

    // Odometry init
    m_Odometry = new DifferentialDriveOdometry(navX.getRotation2d());
    field = new Field2d();
    resetOdometry(Settings.STARTING_POSITION);

    // Put field on the spot RUHEUHEUHEHE
    SmartDashboard.putData("Field", field);
  }

  // *****************************************
  // ************** Driving ******************
  // *****************************************

  public void tankDrive(double left, double right) {
    drive.tankDrive(left, right);
    drive.feed();
  }

  public void arcadeDrive(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation, false);
    drive.feed();
  }

  public void curvatureDrive(double xSpeed, double zRotation, double turnSpeed) {
    // Clamp all inputs to valid values
    xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
    zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);
    turnSpeed = MathUtil.clamp(turnSpeed, 0.0, 1.0);

    // Change turn speed based off of robot speed
    double turnAdj = Math.max(turnSpeed, Math.abs(xSpeed));

    // Find the speeds of the left and right wheels
    double leftSpeed = xSpeed + zRotation * turnAdj;
    double rightSpeed = xSpeed - zRotation * turnAdj;

    // If a wheel goes above 1.0, scale down the other wheels to compensate
    double scale = Math.max(1.0, Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed)));

    leftSpeed /= scale;
    rightSpeed /= scale;

    // Feed the inputs to the drivetrain
    tankDrive(leftSpeed, rightSpeed);
  }

  public void curvatureDrive(double xSpeed, double zRotation, boolean stationaryTurn) {
    drive.curvatureDrive(xSpeed, zRotation, stationaryTurn);
  }

  public void stopMotors() {
    drive.stopMotor();
  }

  // *****************************************
  // ************** Encoders *****************
  // *****************************************

  // TODO: Figure out if native unit conversion is needed
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

  // TODO: test that the angle stuff checks out
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
    m_Odometry.update(getRotation2d(), getLeftDistance(), getRightDistance());
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
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

  // *****************************************
  // ************** Voltage ******************
  // *****************************************

  public double getBatteryVoltage() {
    return RobotController.getBatteryVoltage();
  }

  public double getLeftVoltage() {
    return leftMotors[0].get() * getBatteryVoltage();
  }

  public double getRightVoltage() {
    return rightMotors[0].get() * getBatteryVoltage();
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    for (MotorController motor : leftMotors) {
      motor.setVoltage(leftVolts);
    }

    for (MotorController motor : rightMotors) {
      motor.setVoltage(rightVolts);
    }

    drive.feed();
  }

  @Override
  public void periodic() {
    updateOdometry();
    field.setRobotPose(getPose());
  }

  @Override
  public void initSendable(SendableBuilder builder) {}
}
