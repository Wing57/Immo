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

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;

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

    
  }

  @Override
  public void periodic() {

  }
}
