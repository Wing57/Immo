// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveTrainConstants;

public class DriveTrain extends SubsystemBase {

  private final CANSparkMax[] leftMotors;
  private final CANSparkMax[] rightMotors;

  private final CANSparkMax leftMaster;
  private final CANSparkMax rightMaster;
  
  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;

  private final DifferentialDrive drive;

  private final AHRS navX;

  public DriveTrain() {
    leftMotors = new CANSparkMax[] {
      leftMaster = new CANSparkMax(DriveConstants.LEFT_MOTOR_1, MotorType.kBrushless),
      new CANSparkMax(DriveConstants.LEFT_MOTOR_2, MotorType.kBrushless),
      new CANSparkMax(DriveConstants.LEFT_MOTOR_3, MotorType.kBrushless)
    };

    rightMotors = new CANSparkMax[] {
      rightMaster = new CANSparkMax(DriveConstants.RIGHT_MOTOR_1, MotorType.kBrushless),
      new CANSparkMax(DriveConstants.RIGHT_MOTOR_2, MotorType.kBrushless),
      new CANSparkMax(DriveConstants.RIGHT_MOTOR_3, MotorType.kBrushless)
    };

    drive = new DifferentialDrive(
      new MotorControllerGroup(leftMotors), 
      new MotorControllerGroup(rightMotors)
    );

    leftEncoder = leftMaster.getEncoder();
    rightEncoder = rightMaster.getEncoder();

    navX = new AHRS(SPI.Port.kMXP);

    
  }

  @Override
  public void periodic() {

  }
}
