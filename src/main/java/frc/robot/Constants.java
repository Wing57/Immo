// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class Constants {

  // *****************************************
  // ********** DRIVE TRAIN ******************
  // *****************************************
  public static final class DriveConstants {
    public static final int LEFT_MOTOR_1 = 1;
    public static final int LEFT_MOTOR_2 = 2;
    public static final int LEFT_MOTOR_3 = 3;

    public static final int RIGHT_MOTOR_1 = 4;
    public static final int RIGHT_MOTOR_2 = 5;
    public static final int RIGHT_MOTOR_3 = 6;

    // TODO: Find the actual gear ratio
    public static final double kGearRatio = 15.32;
    public static final double kWheelWheelRadiusInch = 3.0;

    public static final int kEncoderResolution = 8192;

    public static final double POSITION_CONVERSION_FACTOR = ((1/kGearRatio) * (2 * Math.PI * kWheelWheelRadiusInch));
    public static final double VELOCITY_CONVERSION_FACTOR = ((1/kGearRatio) * (2 * Math.PI *kWheelWheelRadiusInch) * (1/60));

    public static final double kEncoderDPP =
        (Units.inchesToMeters(kWheelWheelRadiusInch * 2) * Math.PI) / (double) kEncoderResolution;
    
    /////////////// SYSID VALUES ///////////////

    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;
    public static final double kA_Angular = 0;
    public static final double kV_Angular = 0;

    public static final double kTrackWidthMeters = 0.7551;
  }

  public static final class Settings {
    public static final Translation2d STARTING_TRANSLATION = new Translation2d();
    public static final Rotation2d STARTING_ANGLE = new Rotation2d();

    public static final Pose2d STARTING_POSITION = new Pose2d(STARTING_TRANSLATION, STARTING_ANGLE);
  }
}
