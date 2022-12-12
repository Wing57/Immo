package frc.robot.subsystems.driveTrain;

import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.IDriveTrain;

public class DriveTrainSim extends IDriveTrain {

  private final Encoder leftEncoder, rightEncoder;
  private EncoderSim leftEncoderSim, rightEncoderSim;

  private final ADXRS450_Gyro m_gyro;
  private ADXRS450_GyroSim m_GyroSim;

  public DifferentialDrivetrainSim driveSim;

  private Field2d fieldSim;

  public DriveTrainSim() {
    leftEncoder = new Encoder(0, 1, true);
    rightEncoder = new Encoder(2, 3, false);

    leftEncoder.reset();
    rightEncoder.reset();

    leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDPP);
    rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDPP);


  }
    
}
