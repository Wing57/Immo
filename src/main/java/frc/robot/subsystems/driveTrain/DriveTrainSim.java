package frc.robot.subsystems.driveTrain;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Settings;
import frc.robot.subsystems.IDriveTrain;

public class DriveTrainSim extends IDriveTrain {

  private final Encoder leftEncoder, rightEncoder;
  private EncoderSim leftEncoderSim, rightEncoderSim;

  private final ADXRS450_Gyro m_gyro;
  private ADXRS450_GyroSim m_GyroSim;

  public DifferentialDrivetrainSim driveSim;

  private Field2d fieldSim;

  private final CANSparkMax LeftMotor, rightMotor;
  
  private final DifferentialDriveOdometry m_Odometry;
  private final DifferentialDrive drive;

  public DriveTrainSim() {
    leftEncoder = new Encoder(0, 1, true);
    rightEncoder = new Encoder(2, 3, false);

    leftEncoder.reset();
    rightEncoder.reset();

    leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDPP);
    rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDPP);

    m_gyro = new ADXRS450_Gyro();

    LeftMotor = new CANSparkMax(0, MotorType.kBrushless);
    rightMotor = new CANSparkMax(1, MotorType.kBrushless);

    drive = new DifferentialDrive(LeftMotor, rightMotor);

    if (RobotBase.isSimulation()) {
      driveSim = new DifferentialDrivetrainSim(
        LinearSystemId.identifyDrivetrainSystem(
          DriveConstants.kV,
          DriveConstants.kA,
          DriveConstants.kV_Angular, 
          DriveConstants.kA_Angular), 
          DCMotor.getNEO(3), 
          DriveConstants.kGearRatio, 
          DriveConstants.kTrackWidthMeters, 
          Units.inchesToMeters(DriveConstants.kWheelWheelRadiusInch), 
          VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

      leftEncoderSim = new EncoderSim(leftEncoder);
      rightEncoderSim = new EncoderSim(rightEncoder);
      m_GyroSim = new ADXRS450_GyroSim(m_gyro);

      fieldSim = new Field2d();
      SmartDashboard.putData("Field Sim", fieldSim);
    }

    m_Odometry = new DifferentialDriveOdometry(getRotation2d());
    resetOdometry(Settings.STARTING_POSITION);
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

  public double getLeftDistance() {
    return leftEncoder.getDistance();
  }

  public double getRightDistance() {
    return rightEncoder.getDistance();
  }

  public double getDistance() {
    return (getLeftDistance() + getRightDistance()) / 2.0;
  }

  public double getLeftVelocity() {
    return leftEncoder.getRate();
  }

  public double getRightVelocity() {
    return rightEncoder.getRate();
  }

  public double getVelocity() {
    return (getLeftVelocity() + getRightVelocity()) / 2.0;
  }

  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  // *****************************************
  // ************* Robot Angle ***************
  // *****************************************

  public double getGyroAngle() {
    return m_gyro.getAngle() % 360;
  }

  public Rotation2d getRotation2d() {
    return m_gyro.getRotation2d();
  }

  public void zeroHeading() {
    m_gyro.reset();
  }

  // ********************************************
  // ************ Odometry Functions ************
  // ********************************************

  public void updateOdometry() {
    m_Odometry.update(getRotation2d(), getLeftDistance(), getRightDistance());
  }

  public void resetOdometry(Pose2d pose2d) {
    resetEncoders();
    zeroHeading();
    m_Odometry.resetPosition(pose2d, getRotation2d());
  }

  public Field2d getField() {
    return fieldSim;
  }

  public Pose2d getPose() {
    updateOdometry();
    return m_Odometry.getPoseMeters();
  }

  @Override
  public void periodic() {
    updateOdometry();
    fieldSim.setRobotPose(getPose());

    SmartDashboard.putNumber("dd", getPose().getX());
  }

  @Override
  public void simulationPeriodic() {
    driveSim.setInputs(
      LeftMotor.get() * RobotController.getInputVoltage(), 
      rightMotor.get() * RobotController.getInputVoltage()
      );
    driveSim.update(0.02);
    
  leftEncoderSim.setDistance(driveSim.getLeftPositionMeters());
  leftEncoderSim.setRate(driveSim.getLeftVelocityMetersPerSecond());
  rightEncoderSim.setDistance(driveSim.getRightPositionMeters());
  rightEncoderSim.setRate(driveSim.getRightVelocityMetersPerSecond());
  m_GyroSim.setAngle(-driveSim.getHeading().getDegrees());
  }
  
}
