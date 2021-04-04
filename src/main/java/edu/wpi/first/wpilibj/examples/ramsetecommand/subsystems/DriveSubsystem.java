package edu.wpi.first.wpilibj.examples.ramsetecommand.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class DriveSubsystem extends SubsystemBase {

  private final WPI_TalonFX leftmotor = new WPI_TalonFX(3);
  private final WPI_TalonFX rightmotor = new WPI_TalonFX(1);

  // The motors on the left side of the drive.
  private final SpeedControllerGroup m_leftMotors =
      new SpeedControllerGroup(new WPI_TalonFX(DriveConstants.kLeftMotor1Port),
                               new WPI_TalonFX(DriveConstants.kLeftMotor2Port));

  // The motors on the right side of the drive.
  private final SpeedControllerGroup m_rightMotors =
      new SpeedControllerGroup(new WPI_TalonFX(DriveConstants.kRightMotor1Port),
                               new WPI_TalonFX(DriveConstants.kRightMotor2Port));

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  // The gyro sensor
  public static final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(m_gyro.getRotation2d(), getLeftDistance(),
                      getRightDistance());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  //TODO Change kUnitsPerRevolution?
  final int kUnitsPerRevolution = 2048; /* this is constant for Talon FX */
  double selSenPos = leftmotor.getSelectedSensorPosition(); /* position units */
  double selSenVel = leftmotor.getSelectedSensorVelocity(); /* position units per 100ms */
  double vel_RotPerSec = (double) selSenVel / kUnitsPerRevolution * 10; /* scale per100ms to perSecond */
  double vel_RotPerMin = vel_RotPerSec * 60.0;
  double vel_mpersleft = (vel_RotPerMin * 0.475) / 60;

  double selSenPosr = rightmotor.getSelectedSensorPosition(); /* position units */
  double selSenVelr = rightmotor.getSelectedSensorVelocity(); /* position units per 100ms */
  double vel_RotPerSecr = (double) selSenVelr / kUnitsPerRevolution * 10; /* scale per100ms to perSecond */
  double vel_RotPerMinr = vel_RotPerSecr * 60.0;
  double vel_mpersright = (vel_RotPerMinr * 0.475) / 60;
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(vel_mpersleft, vel_mpersright);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(-rightVolts);
    m_drive.feed();
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    rightmotor.setSelectedSensorPosition(0);
    leftmotor.setSelectedSensorPosition(0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
    // This double method returns the distance travelled by the left side of the drivetrain
    public double getLeftDistance () {
      return 0.475 * (leftmotor.getSelectedSensorPosition() / 20992);
    }
  
    // This double method returns the distance travelled by the right side of the drivetrain
    public double getRightDistance () {
      return 0.475 * (rightmotor.getSelectedSensorPosition() / 20992);
    }



  public double getAverageEncoderDistance() {
    return (getLeftDistance() + getRightDistance()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public Double getLeftEncoder() {
    return leftmotor.getSelectedSensorPosition();
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public Double getRightEncoder() {
    return rightmotor.getSelectedSensorPosition();
  }

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }
}