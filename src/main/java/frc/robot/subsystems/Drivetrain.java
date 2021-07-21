// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  
  //found on robot
  public static final double TRACK_WIDTH_M = 0.5883D;
  public static final double GEAR_RATIO = 11.2444D;
  public static final double WHEEL_DIAMETER_M = 0.158D;
  public static final double WHEEL_CIRCUMFERENCE_M = WHEEL_DIAMETER_M * Math.PI;
  
  public static final int TICKS_PER_ROTATION = 2048;

  public static final int MAX_VOLTAGE = 10;
  public static final int TIMEOUT_MS = 10;

  //found through frc-characterization on robot
  public static final double DRIVE_S = 0.166D;
  public static final double DRIVE_V = 2.41D;
  public static final double DRIVE_A = 0.25D;

  public static final double MAX_VELOCITY_M = 1.5D;
  public static final double MAX_ACCELERATION_M = 2.0D;

  public static final double kP = 0.0D, kI = 0.0D, kD = 0.0D;

  private TalonFX leftMaster, leftFollower, rightMaster, rightFollower;

  private AHRS gyro;

  private DifferentialDriveKinematics kinematics;
  private DifferentialDriveOdometry odometry;
  private SimpleMotorFeedforward feedforward;

  private PIDController leftController, rightController;

  private TrajectoryConfig trajectoryConfig;

  private Pose2d currentPos;

  public Drivetrain() 
  {
    leftMaster = new TalonFX(2);
    leftFollower = new TalonFX(1);
    rightMaster = new TalonFX(3);
    rightFollower = new TalonFX(5);

    leftMaster.configFactoryDefault();
    leftFollower.configFactoryDefault();
    rightMaster.configFactoryDefault();
    rightFollower.configFactoryDefault();

    leftFollower.follow(leftMaster);
    rightFollower.follow(rightMaster);

    leftMaster.setInverted(true);
    rightMaster.setInverted(false);

    leftMaster.configVoltageCompSaturation(MAX_VOLTAGE, TIMEOUT_MS);
    rightMaster.configVoltageCompSaturation(MAX_VOLTAGE, TIMEOUT_MS);
    leftMaster.enableVoltageCompensation(true);
    rightMaster.enableVoltageCompensation(true);

    gyro = new AHRS(SPI.Port.kMXP);

    kinematics = new DifferentialDriveKinematics(TRACK_WIDTH_M);
    odometry = new DifferentialDriveOdometry(getAngle());
    feedforward = new SimpleMotorFeedforward(DRIVE_S, DRIVE_V, DRIVE_A);
    
    leftController = new PIDController(kP, kI, kD);
    rightController = new PIDController(kP, kI, kD);

    trajectoryConfig = new TrajectoryConfig(MAX_VELOCITY_M, MAX_ACCELERATION_M);
  }

  public void setOutputVoltage(double leftVolts, double rightVolts)
  {
    leftMaster.set(ControlMode.PercentOutput, leftVolts / MAX_VOLTAGE);
    rightMaster.set(ControlMode.PercentOutput, rightVolts / MAX_VOLTAGE);
  }

  public DifferentialDriveWheelSpeeds getSpeeds()
  {
    return new DifferentialDriveWheelSpeeds
    (
      ticksToMetersPerS(leftMaster.getSelectedSensorVelocity()),
      ticksToMetersPerS(rightMaster.getSelectedSensorVelocity())
    );
  }

  public double getLeftDistanceM()
  {
    double ticks = (leftMaster.getSelectedSensorPosition() + leftFollower.getSelectedSensorPosition())/2;

    return ticksToMeters(ticks);
  }

  public double getRightDistanceM()
  {
    double ticks = (leftMaster.getSelectedSensorPosition() + leftFollower.getSelectedSensorPosition())/2;

    return ticksToMeters(ticks);
  }

  public double ticksToMeters(double ticks)
  {
    return ticks / GEAR_RATIO / TICKS_PER_ROTATION * WHEEL_CIRCUMFERENCE_M;
  }

  public double ticksToMetersPerS(double ticksPer100MS)
  {
    return ticksToMeters(ticksPer100MS) * 1000 / 100;
  }

  public Rotation2d getAngle()
  {
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }

  public TrajectoryConfig getTrajectoryConfig() 
  {
    return trajectoryConfig;
  }

  public PIDController getLeftController() 
  {
    return leftController;
  }

  public PIDController getRightController()
  {
    return rightController;
  }

  public SimpleMotorFeedforward getFeedforward() 
  {
    return feedforward;
  }

  public DifferentialDriveKinematics getKinematics() 
  {
    return kinematics;
  }

  public DifferentialDriveOdometry getOdometry() 
  {
    return odometry;
  }

  public Pose2d getCurrentPos() 
  {
    return currentPos;
  }

  public void log()
  {
    SmartDashboard.putNumber("Left Output", leftMaster.getMotorOutputPercent());
    SmartDashboard.putNumber("Right Output", rightMaster.getMotorOutputPercent());
    SmartDashboard.putNumber("Direction", -gyro.getAngle());
  }

  @Override
  public void periodic() 
  {
    super.periodic();
    currentPos = odometry.update(getAngle(), getLeftDistanceM(), getRightDistanceM());
    log();
  }
}
