// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.ArcadeDrive;

public class DriveTrain extends SubsystemBase {
  WPI_TalonFX falconFrontRight, falconRearRight, falconMidRight, falconFrontLeft, falconRearLeft, falconMidLeft;
  MotorControllerGroup rightDrive, leftDrive;
  DifferentialDrive drive;
  XboxController driverController = new XboxController(Constants.DRIVER_CONTROLLER_PORT);
  PhotonCamera camera;

  final double LINEAR_P = 0.1;
  final double LINEAR_D = 0.0;
  PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

  final double ANGULAR_P = 0.1;
  final double ANGULAR_D = 0.0;
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  double forwardSpeed;
  double rotationSpeed;

  RamseteController ramseteController;
  SimpleMotorFeedforward simpleMotorFeedforward;

  PIDController leftPIDController, rightPIDController;

  //PigeonIMU gyro;

  DifferentialDriveOdometry odometry;

  SlewRateLimiter slewRateLimiter;

  public DriveTrain() {

    falconFrontRight = new WPI_TalonFX(Constants.DRIVE_TRAIN_FRONT_RIGHT_ID);
    falconRearRight = new WPI_TalonFX(Constants.DRIVE_TRAIN_REAR_RIGHT_ID);
    // falconMidRight = new WPI_TalonFX(Constants.DRIVE_TRAIN_MID_RIGHT_ID);
    rightDrive = new MotorControllerGroup(falconFrontRight, falconRearRight/*, falconMidRight*/);
    falconFrontLeft = new WPI_TalonFX(Constants.DRIVE_TRAIN_FRONT_LEFT_ID);
    falconRearLeft = new WPI_TalonFX(Constants.DRIVE_TRAIN_REAR_LEFT_ID);
    //falconMidLeft = new WPI_TalonFX(Constants.DRIVE_TRAIN_MID_LEFT_ID);
    leftDrive = new MotorControllerGroup(falconFrontLeft, falconRearLeft/*, falconMidLeft*/);

    drive = new DifferentialDrive(rightDrive, leftDrive);     

    ramseteController = new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta);
    simpleMotorFeedforward = new SimpleMotorFeedforward(Constants.kS, Constants.kV, Constants.kA);

    leftPIDController = new PIDController(Constants.kPDriveVelLeft, 0, 0);
    rightPIDController = new PIDController(Constants.kPDriveVelRight, 0, 0);

    slewRateLimiter = new SlewRateLimiter(2.5); //CHANGED FROM 1.2

    final TalonSRX gyroMotor = new TalonSRX(Constants.INTAKE_MOTOR_ID);
    //gyro = new PigeonIMU(gyroMotor);

    rightDrive.setInverted(true);

    //zeroHeading();

   // odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

    falconFrontLeft.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 70, 15, 0.5));
    falconFrontRight.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 70, 15, 0.5));
    falconRearLeft.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 70, 15, 0.5));
    falconRearRight.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 70, 15, 0.5));
 
    falconFrontLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.kTimeoutMs);
    falconRearLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.kTimeoutMs);
    falconFrontRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.kTimeoutMs);
    falconRearRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.kTimeoutMs);

    falconFrontRight.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
    falconFrontLeft.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
    falconRearRight.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
    falconRearLeft.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);

    // falconFrontLeft.setSensorPhase(true);
    // falconFrontRight.setSensorPhase(true);

    camera = new PhotonCamera("gloworm");

    falconFrontRight.setNeutralMode(NeutralMode.Brake);
    falconRearRight.setNeutralMode(NeutralMode.Brake);

    falconFrontLeft.setNeutralMode(NeutralMode.Brake);
    falconRearLeft.setNeutralMode(NeutralMode.Brake);
    
  }

  @Override
  public void periodic() {
    // odometry.update(
    //   Rotation2d.fromDegrees(getHeading()),
    //   victorFrontLeft.getSelectedSensorPosition() / Constants.EDGES_PER_REVOLUTION * Constants.WHEEL_CIRCUMFERENCE,
    //   victorFrontRight.getSelectedSensorPosition() / Constants.EDGES_PER_REVOLUTION *  Constants.WHEEL_CIRCUMFERENCE);
    // SmartDashboard.putNumber("Compass Heading", getHeading());
  }

  public void drive() {
    //double speed = Math.pow(driverController.getLeftY(), 2);
    // System.out.println(slewRateLimiter.calculate(driverController.getLeftY()));
    // System.out.println(driverController.getRightX()*0.7);
    drive.arcadeDrive(slewRateLimiter.calculate(driverController.getLeftY()), driverController.getRightX());
  }

  public void setCoast() {
    falconFrontRight.setNeutralMode(NeutralMode.Coast);
    falconRearRight.setNeutralMode(NeutralMode.Coast);

    falconFrontLeft.setNeutralMode(NeutralMode.Coast);
    falconRearLeft.setNeutralMode(NeutralMode.Coast);
  }

  // public double getHeading() {
  //   // double[] ypr = new double[3];
  //   // gyro.getYawPitchRoll(ypr);
  //   // //System.out.println("Yaw " + ypr[0]);
  //   // return Math.IEEEremainder(ypr[0], 360);
  // }

  // public void zeroHeading() {
    //gyro.setYaw(0);
  // }

  public void setPose(Trajectory path) {
    falconFrontRight.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
    falconFrontLeft.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
    falconRearRight.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
    falconRearLeft.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
    System.out.println("FalconRearLeft: " + falconRearLeft.getSelectedSensorPosition(0));
    System.out.println("FalconRearRight: " + falconRearRight.getSelectedSensorPosition(0));
    System.out.println("FalconFrontLeft: " + falconFrontLeft.getSelectedSensorPosition(0));
    System.out.println("FalconFrontRight: " + falconFrontRight.getSelectedSensorPosition(0));
    
    // odometry.resetPosition(path.getInitialPose(), Rotation2d.fromDegrees(getHeading()));
    // System.out.println("Pose " + odometry.getPoseMeters());
  }

  public void driveForward() {
    drive.arcadeDrive(-0.5, 0); // LOOK INTO WHAT THIS DOES
  }

  public RamseteCommand getRamseteCommand(Trajectory trajectory) {
    return new RamseteCommand(
      trajectory, 
      this::getPose, 
      ramseteController, 
      simpleMotorFeedforward,
      Constants.kDriveKinematics,
      this::getWheelSpeeds,
      leftPIDController,
      rightPIDController,
      this::tankDriveVolts,
      this);
  }

  public void tankDriveVolts(final double leftVolts, final double rightVolts) {
    leftDrive.setVoltage(-leftVolts);
    rightDrive.setVoltage(-rightVolts);
    drive.feed();
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      falconFrontLeft.getSelectedSensorVelocity() / 10 / Constants.EDGES_PER_REVOLUTION * Constants.WHEEL_CIRCUMFERENCE,
      falconFrontRight.getSelectedSensorVelocity() / 10 / Constants.EDGES_PER_REVOLUTION * Constants.WHEEL_CIRCUMFERENCE);
  }

}
