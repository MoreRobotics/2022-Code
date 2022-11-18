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
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
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
  WPI_VictorSPX victorFrontRight, victorRearRight, victorMidRight, victorFrontLeft, victorRearLeft, victorMidLeft;
  MotorControllerGroup rightDrive, leftDrive;
  DifferentialDrive drive;
  XboxController driverController = new XboxController(Constants.DRIVER_CONTROLLER_PORT);
  //PhotonCamera camera;

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
    
    victorFrontRight = new WPI_VictorSPX(Constants.DRIVE_TRAIN_FRONT_RIGHT_ID);
    victorRearRight = new WPI_VictorSPX(Constants.DRIVE_TRAIN_REAR_RIGHT_ID);
    // victorMidRight = new WPI_VictorSPX(Constants.DRIVE_TRAIN_MID_RIGHT_ID);
    rightDrive = new MotorControllerGroup(victorFrontRight/*, victorRearRight/*, victorMidRight*/);
    victorFrontLeft = new WPI_VictorSPX(Constants.DRIVE_TRAIN_FRONT_LEFT_ID);
    victorRearLeft = new WPI_VictorSPX(Constants.DRIVE_TRAIN_REAR_LEFT_ID);
    //victorMidLeft = new WPI_VictorSPX(Constants.DRIVE_TRAIN_MID_LEFT_ID);
    leftDrive = new MotorControllerGroup(victorFrontLeft, victorRearLeft/*, victorMidLeft*/);

    drive = new DifferentialDrive(rightDrive, leftDrive);     

    ramseteController = new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta);
    simpleMotorFeedforward = new SimpleMotorFeedforward(Constants.kS, Constants.kV, Constants.kA);

    leftPIDController = new PIDController(Constants.kPDriveVelLeft, 0, 0);
    rightPIDController = new PIDController(Constants.kPDriveVelRight, 0, 0);

    slewRateLimiter = new SlewRateLimiter(2.5); //CHANGED FROM 1.2

    //final WPI_VictorSPX gyroMotor = new WPI_VictorSPX(Constants.INTAKE_MOTOR_ID);
    //gyro = new PigeonIMU(gyroMotor);

    leftDrive.setInverted(true);

   // zeroHeading();

   // odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

    // victorFrontLeft.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 70, 15, 0.5));
    // victorFrontRight.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 70, 15, 0.5));
    // victorRearLeft.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 70, 15, 0.5));
    // victorRearRight.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 70, 15, 0.5));
 
    victorFrontLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.kTimeoutMs);
    victorRearLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.kTimeoutMs);
    victorFrontRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.kTimeoutMs);
    victorRearRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.kTimeoutMs);

    victorFrontRight.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
    victorFrontLeft.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
    victorRearRight.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
    victorRearLeft.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);

    // victorFrontLeft.setSensorPhase(true);
    // victorFrontRight.setSensorPhase(true);

   // camera = new PhotonCamera("gloworm");

    victorFrontRight.setNeutralMode(NeutralMode.Brake);
    victorRearRight.setNeutralMode(NeutralMode.Brake);

    victorFrontLeft.setNeutralMode(NeutralMode.Brake);
    victorRearLeft.setNeutralMode(NeutralMode.Brake);
    
  }

  @Override
  public void periodic() {
    /*odometry.update(
      Rotation2d.fromDegrees(getHeading()),
      victorFrontLeft.getSelectedSensorPosition() / Constants.EDGES_PER_REVOLUTION * Constants.WHEEL_CIRCUMFERENCE,
      victorFrontRight.getSelectedSensorPosition() / Constants.EDGES_PER_REVOLUTION *  Constants.WHEEL_CIRCUMFERENCE);
    SmartDashboard.putNumber("Compass Heading", getHeading());*/

  }

  public void drive() {
    //double speed = Math.pow(driverController.getLeftY(), 2);
    // System.out.println(slewRateLimiter.calculate(driverController.getLeftY()));
    // System.out.println(driverController.getRightX()*0.7);
    //drive.arcadeDrive(Constants.DRIVE_SPEED*driverController.getLeftY(), Constants.DRIVE_SPEED*driverController.getRightX());
  }

  public void setCoast() {
    victorFrontRight.setNeutralMode(NeutralMode.Coast);
    victorRearRight.setNeutralMode(NeutralMode.Coast);

    victorFrontLeft.setNeutralMode(NeutralMode.Coast);
    victorRearLeft.setNeutralMode(NeutralMode.Coast);
  }

 // public double getHeading() {
  //  double[] ypr = new double[3];
  //  gyro.getYawPitchRoll(ypr);
    //System.out.println("Yaw " + ypr[0]);
  //  return Math.IEEEremainder(ypr[0], 360);
 //}

 // public void zeroHeading() {
  //  gyro.setYaw(0);
  //}

  public void setPose(Trajectory path) {
    victorFrontRight.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
    victorFrontLeft.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
    victorRearRight.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
    victorRearLeft.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
    System.out.println("victorRearLeft: " + victorRearLeft.getSelectedSensorPosition(0));
    System.out.println("victorRearRight: " + victorRearRight.getSelectedSensorPosition(0));
    System.out.println("victorFrontLeft: " + victorFrontLeft.getSelectedSensorPosition(0));
    System.out.println("victorFrontRight: " + victorFrontRight.getSelectedSensorPosition(0));
    
    // odometry.resetPosition(path.getInitialPose(), Rotation2d.fromDegrees(getHeading()));
    System.out.println("Pose " + odometry.getPoseMeters());
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
      victorFrontLeft.getSelectedSensorVelocity() / 10 / Constants.EDGES_PER_REVOLUTION * Constants.WHEEL_CIRCUMFERENCE,
      victorFrontRight.getSelectedSensorVelocity() / 10 / Constants.EDGES_PER_REVOLUTION * Constants.WHEEL_CIRCUMFERENCE);
  }

}
