// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.TankDrive;

public class DriveTrain extends SubsystemBase {
  WPI_TalonFX falconFrontRight, falconRearRight, falconFrontLeft, falconRearLeft;
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

  PIDController leftPIDController, rightPIDController;

  RamseteController ramseteController;

  SimpleMotorFeedforward simpleMotorFeedForward;

  double forwardSpeed;
  double rotationSpeed;

  /** Creates a new ExampleSubsystem. */
  public DriveTrain() {

    falconFrontRight = new WPI_TalonFX(Constants.DRIVE_TRAIN_FRONT_RIGHT_ID);
    falconRearRight = new WPI_TalonFX(Constants.DRIVE_TRAIN_REAR_RIGHT_ID);
    rightDrive = new MotorControllerGroup(falconFrontRight, falconRearRight);
    falconFrontLeft = new WPI_TalonFX(Constants.DRIVE_TRAIN_FRONT_LEFT_ID);
    falconRearLeft = new WPI_TalonFX(Constants.DRIVE_TRAIN_REAR_LEFT_ID);
    leftDrive = new MotorControllerGroup(falconFrontLeft, falconRearLeft);
    drive = new DifferentialDrive(rightDrive, leftDrive);
    this.setDefaultCommand(new TankDrive(this));

    falconFrontLeft.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 70, 15, 0.5));
    falconFrontRight.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 70, 15, 0.5));
    falconRearLeft.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 70, 15, 0.5));
    falconRearRight.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 70, 15, 0.5));
 
    falconFrontLeft.setSensorPhase(true);
    falconFrontRight.setSensorPhase(true);

    camera = new PhotonCamera("photonvision");

    ramseteController = new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta);
    simpleMotorFeedForward = new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter);

    leftPIDController = new PIDController(Constants.kPDriveVel, 0, 0);
    rightPIDController = new PIDController(Constants.kPDriveVel, 0, 0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    
  }

  public void drive(){
    drive.arcadeDrive(driverController.getRightX(), driverController.getLeftY());
  }

  public void autoDrive() {
    var result = camera.getLatestResult();
    System.out.println(result.hasTargets());
    if (result.hasTargets()) {

      double range = PhotonUtils.calculateDistanceToTargetMeters(Constants.CAMERA_HEIGHT_METERS, Constants.TARGET_HEIGHT_METERS, Constants.CAMERA_PITCH_RADIANS, Units.degreesToRadians(result.getBestTarget().getPitch()));

      

      switch ((int)range) {
        case Constants.RANGE1 - 2:
        case Constants.RANGE1 - 1:
        case Constants.RANGE1:
        case Constants.RANGE1 + 1:
        case Constants.RANGE1 + 2:
          break;
        case Constants.RANGE2 - 2:
        case Constants.RANGE2 - 1:
        case Constants.RANGE2:
        case Constants.RANGE2 + 1:
        case Constants.RANGE2 + 2:
          break;
        case Constants.RANGE3 - 2:
        case Constants.RANGE3 - 1:
        case Constants.RANGE3:
        case Constants.RANGE3 + 1:
        case Constants.RANGE3 + 2:
          break;
        case Constants.RANGE4 - 2:
        case Constants.RANGE4 - 1:
        case Constants.RANGE4:
        case Constants.RANGE4 + 1:
        case Constants.RANGE4 + 2:
          break;
        default:
          break;

      }

      forwardSpeed = forwardController.calculate(range, Constants.GOAL_RANGE_METERS);

      // Calculate angular turn power
      // -1.0 required to ensure positive PID controller effort _increases_ yaw
      rotationSpeed = turnController.calculate(result.getBestTarget().getYaw(), 0);
    } else {
      // If we have no targets, stay still.
      rotationSpeed = 0;
      forwardSpeed = 0;
    }

    drive.arcadeDrive(forwardSpeed*0.5, rotationSpeed*0.5);
    
  }

  public RamseteCommand getRamseteCommand(Trajectory trajectory) {
    return new RamseteCommand(
      trajectory, 
      this::getPose, 
      ramseteController, 
      simpleMotorFeedForward,
      Constants.kDriveKinematics,
      this::getWheelSpeeds,
      leftPIDController,
      rightPIDController,
      this::tankDriveVolts,
      this);
  }

  public void tankDriveVolts(final double leftVolts, final double rightVolts) {
    leftDrive.setVoltage(leftVolts);
    rightDrive.setVoltage(-rightVolts);
    drive.feed();
  }

  public Pose2d getPose() {
    return new Pose2d();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(falconRearLeft.getSelectedSensorVelocity() / Constants.EDGES_PER_REVOLUTION * Constants.WHEEL_DIAMETER * Math.PI,
    falconFrontRight.getSelectedSensorVelocity() / Constants.EDGES_PER_REVOLUTION * Constants.WHEEL_DIAMETER * Math.PI);
  }
}
