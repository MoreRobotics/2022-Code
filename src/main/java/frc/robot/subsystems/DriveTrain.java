// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
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

  /** Creates a new ExampleSubsystem. */
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

    leftDrive.setInverted(true);       

    falconFrontLeft.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 70, 15, 0.5));
    falconFrontRight.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 70, 15, 0.5));
    falconRearLeft.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 70, 15, 0.5));
    falconRearRight.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 70, 15, 0.5));
 
    falconFrontLeft.setSensorPhase(true);
    falconFrontRight.setSensorPhase(true);

    camera = new PhotonCamera("gloworm");

    // falconFrontRight.setNeutralMode(NeutralMode.Brake);
    // falconMidRight.setNeutralMode(NeutralMode.Brake);
    // falconRearRight.setNeutralMode(NeutralMode.Brake);

    // falconFrontLeft.setNeutralMode(NeutralMode.Brake);
    // falconRearLeft.setNeutralMode(NeutralMode.Brake);
    // falconMidLeft.setNeutralMode(NeutralMode.Brake);

  }

  @Override
  public void periodic() {}

  public void drive() {
    drive.arcadeDrive(-driverController.getLeftY()*0.7, -driverController.getRightX());
  }


  public void driveForward() {
    drive.arcadeDrive(0.5, 0);
  }

  public void autoDrive() {
    var result = camera.getLatestResult();
    if (result.hasTargets()) {

      double range = PhotonUtils.calculateDistanceToTargetMeters(Constants.CAMERA_HEIGHT_METERS, Constants.TARGET_HEIGHT_METERS, Constants.CAMERA_PITCH_RADIANS, Units.degreesToRadians(result.getBestTarget().getPitch()));

      switch ((int)range) {
        case 1:
          break;
        case 2:
          break;
        case 3:
          break;
        case 4:
          break;
        case 5:
          break;
        case 6:
          break;
        case 7:
          break;
        case 8:
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

    drive.arcadeDrive(forwardSpeed*0.5, 0);
    
  }
}
