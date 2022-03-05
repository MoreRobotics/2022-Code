// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Turret extends SubsystemBase {

  TalonSRX turretMotor;
  XboxController operatorController;

  final double ANGULAR_P = 0.1;
  final double ANGULAR_D = 0.0;
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  double targetYaw;
  double unitsDisplacement;
  double targetPosition;

  public PhotonCamera camera;

  /** Creates a new Turret. */
  public Turret() {

    turretMotor = new TalonSRX(Constants.TURRET_MOTOR_ID);
    operatorController = new XboxController(Constants.OPERATOR_CONTROLLER_PORT);
    turretMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute, 0, Constants.kTimeoutMs);

    turretMotor.config_kF(0, Constants.kGains_Turret_Velocity.kF, Constants.kTimeoutMs);
    turretMotor.config_kP(0, Constants.kGains_Turret_Velocity.kP, Constants.kTimeoutMs);
    turretMotor.config_kI(0, Constants.kGains_Turret_Velocity.kI, Constants.kTimeoutMs);
    turretMotor.config_kD(0, Constants.kGains_Turret_Velocity.kD, Constants.kTimeoutMs);

    turretMotor.configForwardSoftLimitThreshold(Constants.TURRET_MAX_ENCODER_UNITS + Constants.TURRET_OFFSET, Constants.kTimeoutMs);
    turretMotor.configReverseSoftLimitThreshold(Constants.TURRET_OFFSET, Constants.kTimeoutMs);
    turretMotor.configForwardSoftLimitEnable(false, Constants.kTimeoutMs);
    turretMotor.configReverseSoftLimitEnable(false, Constants.kTimeoutMs);

    turretMotor.setNeutralMode(NeutralMode.Brake);

    camera = new PhotonCamera("gloworm");
    //camera.setLED(VisionLEDMode.kOff);
    
  }

  public double getTurretOffset() {
    return SmartDashboard.getNumber("Turret Offset", Constants.TURRET_OFFSET);
  }

  public double getTurretPos() {

    return turretMotor.getSelectedSensorPosition();
  }

  public void turnTurret() {
    var result = camera.getLatestResult();
    if(result.hasTargets()) {
      //Unsure if this works
      targetYaw = result.getBestTarget().getYaw();
      unitsDisplacement = targetYaw * Constants.TURRET_DEGREES_TO_ENCODER;

      targetPosition = turretMotor.getSelectedSensorPosition() - getTurretOffset() + unitsDisplacement;

      if(targetPosition > Constants.TURRET_MAX_ENCODER_UNITS) {
        targetPosition = Constants.TURRET_MAX_ENCODER_UNITS;
      } 
      if(targetPosition < Constants.TURRET_MIN_ENCODER_UNITS) {
        targetPosition = Constants.TURRET_MIN_ENCODER_UNITS;
      }
      

    } else {
      targetYaw = 0;
      unitsDisplacement = 0;
      targetPosition = Constants.TURRET_UP_POSITION;
    }
    //TODO: test
    turretMotor.set(ControlMode.Position, targetPosition + getTurretOffset());
  }

  public void percentTurretLeft() {
    turretMotor.set(ControlMode.PercentOutput, -0.3);
  }

  public void percentTurretRight() {
    turretMotor.set(ControlMode.PercentOutput, 0.3);
  }

  public void setTurretPos(int pos) {
    turretMotor.set(ControlMode.Position, pos + getTurretOffset());
  }

  public void stopTurret() {
    turretMotor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
