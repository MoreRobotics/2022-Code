// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Turret extends SubsystemBase {

  TalonSRX turretMotor;
  XboxController operatorController;
  TalonSRXFeedbackDevice encoder;

  final double ANGULAR_P = 0.1;
  final double ANGULAR_D = 0.0;
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  double targetYaw;
  double unitsDisplacement;
  double targetPosition;

  PhotonCamera camera;

  /** Creates a new Turret. */
  public Turret() {

    turretMotor = new TalonSRX(Constants.TURRET_MOTOR_ID);
    operatorController = new XboxController(Constants.OPERATOR_CONTROLLER_PORT);
    turretMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute, 0, Constants.kTimeoutMs);

    turretMotor.config_kF(0, Constants.kGains_Turret_Velocity.kF, Constants.kTimeoutMs);
    turretMotor.config_kP(0, Constants.kGains_Turret_Velocity.kP, Constants.kTimeoutMs);
    turretMotor.config_kI(0, Constants.kGains_Turret_Velocity.kI, Constants.kTimeoutMs);
    turretMotor.config_kD(0, Constants.kGains_Turret_Velocity.kD, Constants.kTimeoutMs);

    camera = new PhotonCamera("gloworm");
    
  }

  public void turnTurret() {
    var result = camera.getLatestResult();
    if(result.hasTargets()) {
      //Unsure if this works
      targetYaw = result.getBestTarget().getYaw();
      unitsDisplacement = targetYaw * Constants.TURRET_DEGREES_TO_ENCODER;

      targetPosition = turretMotor.getSelectedSensorPosition() + Constants.TURRET_OFFSET + unitsDisplacement;

      if(targetPosition > 4095) {
        targetPosition = 4095;
      } 
      if(targetPosition < 0) {
        targetPosition = 0;
      }
      

    } else {
      targetYaw = 0;
      unitsDisplacement = 0;
      targetPosition = 2048;
    }
    //TODO: test
    turretMotor.set(ControlMode.Position, (targetPosition - Constants.TURRET_OFFSET) % 4096);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
