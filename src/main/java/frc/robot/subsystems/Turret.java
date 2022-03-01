// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Turret extends SubsystemBase {

  TalonSRX turretMotor;
  XboxController operatorController;

  //camera shit goes here
  PhotonCamera camera = new PhotonCamera("photonvision");

  /** Creates a new Turret. */
  public Turret() {

    turretMotor = new TalonSRX(Constants.TURRET_MOTOR_ID);
    operatorController = new XboxController(Constants.OPERATOR_CONTROLLER_PORT);
    
    
  }

  public void TurnTurret() {
    
    var result = camera.getLatestResult();

  }

  //temp method to manually turn turret
  public void MoveTurretLeft() {
    turretMotor.set(ControlMode.PercentOutput, 0.1);
  }

  //temp method to manually turn turret
  public void MoveTurretRight() {
    turretMotor.set(ControlMode.PercentOutput, -0.1);
  }

  //temp method to manually turn turret
  public void StopTurret() {
    turretMotor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
