// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Transporter extends SubsystemBase {
  /** Creates a new Transporter. */
  TalonSRX transporterLeftMotor, transporterRightMotor, towerMotor;
  
  XboxController operatorController;

  public Transporter() {
    towerMotor = new TalonSRX(Constants.TOWER_MOTOR_ID);
    transporterLeftMotor = new TalonSRX(Constants.TRANSPORTER_LEFT_MOTOR);
    transporterRightMotor = new TalonSRX(Constants.TRANSPORTER_RIGHT_MOTOR);
    operatorController = new XboxController(Constants.OPERATOR_CONTROLLER_PORT);
  }

  public void startTowerTransporter() {
    towerMotor.set(ControlMode.PercentOutput, -Constants.TRANSPORTER_SPEED);
  }

  public void stopTowerTransporter() {
    towerMotor.set(ControlMode.PercentOutput, 0);
  }

  public void reverseTowerTransporter() {
    towerMotor.set(ControlMode.PercentOutput, Constants.TRANSPORTER_SPEED);
  }

  public void startTransporter() {
    transporterLeftMotor.set(ControlMode.PercentOutput, Constants.TRANSPORTER_SPEED);
    transporterRightMotor.set(ControlMode.PercentOutput, Constants.TRANSPORTER_SPEED);
  }

  public void stopTransporter() {
    transporterLeftMotor.set(ControlMode.PercentOutput, 0);
    transporterRightMotor.set(ControlMode.PercentOutput, 0);
  }

  public void reverseTransporter() {
    transporterLeftMotor.set(ControlMode.PercentOutput, Constants.TRANSPORTER_SPEED);
    transporterRightMotor.set(ControlMode.PercentOutput, Constants.TRANSPORTER_SPEED);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
