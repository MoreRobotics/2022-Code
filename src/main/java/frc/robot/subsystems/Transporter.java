// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.opencv.features2d.FlannBasedMatcher;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Transporter extends SubsystemBase {
  /** Creates a new Transporter. */
  TalonSRX transporterMotorLeft, transporterMotorRight, towerMotor;
  
  XboxController operatorController;

  public Transporter() {
    transporterMotorLeft = new TalonSRX(Constants.TRANSPORTER_MOTOR_LEFT);
    transporterMotorRight = new TalonSRX(Constants.TRANSPORTER_MOTOR_RIGHT);
    towerMotor = new TalonSRX(Constants.TOWER_MOTOR_ID);
    operatorController = new XboxController(Constants.OPERATOR_CONTROLLER_PORT);
    transporterMotorRight.setInverted(true);
    transporterMotorLeft.setInverted(true);
    towerMotor.setInverted(false);
    transporterMotorRight.follow(transporterMotorLeft);
 
    transporterMotorLeft.configPeakCurrentLimit(20);
    transporterMotorRight.configPeakCurrentLimit(20);

    transporterMotorLeft.configPeakCurrentDuration(100);
    transporterMotorRight.configPeakCurrentDuration(100);

    transporterMotorLeft.configContinuousCurrentLimit(20);
    transporterMotorRight.configContinuousCurrentLimit(20);

    transporterMotorLeft.enableCurrentLimit(true);
    transporterMotorRight.enableCurrentLimit(true);

  }

  public void startTowerTransporter() {
    towerMotor.set(ControlMode.PercentOutput, Constants.TRANSPORTER_SPEED);
  }

  public void stopTowerTransporter() {
    towerMotor.set(ControlMode.PercentOutput, 0);
  }

  public void reverseTowerTransporter() {
    towerMotor.set(ControlMode.PercentOutput, -Constants.TRANSPORTER_SPEED);
  }

  public void startTransporter() {
    transporterMotorLeft.set(ControlMode.PercentOutput, Constants.TRANSPORTER_SPEED);
  }

  public void stopTransporter() {
    transporterMotorLeft.set(ControlMode.PercentOutput, 0);
  }

  public void reverseTransporter() {
    transporterMotorLeft.set(ControlMode.PercentOutput, -Constants.TRANSPORTER_SPEED);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
