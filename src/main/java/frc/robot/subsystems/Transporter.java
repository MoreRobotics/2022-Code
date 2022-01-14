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
  TalonSRX transporterMotor;
  XboxController operatorController;

  public Transporter() {
    transporterMotor = new TalonSRX(Constants.TRANSPORTER_MOTOR_ID);
    operatorController = new XboxController(Constants.OPERATOR_CONTROLLER_PORT);
  }

  public void startTransporter() {
    transporterMotor.set(ControlMode.PercentOutput, Constants.TRANSPORTER_SPEED);
  }

  public void stopTransporter() {
    transporterMotor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
