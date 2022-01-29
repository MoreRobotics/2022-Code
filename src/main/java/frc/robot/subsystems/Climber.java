// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  WPI_TalonFX climberLeft, climberRight;
  XboxController operatorController;

  public Climber() {
    climberLeft = new WPI_TalonFX(Constants.CLIMBER_LEFT_MOTOR_ID);
    climberRight = new WPI_TalonFX(Constants.CLIMBER_RIGHT_MOTOR_ID);
    operatorController = new XboxController(Constants.OPERATOR_CONTROLLER_PORT);

    //resets encoders
    // climberLeft.configFactoryDefault();
    // climberRight.configFactoryDefault();

    //sets the right climber motor to follow the left one
    climberRight.follow(climberRight);

    //sets maximum and minimum power limits for the motors
    // climberLeft.configNominalOutputForward(0, Constants.kTimeoutMs);
    // climberLeft.configNominalOutputReverse(0, Constants.kTimeoutMs);
    // climberLeft.configPeakOutputForward(1, Constants.kTimeoutMs);
    // climberLeft.configPeakOutputReverse(-1, Constants.kTimeoutMs);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
