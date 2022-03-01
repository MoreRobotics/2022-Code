// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  WPI_TalonFX climberLeft, climberRight, climberMid;
  XboxController operatorController;

  public Climber() {
    climberLeft = new WPI_TalonFX(Constants.CLIMBER_LEFT_MOTOR_ID);
    climberRight = new WPI_TalonFX(Constants.CLIMBER_RIGHT_MOTOR_ID);
    climberMid = new WPI_TalonFX(Constants.CLIMBER_MID_MOTOR_ID);
    operatorController = new XboxController(Constants.OPERATOR_CONTROLLER_PORT);

    //resets encoders
    climberLeft.configFactoryDefault();
    climberRight.configFactoryDefault();
    climberMid.configFactoryDefault();

    //sets the right climber motor to follow the left one
    climberRight.follow(climberLeft);

    //sets maximum and minimum power limits for the motors
    climberLeft.configNominalOutputReverse(0, Constants.kTimeoutMs);
    climberLeft.configPeakOutputReverse(-1, Constants.kTimeoutMs);
    climberLeft.configPeakOutputForward(1, Constants.kTimeoutMs);
    climberLeft.configNominalOutputForward(0, Constants.kTimeoutMs);

    climberMid.configNominalOutputReverse(0, Constants.kTimeoutMs);
    climberMid.configPeakOutputReverse(-1, Constants.kTimeoutMs);
    climberMid.configPeakOutputForward(1, Constants.kTimeoutMs);
    climberMid.configNominalOutputForward(0, Constants.kTimeoutMs);

    // climberLeft.config_kF(Constants.CLIMBER_SLOT_INDEX_ID, Constants.kGains_Climber_Rotation_Speed.kF, Constants.kTimeoutMs);
    // climberLeft.config_kP(Constants.CLIMBER_SLOT_INDEX_ID, Constants.kGains_Climber_Rotation_Speed.kP, Constants.kTimeoutMs);
    // climberLeft.config_kI(Constants.CLIMBER_SLOT_INDEX_ID, Constants.kGains_Climber_Rotation_Speed.kI, Constants.kTimeoutMs);
    // climberLeft.config_kD(Constants.CLIMBER_SLOT_INDEX_ID, Constants.kGains_Climber_Rotation_Speed.kD, Constants.kTimeoutMs);
    
  }

  public void rotateBack() {
    //rotate the right and left climber motors to meet specified degrees
    climberLeft.set(ControlMode.PercentOutput, -0.1);
  }

  public void rotateForward() {
    //rotate the right and left climber motors to meet specified degrees
    climberLeft.set(ControlMode.PercentOutput, 0.1);
  }

  public void extendMiddle() {
    climberMid.set(ControlMode.PercentOutput, 0.1);
  }

  public void retractMiddle() {
    climberMid.set(ControlMode.PercentOutput, -0.1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
