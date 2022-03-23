// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  WPI_TalonFX climberLeft, climberRight, climberMid;
  XboxController operatorController;
  public DigitalInput limitSwitch;

  public Climber() {
    SmartDashboard.putNumber("Rotate to", 0);
    SmartDashboard.putNumber("Extend to", 0);
    
    climberLeft = new WPI_TalonFX(Constants.CLIMBER_LEFT_MOTOR_ID);
    climberRight = new WPI_TalonFX(Constants.CLIMBER_RIGHT_MOTOR_ID);
    climberMid = new WPI_TalonFX(Constants.CLIMBER_MID_MOTOR_ID);
    limitSwitch = new DigitalInput(Constants.CLIMBER_LIMIT_SWITCH_PORT);

    operatorController = new XboxController(Constants.OPERATOR_CONTROLLER_PORT);

    //resets encoders
    climberLeft.configFactoryDefault();
    climberRight.configFactoryDefault();
    climberMid.configFactoryDefault();

    //set brake mode
    climberLeft.setNeutralMode(NeutralMode.Brake);
    climberRight.setNeutralMode(NeutralMode.Brake);
    climberMid.setNeutralMode(NeutralMode.Brake);

    //encoders
    climberLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    climberMid.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    
    climberLeft.setSelectedSensorPosition(0);
    climberMid.setSelectedSensorPosition(0);

    //sets the right climber motor to follow the left one, inverted
    climberRight.follow(climberLeft);
    climberRight.setInverted(true);

    //sets maximum and minimum power limits for the motors
    climberLeft.configNominalOutputReverse(0, Constants.kTimeoutMs);
    climberLeft.configPeakOutputReverse(-1, Constants.kTimeoutMs);
    climberLeft.configPeakOutputForward(1, Constants.kTimeoutMs);
    climberLeft.configNominalOutputForward(0, Constants.kTimeoutMs);

    climberMid.configNominalOutputReverse(0, Constants.kTimeoutMs);
    climberMid.configPeakOutputReverse(-1, Constants.kTimeoutMs);
    climberMid.configPeakOutputForward(1, Constants.kTimeoutMs);
    climberMid.configNominalOutputForward(0, Constants.kTimeoutMs);

    climberLeft.config_kF(Constants.CLIMBER_SLOT_INDEX_ID, Constants.kGains_Climber_Rotation_Speed.kF, Constants.kTimeoutMs);
    climberLeft.config_kP(Constants.CLIMBER_SLOT_INDEX_ID, Constants.kGains_Climber_Rotation_Speed.kP, Constants.kTimeoutMs);
    climberLeft.config_kI(Constants.CLIMBER_SLOT_INDEX_ID, Constants.kGains_Climber_Rotation_Speed.kI, Constants.kTimeoutMs);
    climberLeft.config_kD(Constants.CLIMBER_SLOT_INDEX_ID, Constants.kGains_Climber_Rotation_Speed.kD, Constants.kTimeoutMs);
    
    climberMid.config_kF(Constants.CLIMBER_SLOT_INDEX_ID, Constants.kGains_Climber_Rotation_Speed.kF, Constants.kTimeoutMs);
    climberMid.config_kP(Constants.CLIMBER_SLOT_INDEX_ID, Constants.kGains_Climber_Rotation_Speed.kP, Constants.kTimeoutMs);
    climberMid.config_kI(Constants.CLIMBER_SLOT_INDEX_ID, Constants.kGains_Climber_Rotation_Speed.kI, Constants.kTimeoutMs);
    climberMid.config_kD(Constants.CLIMBER_SLOT_INDEX_ID, Constants.kGains_Climber_Rotation_Speed.kD, Constants.kTimeoutMs);

    climberLeft.configForwardSoftLimitThreshold(111325, Constants.kTimeoutMs);
    climberLeft.configReverseSoftLimitThreshold(-100000, Constants.kTimeoutMs);
    climberMid.configForwardSoftLimitThreshold(227082, Constants.kTimeoutMs);
    climberMid.configReverseSoftLimitThreshold(0, Constants.kTimeoutMs);

    climberLeft.configForwardSoftLimitEnable(true, Constants.kTimeoutMs);
    climberLeft.configReverseSoftLimitEnable(true, Constants.kTimeoutMs);
    climberMid.configForwardSoftLimitEnable(true, Constants.kTimeoutMs);
    climberMid.configReverseSoftLimitEnable(true, Constants.kTimeoutMs);
  }

  public void getClimberPos() {
    SmartDashboard.putNumber("Middle Climber Position", climberMid.getSelectedSensorPosition(0));
    SmartDashboard.putNumber("Left CLimber Position", climberLeft.getSelectedSensorPosition(0));
  }

  public void rotateBack() {
    //rotate the right and left climber motors to meet specified degrees
    climberLeft.set(ControlMode.PercentOutput, -1.0);
  }

  public void rotateForward() {
    //rotate the right and left climber motors to meet specified degrees
    climberLeft.set(ControlMode.PercentOutput, 1.0);
  }

  public void extendMiddle() {
    climberMid.set(ControlMode.PercentOutput, 1.0);
  }

  public void retractMiddle() {
    climberMid.set(ControlMode.PercentOutput, -1.0);    
  }

  public void stopPivot() {
    climberLeft.set(ControlMode.PercentOutput, 0);
  }

  public void stopMiddle() {
    climberMid.set(ControlMode.PercentOutput, 0);
  }

  //Checks limit switch
  public boolean safeCheck() {
    
    return limitSwitch.get();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
