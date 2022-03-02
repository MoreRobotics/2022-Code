// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  TalonSRX intakeMotor;
  XboxController operatorController;
  DoubleSolenoid raiseSolenoid;

  public Intake() {
    intakeMotor = new TalonSRX(Constants.INTAKE_MOTOR_ID);
    operatorController = new XboxController(Constants.OPERATOR_CONTROLLER_PORT);
    raiseSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.FORWARD_CHANNEL, Constants.REVERSE_CHANNEL);

  }

  public void startIntake() {
    intakeMotor.set(ControlMode.PercentOutput, Constants.INTAKE_SPEED);
  }
  
  public void stopIntake() {
    intakeMotor.set(ControlMode.PercentOutput, 0);
  }
  
  public void reverseIntake() {
    intakeMotor.set(ControlMode.PercentOutput, -Constants.INTAKE_SPEED);
  }

  public void raiseIntake() {
    raiseSolenoid.set(Value.kReverse);
  }

  public void lowerIntake() {
    raiseSolenoid.set(Value.kForward);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
