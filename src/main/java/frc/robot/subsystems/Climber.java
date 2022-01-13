// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  TalonSRX climberLeft, climberRight;
  XboxController operatorController;

  public Climber() {
    climberLeft = new TalonSRX(-1);
    climberRight = new TalonSRX(-1);
    operatorController = new XboxController(Constants.OPERATOR_CONTROLLER_PORT);

    climberRight.follow(climberRight);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
