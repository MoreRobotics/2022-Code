// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


public class Shooter extends SubsystemBase {

  WPI_TalonFX shooterLeft, shooterRight;
  XboxController operatorController;

  /** Creates a new Shooter. */
  public Shooter() {

    shooterLeft = new WPI_TalonFX(Constants.SHOOTER_LEFT_ID);
    shooterRight = new WPI_TalonFX(Constants.SHOOTER_RIGHT_ID);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
