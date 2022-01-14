// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


public class Shooter extends SubsystemBase {

  WPI_TalonFX shooterLeft, shooterRight;
  XboxController operatorController;

  /** Creates a new Shooter. */
  public Shooter() {

    shooterLeft = new WPI_TalonFX(Constants.SHOOTER_LEFT_ID);
    shooterRight = new WPI_TalonFX(Constants.SHOOTER_RIGHT_ID);

    //resets encoders and stuff
    shooterLeft.configFactoryDefault();
    shooterRight.configFactoryDefault();

    //set motors to coast mode
    shooterLeft.setNeutralMode(NeutralMode.Coast);
    shooterRight.setNeutralMode(NeutralMode.Coast);

    //set the right motor to do the opposite of the left motor
    shooterRight.setInverted(true);

    //set the right shooter motor to move with the left shooter motor
    shooterRight.follow(shooterLeft);

    //sets maximun and minimum power to send to the shooter motors
    shooterLeft.configNominalOutputForward(0, Constants.kTimeoutMs);
    shooterLeft.configNominalOutputReverse(0, Constants.kTimeoutMs);
    shooterLeft.configPeakOutputForward(1, Constants.kTimeoutMs);
    shooterLeft.configPeakOutputReverse(-1, Constants.kTimeoutMs);


  }

  //motors go vroom
  public void startShooter() {

    shooterLeft.set(ControlMode.PercentOutput, Constants.SHOOTER_SPEED);

  }

  //motors no go vroom
  public void stopShooter() {

    shooterLeft.set(ControlMode.PercentOutput, 0);

  }

  //gets the encoder value of the left wheel
  public double getWheelPosition() {
    double currentPositionEncoderUnits = shooterLeft.getSensorCollection().getIntegratedSensorPosition();
    double currentPositionDegrees = currentPositionEncoderUnits / Constants.ENCODER_UNITS_TO_DEGREES;

    return currentPositionDegrees;
  }

  //sets the encoder value
  public void zeroWheels() {
    shooterLeft.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
