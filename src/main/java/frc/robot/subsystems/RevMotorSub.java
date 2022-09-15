package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class RevMotorSub extends SubsystemBase {

  public CANSparkMax RevMotor;

  public RevMotorSub() {

    RevMotor = new CANSparkMax(Constants.REV_MOTOR_ID, MotorType.kBrushless);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void DriveForward() {

    RevMotor.set(Constants.REV_MOTOR_SPEED);

  }

  public void DriveBackward() {

    RevMotor.set(-Constants.REV_MOTOR_SPEED);

  }

  public void Stop() {

    RevMotor.set(0);

  }
}