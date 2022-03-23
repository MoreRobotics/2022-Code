// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Hood;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;


public class Shooter extends SubsystemBase {

  WPI_TalonFX shooterLeft, shooterRight;
  XboxController operatorController;
  public Hood hood1, hood2;
  PhotonCamera camera;
  PhotonPipelineResult result;
  double targetRPM;

  /** Creates a new Shooter. */
  public Shooter() {

    hood1 = new Hood(Constants.ACTUATOR1_PORT, Constants.ACTUATOR_LENGTH, Constants.ACTUATOR_SPEED);
    hood2 = new Hood(Constants.ACTUATOR2_PORT, Constants.ACTUATOR_LENGTH, Constants.ACTUATOR_SPEED);
    shooterLeft = new WPI_TalonFX(Constants.SHOOTER_LEFT_ID);
    shooterRight = new WPI_TalonFX(Constants.SHOOTER_RIGHT_ID);

    camera = new PhotonCamera("gloworm");

    //resets encoders
    shooterLeft.configFactoryDefault();
    shooterRight.configFactoryDefault();

    shooterLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30);

    //set motors to coast mode
    shooterLeft.setNeutralMode(NeutralMode.Coast);
    shooterRight.setNeutralMode(NeutralMode.Coast);

    //set the right shooter motor to move with the left shooter motor
    shooterRight.follow(shooterLeft);

    //set the right motor to do the opposite of the left motor
    shooterLeft.setInverted(false);
    shooterRight.setInverted(true);

    //sets maximun and minimum power to send to the shooter motors
    shooterLeft.configNominalOutputForward(0, Constants.kTimeoutMs);
    shooterLeft.configNominalOutputReverse(0, Constants.kTimeoutMs);
    shooterLeft.configPeakOutputForward(1, Constants.kTimeoutMs);
    shooterLeft.configPeakOutputReverse(-1, Constants.kTimeoutMs);

    shooterLeft.config_kF(Constants.SHOOTER_SLOT_INDEX_ID, Constants.kGains_Shooter_Velocity.kF, Constants.kTimeoutMs);
    shooterLeft.config_kP(Constants.SHOOTER_SLOT_INDEX_ID, Constants.kGains_Shooter_Velocity.kP, Constants.kTimeoutMs);
    shooterLeft.config_kI(Constants.SHOOTER_SLOT_INDEX_ID, Constants.kGains_Shooter_Velocity.kI, Constants.kTimeoutMs);
    shooterLeft.config_kD(Constants.SHOOTER_SLOT_INDEX_ID, Constants.kGains_Shooter_Velocity.kD, Constants.kTimeoutMs);
    
    
  }

  //motors go
  public void startShooter() {

    shooterLeft.set(ControlMode.PercentOutput, Constants.SHOOTER_SPEED);

  }

  public void startShooterVelocity() {

    double targetRPM = SmartDashboard.getNumber("Shooter Target RPM", Constants.SHOOTER_TARGET_RPM);
    double targetEncoderUnitsPer100Ms = targetRPM * Constants.RPM_TO_ENCODER_UNITS_PER_100_MS;

    shooterLeft.set(ControlMode.Velocity, targetEncoderUnitsPer100Ms);

  }

  //NOTE: "auto" here doesn't mean that it is used in autonomous mode. This method autonomously adjusts the shooter RPM while in teleop.
  public void autoStartShooterVelocity() {

    double targetRPM = SmartDashboard.getNumber("Shooter Target RPM", Constants.SHOOTER_TARGET_RPM);
    double targetEncoderUnitsPer100Ms = targetRPM * Constants.RPM_TO_ENCODER_UNITS_PER_100_MS;

    shooterLeft.set(ControlMode.Velocity, targetEncoderUnitsPer100Ms);
  }

  //motors stop
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

  //moves hood actuators
  public void setHoodPos(double setpoint) {
    hood1.setPosition(setpoint);
    hood2.setPosition(setpoint);
  }

  public void shooterDistance() {
    if (result.hasTargets()) {

      double range = PhotonUtils.calculateDistanceToTargetMeters(Constants.CAMERA_HEIGHT_METERS, Constants.TARGET_HEIGHT_METERS, Constants.CAMERA_PITCH_RADIANS, Units.degreesToRadians(result.getBestTarget().getPitch()));

      switch ((int)range) {
        case 1:
          break;
        case 2:
          break;
        case 3:
          break;
        case 4:
          break;
        case 5:
          break;
        case 6:
          break;
        case 7:
          break;
        case 8:
          break;
        default:
          break;

      }

      
    } else {
      // If we have no targets, stay still.
    }    
  }

  
  public void updateShootRPM(double distance) {
    double shootRPM = (((Constants.FAR_SPOT_SHOOTER_RPM - Constants.CLOSE_SPOT_SHOOTER_RPM)/
      (Constants.FAR_SPOT_LIMELIGHT_DISTANCE - Constants.CLOSE_SPOT_LIMELIGHT_DISTANCE))* //TODO: Test
      (distance - Constants.CLOSE_SPOT_LIMELIGHT_DISTANCE) + Constants.CLOSE_SPOT_SHOOTER_RPM);
    SmartDashboard.putNumber("Shooter Target RPM", shootRPM);
  }

  public void autoHood() {
    hood1.setAutoHood(SmartDashboard.getNumber("Distance", 0));
    hood2.setAutoHood(SmartDashboard.getNumber("Distance", 0));
  }
  
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    hood1.updateCurPos();
    updateShootRPM(SmartDashboard.getNumber("Distance", 0));
    SmartDashboard.putNumber("Shooter RPM", shooterLeft.getSelectedSensorVelocity() / Constants.RPM_TO_ENCODER_UNITS_PER_100_MS);
    SmartDashboard.putNumber("Shooter RPM Graph", shooterLeft.getSelectedSensorVelocity() / Constants.RPM_TO_ENCODER_UNITS_PER_100_MS);
    SmartDashboard.putNumber("Hood Angle", hood1.getPosition());
    result = camera.getLatestResult();
    if (result.hasTargets()) {
      SmartDashboard.putNumber("Distance", PhotonUtils.calculateDistanceToTargetMeters(Constants.CAMERA_HEIGHT_METERS, Constants.TARGET_HEIGHT_METERS, Constants.CAMERA_PITCH_RADIANS, Units.degreesToRadians(result.getBestTarget().getPitch())));
    }    
  }
}
