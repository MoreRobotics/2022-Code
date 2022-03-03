// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class TurnTurret extends CommandBase {
  /** Creates a new TurnTurret. */
  private final Turret turret;
  public TurnTurret(Turret turret) {
    this.turret = turret;
    addRequirements(turret);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turret.camera.setLED(VisionLEDMode.kOn);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turret.turnTurret();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.camera.setLED(VisionLEDMode.kOff);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
