// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class MoveTurret extends CommandBase {

  private final Turret turret_class_object;
  private final int pos;
  /** Creates a new MoveTurret. */
  public MoveTurret(Turret turret_object, int pos) {
    this.turret_class_object = turret_object;
    
    this.pos = pos;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret_class_object);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turret_class_object.setTurretPos(pos);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret_class_object.stopTurret();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
