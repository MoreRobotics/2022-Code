// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Transporter;

public class RunTransporter extends CommandBase {
  private final Transporter transporter;
  /** Creates a new RunTransporter. */
  public RunTransporter(Transporter transporter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.transporter = transporter;
    addRequirements(transporter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    transporter.startTransporter();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    transporter.stopTransporter();
    if (SmartDashboard.getNumber("Robot State", 0) == 4) {
      SmartDashboard.putNumber("Robot State", 1);
    } else if (SmartDashboard.getNumber("Robot State", 0) == 5) {
      SmartDashboard.putNumber("Robot State", 2);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
