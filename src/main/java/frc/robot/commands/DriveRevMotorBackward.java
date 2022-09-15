package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RevMotorSub;

/** An example command that uses an example subsystem. */
public class DriveRevMotorBackward extends CommandBase {

    private final RevMotorSub revMotorSub;

    public DriveRevMotorBackward(RevMotorSub revMotorSub) {
    
    this.revMotorSub = revMotorSub;
    addRequirements(revMotorSub);

  }

  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    revMotorSub.DriveBackward();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    revMotorSub.Stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}