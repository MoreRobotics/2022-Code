package frc.robot.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.TrajectoryManager;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunAuto extends SequentialCommandGroup {
  /** Creates a new RunAuto. */

  private final DriveTrain m_driveTrain;
  public RunAuto(DriveTrain driveTrain, Trajectory path) {
    m_driveTrain = driveTrain;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    driveTrain.setPose(path);
    addCommands(m_driveTrain.getRamseteCommand(path));
  }
}
