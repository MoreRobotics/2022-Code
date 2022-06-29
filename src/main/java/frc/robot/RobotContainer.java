// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  DriveTrain driveTrain = new DriveTrain();
  Shooter shooter = new Shooter();
  Intake intake = new Intake();
  Climber climber = new Climber();
  Turret turret = new Turret();
  Transporter transporter = new Transporter();

  //XboxController setup
  XboxController driverController = new XboxController(Constants.DRIVER_CONTROLLER_PORT);
  JoystickButton driverAButton = new JoystickButton(driverController, XboxController.Button.kA.value);
  JoystickButton driverBButton = new JoystickButton(driverController, XboxController.Button.kB.value);
  JoystickButton driverXButton = new JoystickButton(driverController, XboxController.Button.kX.value);
  JoystickButton driverLBumper = new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);
  JoystickButton driverRBumper = new JoystickButton(driverController, XboxController.Button.kRightBumper.value);
  
  XboxController operatorController = new XboxController(Constants.OPERATOR_CONTROLLER_PORT);
  JoystickButton operatorAButton = new JoystickButton(operatorController, XboxController.Button.kA.value);
  JoystickButton operatorBButton = new JoystickButton(operatorController, XboxController.Button.kB.value);
  JoystickButton operatorXButton = new JoystickButton(operatorController, XboxController.Button.kX.value);
  JoystickButton operatorYButton = new JoystickButton(operatorController, XboxController.Button.kY.value);
  JoystickButton operatorLBumper = new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value);
  JoystickButton operatorRBumper = new JoystickButton(operatorController, XboxController.Button.kRightBumper.value);
  JoystickButton operatorBackButton = new JoystickButton(operatorController, XboxController.Button.kBack.value);

  POVButton operatorDPadLeft = new POVButton(operatorController, 270);
  POVButton operatorDPadRight = new POVButton(operatorController, 90);
  POVButton operatorDPadUp = new POVButton(operatorController, 0);
  POVButton operatorDPadUpLeft = new POVButton(operatorController, 315);
  POVButton operatorDPadUpRight = new POVButton(operatorController, 45);
  POVButton operatorDPadDown = new POVButton(operatorController, 180);
  POVButton operatorDPadDownLeft = new POVButton(operatorController, 225);
  POVButton operatorDPadDownRight = new POVButton(operatorController, 135);

  POVButton driverDPadUp = new POVButton(driverController, 0);
  POVButton driverDPadUpLeft = new POVButton(driverController, 315);
  POVButton driverDPadUpRight = new POVButton(driverController, 45);
  POVButton driverDPadDown = new POVButton(driverController, 180);
  POVButton driverDPadDownLeft = new POVButton(driverController, 225);
  POVButton driverDPadDownRight = new POVButton(driverController, 135);

  Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    intake.raiseIntake();
    configureButtonBindings();
  }

  
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    turretHandler();
    hoodHandler();
    outtakeHandler();
    intakeHandler();
    climberHandler();
    shooterHandler();
  }

  private void turretHandler() {
    operatorDPadLeft.whenHeld(new TurnTurretLeft(turret));
    operatorDPadDownLeft.whenHeld(new TurnTurretLeft(turret));
    operatorDPadUpLeft.whenHeld(new TurnTurretLeft(turret));
    operatorDPadRight.whenHeld(new TurnTurretRight(turret));
    operatorDPadDownRight.whenHeld(new TurnTurretRight(turret));
    operatorDPadUpRight.whenHeld(new TurnTurretRight(turret));
    operatorDPadUp.whenHeld(new MoveTurret(turret, Constants.TURRET_UP_POSITION));     
    operatorAButton.whenHeld(new ParallelCommandGroup(new TurnTurret(turret), new MoveHoodAuto(shooter)));
  }

  private void hoodHandler() {
    operatorRBumper.whenHeld(new HoodUp(shooter));
    operatorLBumper.whenHeld(new HoodDown(shooter));
  }


  private void outtakeHandler() {
    driverAButton.whenHeld(new ReverseTowerTransporter(transporter));
  }


  private void intakeHandler() {
    driverXButton.whenHeld(new ParallelCommandGroup(new RunIntake(intake), new RunTransporter(transporter)));
  }


  private void climberHandler() {
    driverLBumper.whenHeld(new ExtendClimber(climber));
    driverRBumper.whenHeld(new RetractClimber(climber));
    driverDPadDown.whenHeld(new RotateClimberBackward(climber));
    driverDPadUp.whenHeld(new RotateClimberForward(climber));
  }

  /*
  public void shooterHandler() {
    operatorXButton.whenHeld(new SequentialCommandGroup(
      new ParallelDeadlineGroup(new WaitCommand(2.5), new RunShooter(shooter), new TurnTurret(turret)), 
      new ParallelDeadlineGroup(new WaitCommand(2.0), new RunTower(transporter), new RunShooter(shooter), new TurnTurret(turret), new MoveHoodAuto(shooter)),
      new ParallelCommandGroup(new RunTowerTransporter(transporter), new RunShooter(shooter), new TurnTurret(turret))));
  }

  */
  public void shooterHandler() {
    operatorXButton.whenHeld(new ParallelCommandGroup(
      new InstantCommand(() ->{compressor.disable();}),
      new AimBot(shooter),
      new TurnTurret(turret),
      new SequentialCommandGroup(new WaitCommand(2), new ParallelDeadlineGroup(new WaitCommand(1.0), new RunTower(transporter)), new RunTowerTransporter(transporter))));

      operatorXButton.whenReleased(new InstantCommand(() ->{compressor.enableDigital();}));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new ParallelCommandGroup(
      new SequentialCommandGroup(
        new ParallelDeadlineGroup(new WaitCommand(1.8), new DriveForwardAuto(driveTrain), new RunIntake(intake), new RunTransporter(transporter)),
        
        new ParallelDeadlineGroup(new WaitCommand(4.5), new AimBot(shooter), new TurnTurret(turret), 
          new SequentialCommandGroup(new WaitCommand(1.5), new ParallelDeadlineGroup(new WaitCommand(1.0), new RunTower(transporter)), new RunTowerTransporter(transporter)))
      ));
  }
}