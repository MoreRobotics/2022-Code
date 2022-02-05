// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.POVButton;
import frc.robot.commands.MoveHood;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunShooter;
import frc.robot.commands.RunTransporter;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transporter;
import frc.robot.subsystems.Turret;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  DriveTrain driveTrain = new DriveTrain();
  DriveTrain autoDriveTrain = new DriveTrain();
  Shooter shooter = new Shooter();
  Intake intake = new Intake();
  Climber climber = new Climber();
  Turret turret = new Turret();
  Transporter transporter = new Transporter();
  Hood hood = new Hood(Constants.ACTUATOR1_PORT, Constants.ACTUATOR_LENGTH, Constants.ACTUATOR_SPEED);


  //XboxController setup
  XboxController driverController = new XboxController(Constants.DRIVER_CONTROLLER_PORT);
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
  POVButton operatorDPadUp = new POVButton(operatorController, 0);
  POVButton operatorDPadDown = new POVButton(operatorController, 180);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    // Configure the button bindings
    configureButtonBindings();
  }

  
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    operatorLBumper.whenHeld(new RunIntake(intake));
    operatorRBumper.whenHeld(new RunTransporter(transporter));
    operatorYButton.whenPressed(new MoveHood(hood));
    shooterHandler();
  }

  public void shooterHandler() {
    operatorAButton.whenHeld(new RunShooter(shooter));
    
  }

  public void climberHandler() {
    
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /*public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }*/
}
