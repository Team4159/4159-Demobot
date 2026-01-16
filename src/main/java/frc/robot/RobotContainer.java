// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.FeederConstants.FeederState;
import frc.robot.Constants.ShooterConstants.ShooterState;
import frc.robot.lib.FluentTrigger;
import frc.robot.lib.Orchestrator;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.ArcadeDrive;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Turret.TurretPositionControl;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain drivetrain = new Drivetrain();
  private final Feeder feeder = new Feeder();
  private final Turret turret = new Turret();
  private final Shooter shooter = new Shooter();

  private final CommandXboxController driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  private final ArcadeDrive drive = drivetrain.new ArcadeDrive(driverController);
  private final TurretPositionControl turnTurret = turret.new TurretPositionControl(driverController);

  private final Trigger shootTrigger = driverController.leftTrigger();
  private final Trigger outtakeTrigger = driverController.a();
  private final Trigger hoodUpTrigger = driverController.rightBumper();
  private final Trigger hoodDownTrigger = driverController.rightTrigger(); 

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    drivetrain.setDefaultCommand(drive);
    turret.setDefaultCommand(turnTurret);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    new FluentTrigger()
    .bind(hoodUpTrigger, shooter.new AdjustHood(ShooterConstants.kHoodAdjustSpeed))
    .bind(hoodDownTrigger, shooter.new AdjustHood(ShooterConstants.kHoodAdjustSpeed));
    new FluentTrigger()
    .setDefault(shooter.new ControlSpin(ShooterState.OFF))
    .bind(shootTrigger, shooter.new ControlSpin(ShooterState.SHOOT))
    .bind(outtakeTrigger, shooter.new ControlSpin(ShooterState.REVERSE));
    new FluentTrigger()
      .setDefault(feeder.new ChangeState(FeederState.IDLE))
      .bind(shootTrigger, new Orchestrator().require(feeder).yield(shooter::isShooterReady).command(feeder.new ChangeState(FeederState.INTAKE)))
      .bind(outtakeTrigger, feeder.new ChangeState(FeederState.OUTTAKE));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Commands.print("No autonomous command configured");
  }
}