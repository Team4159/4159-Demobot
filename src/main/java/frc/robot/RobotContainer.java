// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.lib.FluentTrigger;
import frc.lib.HIDRumble;
import frc.lib.Orchestrator;
import frc.lib.HIDRumble.RumbleRequest;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.RumbleConstants;
import frc.robot.Constants.FeederConstants.FeederState;
import frc.robot.Constants.ShooterConstants.HoodState;
import frc.robot.Constants.ShooterConstants.ShooterState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.ArcadeDrive;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Turret.TurretPositionControl;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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

  private final Trigger shootTrigger = driverController.leftBumper();
  private final Trigger intakeTrigger = driverController.leftTrigger();
  private final Trigger outtakeTrigger = driverController.a();
  private final Trigger hoodUpTrigger = driverController.rightBumper();
  private final Trigger hoodDownTrigger = driverController.rightTrigger();
  private final Trigger turretZeroTrigger = driverController.b();
  private final Trigger hoodZeroTrigger = driverController.y();

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
        .bind(hoodUpTrigger, shooter.new AdjustHood(HoodState.UP))
        .bind(hoodDownTrigger, shooter.new AdjustHood(HoodState.DOWN));
    new FluentTrigger()
        .setDefault(shooter.new ControlSpin(ShooterState.IDLE))
        .bind(shootTrigger, shooter.new ControlSpin(ShooterState.SHOOT))
        .bind(outtakeTrigger, shooter.new ControlSpin(ShooterState.REVERSE));
    new FluentTrigger()
        .setDefault(feeder.new ChangeState(FeederState.IDLE))
        .bind(intakeTrigger, feeder.new ChangeState(FeederState.INTAKE))
        .bind(outtakeTrigger, feeder.new ChangeState(FeederState.OUTTAKE));
    turretZeroTrigger.whileTrue(new Orchestrator()
        .yield(3)
        .run(turnTurret::zeroTurret).run(() -> HIDRumble.rumble(driverController,
            new RumbleRequest(RumbleType.kLeftRumble, RumbleConstants.kTurretZeroStrength, 2, 0.15))));
    hoodZeroTrigger.whileTrue(new Orchestrator()
        .yield(3)
        .require(shooter)
        .command(shooter.new AdjustHood(HoodState.DOWN_SLOW))
        .run(() -> {
          shooter.enableHoodReverseSoftLimit(false);
          HIDRumble.rumble(driverController,
              new RumbleRequest(RumbleType.kLeftRumble, RumbleConstants.kTurretZeroStrength, 2, 0.15));
        })
        .repeat(() -> HIDRumble.rumble(driverController,
            new RumbleRequest(RumbleType.kRightRumble, RumbleConstants.kHoodZeroStrength, 2)))
        .onexit((interrupted) -> shooter.enableHoodReverseSoftLimit(true))
        .yield(5)
        .exit());
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