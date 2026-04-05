// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Aim.HoodTrackingCommand;
import frc.robot.commands.Aim.ShooterTrackingCommand;
import frc.robot.commands.Aim.TurretTrackingCommand;
import frc.robot.commands.Drivetrain.DriveCommand;
import frc.robot.commands.InstantSetter.StartAimingCommand;
import frc.robot.commands.InstantSetter.StartShooterCommand;
import frc.robot.commands.InstantSetter.StartShootingCommand;
import frc.robot.commands.InstantSetter.StopAimingCommand;
import frc.robot.commands.InstantSetter.StopShooterCommand;
import frc.robot.commands.InstantSetter.StopShootingCommand;
import frc.robot.commands.InstantSetter.Intake.ReverseIntakeCommand;
import frc.robot.commands.InstantSetter.Intake.StartIntakeCommand;
import frc.robot.commands.InstantSetter.Intake.StopIntakeCommand;
import frc.robot.commands.Manual.MoveHoodCommand;
import frc.robot.commands.Manual.MoveTurretCommand;
import frc.robot.commands.Manual.UseSlider;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeSlider;
import frc.robot.subsystems.Roller;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private static final CommandXboxController controller = new CommandXboxController(0);
   private static final CommandXboxController controller1 = new CommandXboxController(1);
  private static final Drivetrain drivetrain = new Drivetrain();
  private static final Shooter shooter = new Shooter();
  private static final Hood hood = new Hood();
  private static final IntakeSlider intakeSlider = new IntakeSlider();
  private static final Roller roller = new Roller();
  private static final Intake intake = new Intake();
  private static final Turret turret = new Turret();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    drivetrain.setDefaultCommand(new DriveCommand(drivetrain, controller));

    //auto aim commands
    shooter.setDefaultCommand(new ShooterTrackingCommand(shooter));
    //hood.setDefaultCommand(new HoodTrackingCommand(hood));
    //turret.setDefaultCommand(new TurretTrackingCommand(turret));
   

    //Manual control commands Deprecated soon
    intakeSlider.setDefaultCommand(new UseSlider(intakeSlider, controller1));
    hood.setDefaultCommand(new MoveHoodCommand(hood, controller1));
    turret.setDefaultCommand(new MoveTurretCommand(turret, controller1));


    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    controller1.a().onTrue(new StartShooterCommand(shooter));
    controller1.b().onTrue(new StopShooterCommand(shooter));
    controller1.y().onTrue(new StartShootingCommand(roller, shooter));
    controller1.y().onFalse(new StopShootingCommand(roller, shooter));
    controller1.rightBumper().onTrue(new StartAimingCommand(turret, hood, shooter));
    controller1.rightBumper().onFalse(new StopAimingCommand(turret, hood, shooter));

    controller.rightTrigger().onTrue(new StartIntakeCommand(intake));
    controller.rightTrigger().onFalse(new StopIntakeCommand(intake));
    controller.leftTrigger().onTrue(new ReverseIntakeCommand(intake));
    controller.leftTrigger().onFalse(new StopIntakeCommand(intake));
  
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
