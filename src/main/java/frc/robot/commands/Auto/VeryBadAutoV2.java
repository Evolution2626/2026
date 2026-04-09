// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Aim.ShooterTrackingCommand;
import frc.robot.commands.Aim.TurretTrackingCommand;
import frc.robot.commands.InstantSetter.StartAimingCommand;
import frc.robot.commands.InstantSetter.StartShootingCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IntakeSlider;
import frc.robot.subsystems.Roller;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.commands.Drivetrain.ResetGyroCommand;
import frc.robot.commands.InstantSetter.StopShootingCommand;
import frc.robot.commands.InstantSetter.StopAimingCommand;
import frc.robot.commands.InstantSetter.StopShooterCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class VeryBadAutoV2 extends SequentialCommandGroup {
  /** Creates a new VeryBadAutoV2. */
  Drivetrain drivetrain;
  Turret turret;
  Shooter shooter;
  Roller roller;
  IntakeSlider intakeSlider;
  public VeryBadAutoV2(Drivetrain drivetrain, Turret turret, Shooter shooter, Roller roller, IntakeSlider intakeSlider) {
    this.drivetrain = drivetrain;
    this.turret = turret;
    this.shooter = shooter;
    this.roller = roller;
    this.intakeSlider = intakeSlider;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ResetGyroCommand(drivetrain));
    addCommands(new ParallelRaceGroup(new WaitAutoCommand(1.8), new MoveBackwardCommand(drivetrain)));
    addCommands(new ParallelRaceGroup(new WaitAutoCommand(0.5), new GetTheIntakeOut(intakeSlider)));
    addCommands(new ParallelRaceGroup(new WaitAutoCommand(1.2), new RotateCommand(drivetrain)));
  
    addCommands(new StartAimingCommand(turret, shooter));
    addCommands(new ParallelRaceGroup(new WaitAutoCommand(2), new ParallelCommandGroup(new ShooterTrackingCommand(shooter), new TurretTrackingCommand(turret))));
    addCommands(new StartShootingCommand(roller, shooter));
    addCommands(new WaitAutoCommand(5));
    addCommands(new StopShootingCommand(roller, shooter));
    addCommands(new StopShooterCommand(shooter));
    addCommands(new StopAimingCommand(turret, shooter));
    
    addRequirements(drivetrain,turret,shooter,roller);
  }
}
