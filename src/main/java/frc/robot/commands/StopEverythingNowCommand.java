// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StopEverythingNowCommand extends InstantCommand {
  Shooter shooter;
  Turret turret;
  Intake intake;
  public StopEverythingNowCommand(Shooter shooter, Turret turret, Intake intake) {
    this.shooter = shooter;
    this.turret = turret;
    this.intake = intake;
    addRequirements(shooter, turret, intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     shooter.stopShooter();
      turret.setIsTracking(false);
    shooter.setIsTracking(false);
    turret.resetModifier();
        turret.setManual(false);
    intake.setPower(0);;
  }
}
