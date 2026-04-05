// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.InstantSetter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StopAimingCommand extends InstantCommand {
  private Turret turret;
  private Hood hood;
  private Shooter shooter;
  public StopAimingCommand(Turret turret, Hood hood, Shooter shooter) {
    this.turret = turret;
    this.hood = hood;
    this.shooter = shooter;
    addRequirements(turret, hood, shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turret.setIsTracking(false);
    hood.setIsTracking(false);
    shooter.setIsTracking(false);
  }
}
