// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.InstantSetter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Roller;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StartShooterCommand extends InstantCommand {
  private Shooter shooter;

  public StartShooterCommand(Shooter shooter) {

    this.shooter = shooter;

    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //shooter.setShooterState(Shooter.ShooterState.SHOOTING);
    //hood.setHoodState(Hood.HoodState.TRACKING);
    shooter.startShooter();
    
    
  }
}
