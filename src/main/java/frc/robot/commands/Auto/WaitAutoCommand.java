// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.WaitCommand;

/** Add your docs here. */
public class WaitAutoCommand extends WaitCommand{
    private double time;
    public WaitAutoCommand(double time){
        super(0);
        this.time = time;
    }
    @Override
    public boolean isFinished(){
        return m_timer.hasElapsed(time);
    }
}
