// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {
  /** Creates a new Hood. */
  private SparkMax hoodMotor;
  private SparkMaxConfig hoodConfig = new SparkMaxConfig();

  public enum HoodState {
    TRACKING, STOPPED
  }
  private HoodState hoodState = HoodState.STOPPED;  
  public Hood() {
    hoodMotor = new SparkMax(1, SparkMax.MotorType.kBrushless);
    hoodConfig.inverted(false)
                    .idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake)
                    .smartCurrentLimit(20)
                    .closedLoopRampRate(0.15)
                    .openLoopRampRate(0.2);
    hoodMotor.configure(hoodConfig, (com.revrobotics.spark.SparkBase.ResetMode) null, (com.revrobotics.spark.SparkBase.PersistMode) null);
  }

    public void setHoodState(HoodState state) {
      hoodState = state;
    }
    
    public HoodState getHoodState() {
      return hoodState;
    }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
