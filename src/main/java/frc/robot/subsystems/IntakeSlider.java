// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSlider extends SubsystemBase {
  private SparkMax intakeSliderMotor;
  private SparkMaxConfig intakeSliderConfig = new SparkMaxConfig();
  DigitalInput intakeLimit = new DigitalInput(Constants.intakeSliderLimitSwitchID);

  public IntakeSlider() {
    intakeSliderMotor = new SparkMax(Constants.intakeSliderMotorID, SparkMax.MotorType.kBrushless);
    intakeSliderConfig.inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(20)
        .closedLoopRampRate(0.15)
        .openLoopRampRate(0.2);
    intakeSliderMotor.configure(intakeSliderConfig, (com.revrobotics.spark.SparkBase.ResetMode) null,
        (com.revrobotics.spark.SparkBase.PersistMode) null);
  }

  public void setPower(double power) {
    intakeSliderMotor.set(power);
  }

  @Override
  public void periodic() {

  }
}
