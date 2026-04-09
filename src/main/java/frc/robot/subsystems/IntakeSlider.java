// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.util.Range;

public class IntakeSlider extends SubsystemBase {
  private SparkMax intakeSliderMotor;
  private SparkMaxConfig intakeSliderConfig = new SparkMaxConfig();
   DigitalInput intakeLimiOut = new DigitalInput(Constants.intakeSliderLimitSwitchOUTID);
  DigitalInput intakeLimitIn = new DigitalInput(Constants.intakeSliderLimitSwitchINID);


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
    if(intakeLimitIn.get()){
      intakeSliderMotor.set(Range.coerce(-1, 0, power));
    }
    else if(intakeLimiOut.get()){
      intakeSliderMotor.set(Range.coerce(0, 1, power));
    }
    else{
          intakeSliderMotor.set(power);
    }

  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("limit in", intakeLimitIn.get());
    SmartDashboard.putBoolean("limit out", intakeLimiOut.get());

  }
}
