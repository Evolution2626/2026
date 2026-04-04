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

  public enum IntakeSliderState {
    EXTENDED, RETRACTED
  }
  private IntakeSliderState intakeSliderState = IntakeSliderState.RETRACTED;
  /** Creates a new IntakeSlider. */
  public IntakeSlider() {
    intakeSliderMotor = new SparkMax(Constants.intakeSliderMotorID, SparkMax.MotorType.kBrushless);
    intakeSliderConfig.inverted(false)
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20)
                    .closedLoopRampRate(0.15)
                    .openLoopRampRate(0.2);
    intakeSliderMotor.configure(intakeSliderConfig, (com.revrobotics.spark.SparkBase.ResetMode) null, (com.revrobotics.spark.SparkBase.PersistMode) null);
  }
  public void setIntakeSliderState(IntakeSliderState state) {
    intakeSliderState = state;
  }
  public IntakeSliderState getIntakeSliderState() {
    return intakeSliderState;
  }
  public void setPower(double power){
    intakeSliderMotor.set(power);
  }

  @Override
  public void periodic() {
    if(intakeSliderState == IntakeSliderState.EXTENDED) {
     // intakeSliderMotor.set(0.5);
    } else if (intakeSliderState == IntakeSliderState.RETRACTED) {
      //intakeSliderMotor.set(-0.5);
    }
    else{
      //intakeSliderMotor.set(0);
    }
    // This method will be called once per scheduler run
  }
}
