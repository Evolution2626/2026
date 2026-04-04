// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private SparkFlex intakeMotor;
  private SparkFlexConfig intakeConfig = new SparkFlexConfig();

  public enum IntakeState {
    INTAKING, OUTTAKING, STOPPED
  }
  private IntakeState intakeState = IntakeState.STOPPED;
  /** Creates a new Intake. */
  public Intake() {
    intakeMotor = new SparkFlex(Constants.intakeMotorID, SparkFlex.MotorType.kBrushless);
    intakeConfig.inverted(false)
                    .idleMode(IdleMode.kCoast)
                    .smartCurrentLimit(60, 40);
    intakeMotor.configure(intakeConfig, (com.revrobotics.spark.SparkBase.ResetMode) null, (com.revrobotics.spark.SparkBase.PersistMode) null);
  }

  public void setIntakeState(IntakeState state) {
    intakeState = state;
    switch (intakeState) {
      case INTAKING:
        intakeMotor.set(1.0);
        break;
      case OUTTAKING:
        intakeMotor.set(-1.0);
        break;
      case STOPPED:
        intakeMotor.set(0);
        break;
    }
  }
  public void setPower(double power){
    intakeMotor.set(power/2);
  }

  public IntakeState getIntakeState() {
    return intakeState;
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
