// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Roller extends SubsystemBase {
  private SparkMax rollerMotor;
  private SparkMaxConfig rollerMotorConfig = new SparkMaxConfig();

   public enum RollerState {
    INTAKING, OUTTAKING, STOPPED
  }
  private RollerState rollerState = RollerState.STOPPED;
  /** Creates a new Transfer. */
  public Roller() {
    rollerMotor = new SparkMax(Constants.rollerMotorID, SparkMax.MotorType.kBrushless);
    rollerMotorConfig.inverted(false)
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(60,40)
                    .closedLoopRampRate(0.15)
                    .openLoopRampRate(0.2);
    rollerMotor.configure(rollerMotorConfig, (com.revrobotics.spark.SparkBase.ResetMode) null, (com.revrobotics.spark.SparkBase.PersistMode) null);
  }
  public void setRollerState(RollerState state) {
    rollerState = state;
    switch (rollerState) {
      case INTAKING:
       // rollerMotor.set(0.5);
        break;
      case OUTTAKING:
       // rollerMotor.set(-0.5);
        break;
      case STOPPED:
       // rollerMotor.set(0);
        break;
    }
  }
  public RollerState getRollerState() {
    return rollerState;
  }
  public void setPower(double power){
    rollerMotor.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
