// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.util.Range;

public class Drivetrain extends SubsystemBase {
   public static final ADIS16470_IMU gyro = new ADIS16470_IMU();

  private TalonFX flDriveMotor;
  private TalonFX frDriveMotor;
  private TalonFX blDriveMotor;
  private TalonFX brDriveMotor;

  private SparkMax flRotationMotor;
  private SparkMax frRotationMotor;
  private SparkMax blRotationMotor;
  private SparkMax brRotationMotor;

  TalonFXConfiguration drivingConfig;
  FeedbackConfigs feedBackConfig;

  private final AnalogEncoder flEncoder;
  private final AnalogEncoder frEncoder;
  private final AnalogEncoder blEncoder;
  private final AnalogEncoder brEncoder;

  SwerveDriveKinematics kinematics;
  SparkMaxConfig turningConfig;

  PIDController motorOutputPIDRotation;
  PIDController motorOutputPIDDrive;

  double currentAngleRAD;
  double targetAngleRAD;

    
  /** Creates a new Drivetrain. */
  public Drivetrain() {
    motorOutputPIDRotation = new PIDController(0.5, 0, 0.00); 
    motorOutputPIDRotation.enableContinuousInput(-Math.PI, Math.PI);


    flRotationMotor = new SparkMax(Constants.flR, MotorType.kBrushless);
    frRotationMotor = new SparkMax(Constants.frR, MotorType.kBrushless);
    blRotationMotor = new SparkMax(Constants.blR, MotorType.kBrushless);
    brRotationMotor = new SparkMax(Constants.brR, MotorType.kBrushless);

    flDriveMotor = new TalonFX(Constants.flD);
    frDriveMotor = new TalonFX(Constants.frD);
    blDriveMotor = new TalonFX(Constants.blD);
    brDriveMotor = new TalonFX(Constants.brD);

    drivingConfig = new TalonFXConfiguration();
    feedBackConfig = new FeedbackConfigs();

    feedBackConfig.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    feedBackConfig.SensorToMechanismRatio = 5.14;

     drivingConfig.withFeedback(feedBackConfig);
     drivingConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
    
    
    flDriveMotor.getConfigurator().apply(drivingConfig);
    flDriveMotor.getConfigurator().apply(new Slot0Configs().withKP(Constants.driveKp).withKI(Constants.driveKi).withKD(Constants.driveKd));
    frDriveMotor.getConfigurator().apply(drivingConfig);
    frDriveMotor.getConfigurator().apply(new Slot0Configs().withKP(Constants.driveKp).withKI(Constants.driveKi).withKD(Constants.driveKd));
    blDriveMotor.getConfigurator().apply(drivingConfig);
    blDriveMotor.getConfigurator().apply(new Slot0Configs().withKP(Constants.driveKp).withKI(Constants.driveKi).withKD(Constants.driveKd));
    brDriveMotor.getConfigurator().apply(drivingConfig);
    brDriveMotor.getConfigurator().apply(new Slot0Configs().withKP(Constants.driveKp).withKI(Constants.driveKi).withKD(Constants.driveKd));

    flEncoder = new AnalogEncoder(Constants.flE, 2*Math.PI, 0);
    frEncoder = new AnalogEncoder(Constants.frE, 2*Math.PI, 0);
    blEncoder = new AnalogEncoder(Constants.blE, 2*Math.PI, 0);
    brEncoder = new AnalogEncoder(Constants.brE, 2*Math.PI, 0);

    turningConfig = new SparkMaxConfig();
    turningConfig
                    .inverted(false)
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20)
                    .closedLoopRampRate(0.15)
                    .openLoopRampRate(0.2);

    flRotationMotor.configure(turningConfig, (com.revrobotics.spark.SparkBase.ResetMode) null, (com.revrobotics.spark.SparkBase.PersistMode) null);
    frRotationMotor.configure(turningConfig, (com.revrobotics.spark.SparkBase.ResetMode) null, (com.revrobotics.spark.SparkBase.PersistMode) null);
    blRotationMotor.configure(turningConfig, (com.revrobotics.spark.SparkBase.ResetMode) null, (com.revrobotics.spark.SparkBase.PersistMode) null);
    brRotationMotor.configure(turningConfig, (com.revrobotics.spark.SparkBase.ResetMode) null, (com.revrobotics.spark.SparkBase.PersistMode) null);

    Translation2d frontLeftLocation = new Translation2d(-0.2, 0.31);  
    Translation2d frontRightLocation = new Translation2d(0.2, 0.31); 
    Translation2d backLeftLocation = new Translation2d(-0.2, -0.31); 
    Translation2d backRightLocation = new Translation2d(0.2, -0.31); 

    // Creating my kinematics object using the module locations
    kinematics = new SwerveDriveKinematics(
      frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation
    );
  }

  public void allMotorAtZero(){
    flDriveMotor.set(0);
    frDriveMotor.set(0);
    blDriveMotor.set(0);
    brDriveMotor.set(0);

    flRotationMotor.set(0);
    frRotationMotor.set(0);
    blRotationMotor.set(0);
    brRotationMotor.set(0);
  }

  public double getGyroAngle(){
    return gyro.getAngle();
  }

  public void resetGyroAngle(){
    gyro.reset();
  }

  double returnEncoderAngle(int encoderNumber){
    switch (encoderNumber) {
      case 0:
        return flEncoder.get()-Math.PI - Constants.flEOffset;
      case 1:
        return frEncoder.get()-Math.PI - Constants.frEOffset;  
      case 2:
        return blEncoder.get()-Math.PI - Constants.blEOffset;        
      case 3:
        return brEncoder.get()-Math.PI - Constants.brEOffset;    
      default:
        return 0;
    }
  }
  //fontion qui va faire tourner une roue à une certaine vitesse linéaire en m/s
  /*public double setLinearVelocity(double targetSpeed, TalonFX driveMotor){
    double currentMotorSpeed =  driveMotor.getVelocity().getValueAsDouble();
    //ajouter un PID qui calcule la vitesse à output dans le moteur pour atteindre le targetSpeed de manière optimale
    //peut-être utiliser le PID inclut dans les sparkmax pour plus d'efficacité
    double motorOutput = MathUtil.clamp(motorOutputPIDDrive.calculate(currentMotorSpeed, targetSpeed), -1, 1);
    
	return motorOutput;
  }*/
//fontion qui va orienter la roue vers un certain angle en rotation2d
  public double goToAngle(Rotation2d targetAngle, SparkMax rotationMotor, int encoderNumber){
    currentAngleRAD = returnEncoderAngle(encoderNumber);
    targetAngleRAD = targetAngle.getRadians(); 
    SmartDashboard.putNumber("target angle", targetAngleRAD);
    

    //ajouter un PID qui calcule la vitesse à output dans le moteur pour atteindre le targetAngle de manière optimale
     //peut-être utiliser le PID inclut dans les sparkmax pour plus d'efficacité
    double motorOutput = MathUtil.clamp(motorOutputPIDRotation.calculate(currentAngleRAD, targetAngleRAD), -1, 1);
    motorOutput = Range.threshold(0.05, motorOutput);
    SmartDashboard.putNumber("motor output", motorOutput);
    return motorOutput;
  }
//fonction qui va gérer la position et la vitesse d'une roue
  public void driveOneSwerve(SwerveModuleState moduleState, SparkMax rotationMotor, TalonFX driveMotor, int encoderNumber){
    
    // create a velocity closed-loop request, voltage output, slot 0 configs
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    // set velocity to 8 rps, add 0.5 V to overcome gravity
    driveMotor.setControl(m_request.withVelocity(moduleState.speedMetersPerSecond/Constants.RPStoMPS*1.5));
    rotationMotor.set(goToAngle(moduleState.angle, rotationMotor, encoderNumber));
   // goToAngle(moduleState.angle, rotationMotor, encoderNumber);
  }

  public void driveSwerve(double x, double y, double r, boolean fieldRelative){
    // Example chassis speeds: 1 meter per second forward, 3 meters
    // per second to the left, and rotation at 1.5 radians per second
    // counterclockwise.
    ChassisSpeeds speeds = new ChassisSpeeds();
    
    if (fieldRelative) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, r, new Rotation2d(Units.degreesToRadians(getGyroAngle()))); 

    } else {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, r, new Rotation2d(0)); 

    }
    // Convert to module states
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
    // Front left module state
    SwerveModuleState frontLeft = moduleStates[0];
   frontLeft.optimize(new Rotation2d(returnEncoderAngle(0)));
    // // Front right module state
    SwerveModuleState frontRight = moduleStates[1];
    frontRight.optimize(new Rotation2d(returnEncoderAngle(1)));
    // // Back left module state
    SwerveModuleState backLeft = moduleStates[2];
    backLeft.optimize(new Rotation2d(returnEncoderAngle(2)));
    // // Back right module state
    SwerveModuleState backRight = moduleStates[3];
    backRight.optimize(new Rotation2d(returnEncoderAngle(3)));

    driveOneSwerve(frontLeft, flRotationMotor, flDriveMotor, 0);
    driveOneSwerve(frontRight, frRotationMotor, frDriveMotor, 1);
    driveOneSwerve(backLeft, blRotationMotor, blDriveMotor, 2);
    driveOneSwerve(backRight, brRotationMotor, brDriveMotor, 3);
  }

  @Override
  public void periodic() {
     SmartDashboard.putNumber("FL", returnEncoderAngle(0));
    SmartDashboard.putNumber("FR",returnEncoderAngle(1));
    SmartDashboard.putNumber("BL", returnEncoderAngle(2));
    SmartDashboard.putNumber("BR", returnEncoderAngle(3));
    SmartDashboard.putNumber("Gyro", getGyroAngle());
    SmartDashboard.putNumber("velocity", frDriveMotor.getVelocity().getValueAsDouble());
  }
}
