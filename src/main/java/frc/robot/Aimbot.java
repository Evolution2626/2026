// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;

/** Add your docs here. */
public class Aimbot {

    static double[] redGoalCoordinates = { 11.91, 4 }; 
    static double[] blueGoalCoordinates = { 4.6, 4 }; 

    public static Pose2d getRobotPose() {
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");//TODO check the name of the limelight    

        if (limelightMeasurement.tagCount > 0) {
            Pose2d robotPose = limelightMeasurement.pose;
            SmartDashboard.putNumber("robot x", robotPose.getX());
            SmartDashboard.putNumber("robot y", robotPose.getY());
            Drivetrain.setRobotPose(robotPose);

            return robotPose;
        }

        return null;
    }

    public static double getRobotDistanceToGoal() {
        getRobotPose();
        Pose2d robotPose = Drivetrain.getRobotPose();
        Optional<Alliance> ally = DriverStation.getAlliance();
        double goalX = ally.isPresent() && ally.get() == DriverStation.Alliance.Red ? redGoalCoordinates[0]
                : blueGoalCoordinates[0]; 
        double goalY = ally.isPresent() && ally.get() == DriverStation.Alliance.Red ? redGoalCoordinates[1]
                : blueGoalCoordinates[1]; 

        if (robotPose != null) {
            Translation2d robotTranslation = robotPose.getTranslation();
            double distance = Math
                    .sqrt(Math.pow(goalX - robotTranslation.getX(), 2) + Math.pow(goalY - robotTranslation.getY(), 2));
                    SmartDashboard.putNumber("distance to goal", distance);
            return distance;
        }
        return 0;
    }

    public static double getTurretRotationOffsetToGoal() {
         getRobotPose();
        Pose2d robotPose = Drivetrain.getRobotPose();
        Optional<Alliance> ally = DriverStation.getAlliance();
        double goalX = ally.isPresent() && ally.get() == DriverStation.Alliance.Red ? redGoalCoordinates[0]
                : blueGoalCoordinates[0];
        double goalY = ally.isPresent() && ally.get() == DriverStation.Alliance.Red ? redGoalCoordinates[1]
                : blueGoalCoordinates[1];

        if (robotPose != null) {
            Translation2d robotTranslation = robotPose.getTranslation();
            double angleToGoal = Math.atan2(goalY - robotTranslation.getY(), goalX - robotTranslation.getX());
            double turretRotationOffset = angleToGoal - robotPose.getRotation().getRadians();
            return turretRotationOffset;//TODO check the math above
        }
        return 0;
    }
    public static double getTurretRotation(){
        getRobotPose();
        Pose2d robotPose = Drivetrain.getRobotPose();
        Optional<Alliance> ally = DriverStation.getAlliance();
        double goalX = ally.isPresent() && ally.get() == DriverStation.Alliance.Red ? redGoalCoordinates[0]
                : blueGoalCoordinates[0];
        double goalY = ally.isPresent() && ally.get() == DriverStation.Alliance.Red ? redGoalCoordinates[1]
                : blueGoalCoordinates[1];

        if (robotPose != null) {
            Translation2d robotTranslation = robotPose.getTranslation();
            double angleToGoal = Math.atan2(goalY - robotTranslation.getY(), goalX - robotTranslation.getX());
            return angleToGoal - robotPose.getRotation().getRadians();
        }
        return 0;
    }

    public static double getShooterRPM() {
        return linearVelocityInterpolation(getRobotDistanceToGoal());
    }

    public static double getHoodAngle() {
        return linearHoodAngleInterpolation(getRobotDistanceToGoal());
    }

    private static double linearVelocityInterpolation(double distance) {
        double[] point1 = { 2.26, 3500 }; 
        double[] point2 = { 4.5, 4800 }; 

        double slope = (point2[1] - point1[1]) / (point2[0] - point1[0]);
        double intercept = point1[1] - slope * point1[0];
        return slope * distance + intercept;
    }

    private static double linearHoodAngleInterpolation(double distance) {
        double[] point1 = { 0, 0 }; 
        double[] point2 = { 0, 0 }; 

        double slope = (point2[1] - point1[1]) / (point2[0] - point1[0]);
        double intercept = point1[1] - slope * point1[0];
        return slope * distance + intercept;
    }
}
