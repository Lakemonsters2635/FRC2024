// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

/** Add your docs here. */
public class AutonomousCommands {
    private DrivetrainSubsystem m_dts;
    public AutonomousCommands(DrivetrainSubsystem dts){
        m_dts = dts;
    }

    public Command testCommand() {
        return new SequentialCommandGroup(
            new InstantCommand(()->m_dts.resetOdometry(new Pose2d(0,0, Rotation2d.fromDegrees(0)))),
            new InstantCommand(()->m_dts.zeroOdometry()),
            m_dts.goToTargetPos(new Pose2d(0,0, Rotation2d.fromDegrees(0)))
        );
    }

    public Command pathPlannerTestCommand() {
        return new SequentialCommandGroup(
            new InstantCommand(()->m_dts.resetOdometry(new Pose2d(0,0, Rotation2d.fromDegrees(0)))),
            new InstantCommand(()->m_dts.zeroOdometry()),
            m_dts.pathPlannerAuto(new Pose2d(0,1, m_dts.getPose().getRotation()))
        );
    }

    private Command leftNotePath(){
        return new SequentialCommandGroup(
            m_dts.createPath(
                new Pose2d(0,0, new Rotation2d(Math.toRadians(-45))),
                new Translation2d(Constants.DISTANCE_BETWEEN_NOTES, -(Constants.DISTANCE_TO_NOTE/2)),
                new Pose2d(Constants.DISTANCE_BETWEEN_NOTES, -(Constants.DISTANCE_TO_NOTE), new Rotation2d(Math.toRadians(-45)))
            ),
            m_dts.createPath(
                new Pose2d(Constants.DISTANCE_BETWEEN_NOTES, -Constants.DISTANCE_TO_NOTE, new Rotation2d(Math.toRadians(45))),// 135?
                new Translation2d(Constants.DISTANCE_BETWEEN_NOTES/2, -Constants.DISTANCE_TO_NOTE/2),
                new Pose2d(0, 0, new Rotation2d(Math.toRadians(45))) // 135?
            )
        );

    }

    private Command rightNotePath(){
        return new SequentialCommandGroup(
            m_dts.createPath(
                new Pose2d(0,0, new Rotation2d(Math.toRadians(-135))),
                new Translation2d(-Constants.DISTANCE_BETWEEN_NOTES, -(Constants.DISTANCE_TO_NOTE/2)),
                new Pose2d(-Constants.DISTANCE_BETWEEN_NOTES, -(Constants.DISTANCE_TO_NOTE), new Rotation2d(Math.toRadians(-135)))
            ),
            m_dts.createPath(
                new Pose2d(-Constants.DISTANCE_BETWEEN_NOTES, -Constants.DISTANCE_TO_NOTE, new Rotation2d(Math.toRadians(135))), // 45?
                new Translation2d(-Constants.DISTANCE_BETWEEN_NOTES/2, -Constants.DISTANCE_TO_NOTE/2),
                new Pose2d(0, 0, new Rotation2d(Math.toRadians(135))) // 45?
            )
        );
    }

    private Command midNotePath(){
        return new SequentialCommandGroup(
            m_dts.createPath(
                new Pose2d(0,0, new Rotation2d(Math.toRadians(-90))),
                new Translation2d(0, -(Constants.DISTANCE_TO_NOTE/2)),
                new Pose2d(0, -(Constants.DISTANCE_TO_NOTE), new Rotation2d(Math.toRadians(-90)))
            ),
            m_dts.createPath(
                new Pose2d(0, -(Constants.DISTANCE_TO_NOTE), new Rotation2d(Math.toRadians(90))),
                new Translation2d(0, -(Constants.DISTANCE_TO_NOTE/2)),
                new Pose2d(0,0, new Rotation2d(Math.toRadians(90)))
            )
        );
    }

    public Command shootMidCommand(){
        return new SequentialCommandGroup(
            new InstantCommand(() -> m_dts.resetOdometry(new Pose2d(0, 0, new Rotation2d()))).withTimeout(0.1),
            new InstantCommand(() -> m_dts.resetAngle()),
            midNotePath()
        );
    }

    public Command shootLeftCommand(){
        return new SequentialCommandGroup(
            new InstantCommand(() -> m_dts.resetOdometry(new Pose2d(0, 0, new Rotation2d()))).withTimeout(0.1),
            new InstantCommand(() -> m_dts.resetAngle()),
            leftNotePath()
        );
    }

    public Command shootRightCommand(){
        return new SequentialCommandGroup(
            new InstantCommand(() -> m_dts.resetOdometry(new Pose2d(0, 0, new Rotation2d()))).withTimeout(0.1),
            new InstantCommand(() -> m_dts.resetAngle()),
            rightNotePath()
        );
    }

    public Command shootMidLeftCommand(){
        return new SequentialCommandGroup(
            new InstantCommand(() -> m_dts.resetOdometry(new Pose2d(0, 0, new Rotation2d()))).withTimeout(0.1),
            new InstantCommand(() -> m_dts.resetAngle()),
            midNotePath(),
            leftNotePath()
        );
    }

    public Command shootMidRightCommand(){
        return new SequentialCommandGroup(
            new InstantCommand(() -> m_dts.resetOdometry(new Pose2d(0, 0, new Rotation2d()))).withTimeout(0.1),
            new InstantCommand(() -> m_dts.resetAngle()),
            midNotePath(),
            rightNotePath()
        );
    }

    public Command shootAllThreeCommand(){
        return new SequentialCommandGroup(
            new InstantCommand(() -> m_dts.resetOdometry(new Pose2d(0, 0, new Rotation2d()))).withTimeout(0.1),
            new InstantCommand(() -> m_dts.resetAngle()),
            leftNotePath(),
            midNotePath(),
            rightNotePath()
        );
    }
}
