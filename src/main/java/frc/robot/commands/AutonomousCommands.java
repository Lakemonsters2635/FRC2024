// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

/** Add your docs here. */
public class AutonomousCommands {
    private DrivetrainSubsystem m_dts;
    public AutonomousCommands(DrivetrainSubsystem dts){
        m_dts = dts;
    }

    private Command leftNotePath(){
        return new SequentialCommandGroup(
            m_dts.createPath(
                new Pose2d(0,0, new Rotation2d(Math.toRadians(-45))),
                new Translation2d(Constants.DISTANCE_BETWEEN_NOTES/2, (Constants.DISTANCE_TO_NOTE/2)),
                new Pose2d(Constants.DISTANCE_BETWEEN_NOTES, (Constants.DISTANCE_TO_NOTE), new Rotation2d(Math.toRadians(-45)))
            ),
            m_dts.createPath(
                new Pose2d(1, -1.8, new Rotation2d(Math.toRadians(45))),
                new Translation2d(Constants.DISTANCE_BETWEEN_NOTES/2, Constants.DISTANCE_TO_NOTE/2),
                new Pose2d(Constants.DISTANCE_BETWEEN_NOTES,Constants.DISTANCE_TO_NOTE, new Rotation2d(Math.toRadians(45)))
            )
        );

    }

    private Command rightNotePath(){
        return new SequentialCommandGroup(
            m_dts.createPath(
                new Pose2d(0,0, new Rotation2d(Math.toRadians(135))),
                new Translation2d((Constants.DISTANCE_BETWEEN_NOTES/2), (Constants.DISTANCE_TO_NOTE/2)),
                new Pose2d((Constants.DISTANCE_BETWEEN_NOTES), (Constants.DISTANCE_TO_NOTE), new Rotation2d(Math.toRadians(135)))
            ),
            m_dts.createPath(
                new Pose2d((Constants.DISTANCE_BETWEEN_NOTES), Constants.DISTANCE_TO_NOTE, new Rotation2d(Math.toRadians(-135))),
                new Translation2d((Constants.DISTANCE_BETWEEN_NOTES/2), (Constants.DISTANCE_TO_NOTE/2)),
                new Pose2d(0,0, new Rotation2d(Math.toRadians(-135)))
            )
        );
    }

    private Command midNotePath(){
        return new SequentialCommandGroup(
            m_dts.createPath(
                new Pose2d(0,0, new Rotation2d(Math.toRadians(90))),
                new Translation2d(0, (Constants.DISTANCE_TO_NOTE/2)),
                new Pose2d(0, (Constants.DISTANCE_TO_NOTE), new Rotation2d(Math.toRadians(90)))
            ),
            m_dts.createPath(
                new Pose2d(0, (Constants.DISTANCE_TO_NOTE), new Rotation2d(Math.toRadians(-90))),
                new Translation2d(0, (Constants.DISTANCE_TO_NOTE/2)),
                new Pose2d(0,0, new Rotation2d(Math.toRadians(-90)))
            )
        );
    }

    public Command shootMidCommand(){
        return midNotePath();
    }

    public Command shootLeftCommand(){
        return leftNotePath();
    }

    public Command shootRightCommand(){
        return rightNotePath();
    }

    public Command shootMidLeftCommand(){
        return new SequentialCommandGroup(
            midNotePath(),
            leftNotePath()
        );
    }

    public Command shootMidRightCommand(){
        return new SequentialCommandGroup(
            midNotePath(),
            rightNotePath()
        );
    }

    public Command shootAllThreeCommand(){
        return new SequentialCommandGroup(
            leftNotePath(),
            midNotePath(),
            rightNotePath()
        );
    }
}
