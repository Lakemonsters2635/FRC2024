// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OutakeSubsystem;

/** Add your docs here. */
public class AutonomousCommands {
    private DrivetrainSubsystem m_dts;
    private IntakeSubsystem m_is;
    private ArmSubsystem m_as;
    private OutakeSubsystem m_os;
    public AutonomousCommands(DrivetrainSubsystem dts, ArmSubsystem as, IntakeSubsystem is, OutakeSubsystem os){
        m_dts = dts;
        m_as = as;
        m_is = is;
        m_os = os;
    }

    private Command leftNotePath(){
        return new SequentialCommandGroup(
            m_dts.createPath(
                new Pose2d(0,0, new Rotation2d(Math.toRadians(-20))),
                new Translation2d(Constants.DISTANCE_BETWEEN_NOTES/2-0.15, -(Constants.DISTANCE_TO_NOTE/2)),
                new Pose2d(Constants.DISTANCE_BETWEEN_NOTES-0.25, -(Constants.DISTANCE_TO_NOTE), new Rotation2d(Math.toRadians(-90)))
            ),
            m_dts.createPath(
                new Pose2d(Constants.DISTANCE_BETWEEN_NOTES-0.25, -Constants.DISTANCE_TO_NOTE, new Rotation2d(Math.toRadians(160))),// 135?
                new Translation2d(Constants.DISTANCE_BETWEEN_NOTES/2-.15, -Constants.DISTANCE_TO_NOTE/2),
                new Pose2d(0, 0, new Rotation2d(Math.toRadians(90))) // 135?
            )
        );

    }

    private Command rightNotePath(){
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                m_dts.createPath(
                    new Pose2d(0,0, new Rotation2d(Math.toRadians(-160))), //-135
                    new Translation2d(-Constants.DISTANCE_BETWEEN_NOTES/2, -(Constants.DISTANCE_TO_NOTE/2)),
                    new Pose2d(-Constants.DISTANCE_BETWEEN_NOTES, -(Constants.DISTANCE_TO_NOTE), new Rotation2d(Math.toRadians(-90)))//-135
                ),
                new MoveArmToPoseCommand(m_as, Constants.ARM_PICKUP_ANGLE)
            ),

            m_dts.createPath(
                new Pose2d(-Constants.DISTANCE_BETWEEN_NOTES, -Constants.DISTANCE_TO_NOTE, new Rotation2d(Math.toRadians(30))), // 45?
                new Translation2d(-Constants.DISTANCE_BETWEEN_NOTES/2, -Constants.DISTANCE_TO_NOTE/2),
                new Pose2d(Units.inchesToMeters(-18), 0, new Rotation2d(Math.toRadians(45))) // 45?
            )
        );
    }

    // private Command midNotePath(){
    //     return new SequentialCommandGroup(
    //         new InstantCommand(() -> m_dts.resetOdometry(new Pose2d(0, 0, new Rotation2d()))).withTimeout(0.1),
    //         new InstantCommand(() -> m_dts.resetAngle()).withTimeout(0.1),
    //         new ParallelCommandGroup(
    //             m_dts.createPath(
    //                 new Pose2d(0,0, new Rotation2d(Math.toRadians(0))),
    //                 new Translation2d(0, (-Constants.DISTANCE_TO_NOTE/2)),
    //                 new Pose2d(0.1, (-Constants.DISTANCE_TO_NOTE), new Rotation2d(Math.toRadians(0)))
    //             ),
    //             new MoveArmToPoseCommand(m_as, Constants.ARM_PICKUP_ANGLE)
    //         ),
    //         // new InstantCommand(()->System.out.println("Start of move arm")),
    //         // new InstantCommand(()->m_is.inIntake()).withTimeout(0.2),
    //         // new WaitCommand(0.5),
    //         // new InstantCommand(()->m_is.stopIntake()).withTimeout(0.2),
    //         m_dts.createPath(
    //             new Pose2d(0.1, (-Constants.DISTANCE_TO_NOTE), new Rotation2d(Math.toRadians(0))),
    //             new Translation2d(0, (-Constants.DISTANCE_TO_NOTE/2)),
    //             new Pose2d(0,0, new Rotation2d(Math.toRadians(0)))
    //         )
    //     );
    // }

    private Command midNotePath(){
        return new SequentialCommandGroup(
            // new SpeakerCommand(m_as, m_is, m_os),
            new ParallelCommandGroup(
                m_dts.createPath(
                    new Pose2d(0,0, new Rotation2d(Math.toRadians(-90))),
                    new Translation2d(0, -(Constants.DISTANCE_TO_NOTE/2)),
                    new Pose2d(0, -(Constants.DISTANCE_TO_NOTE), new Rotation2d(Math.toRadians(-90)))
                ),
                new MoveArmToPoseCommand(m_as, Constants.ARM_PICKUP_ANGLE)
                // new InstantCommand(()->m_is.inIntake()).withTimeout(0.3)
            ),
            new InstantCommand(()->System.out.println("Start of move arm")).withTimeout(0.05),
            new WaitCommand(0.5),
            // new InstantCommand(()->m_is.stopIntake()).withTimeout(0.2),
            // m_dts.createPath(
            //     new Pose2d(0,0, new Rotation2d(Math.toRadians(-90))),
            //     new Translation2d(0, -(Constants.DISTANCE_TO_NOTE/2)),
            //     new Pose2d(0, -(Constants.DISTANCE_TO_NOTE), new Rotation2d(Math.toRadians(-90)))
            // ),
            m_dts.createPath(
                new Pose2d(0, -(Constants.DISTANCE_TO_NOTE), new Rotation2d(Math.toRadians(90))),
                new Translation2d(0, -(Constants.DISTANCE_TO_NOTE/2)),
                new Pose2d(0,0, new Rotation2d(Math.toRadians(90)))
            )
            // new SpeakerCommand(m_as, m_is, m_os)
        );
    }

    public Command shootMidCommand(){
        return new SequentialCommandGroup(
            new InstantCommand(() -> m_dts.resetOdometry(new Pose2d(0, 0, new Rotation2d()))).withTimeout(0.1),
            new InstantCommand(() -> m_dts.resetAngle()).withTimeout(0.1),
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
