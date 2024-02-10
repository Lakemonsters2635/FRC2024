// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AprilTagChooser{
  /** Creates a new AprilTagChooser. */

  private final static DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final static TelescopeSubsystem m_telescopeSubsystem = new TelescopeSubsystem();

  public final static TelescopeExtendCommand m_telescopeExtendCommand = new TelescopeExtendCommand(m_telescopeSubsystem);
  public final static TelescopeRetractCommand m_telescopeRetractCommand = new TelescopeRetractCommand(m_telescopeSubsystem);

  public final static ScoreAmpCommand m_scoreAmpCommand = new ScoreAmpCommand(m_drivetrainSubsystem, m_telescopeExtendCommand, m_telescopeRetractCommand);
  public final static ScoreShooterCommand m_scoreShooterCommand = new ScoreShooterCommand(m_drivetrainSubsystem, m_telescopeExtendCommand, m_telescopeRetractCommand);
  public final static NoteTakerCommand m_noteTakerCommand = new NoteTakerCommand(m_drivetrainSubsystem, m_telescopeExtendCommand, m_telescopeRetractCommand);

  public Command choose(){
    var alliance = DriverStation.getAlliance();
    int id = 8;
    if (alliance.get()== DriverStation.Alliance.Red) {
      if (id==5) {
        return m_scoreAmpCommand;
      }
      if (id==4) {
        return m_scoreShooterCommand;
      }
    }
    else if (alliance.get()== DriverStation.Alliance.Blue) {
      if (id==6) {
        return m_scoreAmpCommand;
      }
      if (id==8) {
        return m_scoreShooterCommand;
      }
    }
    return new Command() {};
  }
}
