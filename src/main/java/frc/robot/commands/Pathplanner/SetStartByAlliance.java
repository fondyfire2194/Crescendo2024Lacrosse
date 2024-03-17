// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Pathplanner;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetStartByAlliance extends SequentialCommandGroup {
  /** Creates a new RunLoadedPPPath. */
  private final SwerveSubsystem m_swerve;
  private final String m_pathname;

  private Pose2d startPosebyAlliance;

  public SetStartByAlliance(SwerveSubsystem swerve, String pathName) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_swerve = swerve;
    m_pathname = pathName;
    addRequirements(m_swerve);

    PathPlannerPath m_path = PathPlannerPath.fromPathFile(m_pathname);
    Pose2d bluestart = m_path.getPreviewStartingHolonomicPose();
    startPosebyAlliance = GeometryUtil.flipFieldPose(bluestart);

    addCommands(
        new ConditionalCommand(
            m_swerve.setPose(startPosebyAlliance),
            m_swerve.setPose(bluestart),
            () -> (DriverStation.getAlliance().isPresent()
             && DriverStation.getAlliance().get() == Alliance.Red)));

  }
}
