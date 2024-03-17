// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Pathplanner;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunPPath extends SequentialCommandGroup {
  /** Creates a new RunLoadedPPPath. */
  private final SwerveSubsystem m_swerve;
  private final PathPlannerPath m_path;
  private final boolean m_setStartPose;
  private Pose2d startPosebyAlliance;

  public RunPPath(SwerveSubsystem swerve, PathPlannerPath path, boolean setStartPose) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_swerve = swerve;
    m_path = path;
    m_setStartPose = setStartPose;
    addRequirements(m_swerve);

    startPosebyAlliance = GeometryUtil.flipFieldPose(m_path.getPreviewStartingHolonomicPose());
    addCommands(
        new ConditionalCommand(m_swerve.setPose(startPosebyAlliance),
            Commands.none(),
            () -> m_setStartPose),
        AutoBuilder.followPath(m_path));
  }
}
