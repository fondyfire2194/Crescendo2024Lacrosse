// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.CameraConstants.CameraValues;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.LimelightVision;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;

public class AutoPickupNote extends Command {
  /** Creates a new AlignToTagSetShootSpeed. */

  private final SwerveSubsystem m_swerve;
  private final TransferSubsystem m_transfer;
  private final CameraValues m_camval;
  private final LimelightVision m_llv;

  private double rotationVal;

  private Pose2d startPose = new Pose2d();

  private Timer elapsedTime;

  public AutoPickupNote(
      SwerveSubsystem swerve,
      TransferSubsystem transfer,
      LimelightVision llv,
      CameraValues camval)

  {
    m_swerve = swerve;
    m_llv = llv;
    m_transfer = transfer;
    m_camval = camval;
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_llv.setRearNoteDetectorPipeline();
    startPose = m_swerve.getPose();
    elapsedTime.reset();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // get horizontal angle

    if (LimelightHelpers.getTV(m_camval.camname)) {

      double angleError = LimelightHelpers.getTX(m_camval.camname);

      rotationVal = m_swerve.m_alignNotePID.calculate(angleError, 0);

    }
    /* Drive */
    m_swerve.drive(
        SwerveConstants.notePickupSpeed,
        0,
        rotationVal *= Constants.SwerveConstants.kmaxAngularVelocity,
        false,
        true,
        false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elapsedTime.hasElapsed(3) || m_transfer.noteAtIntake();
  }
}
