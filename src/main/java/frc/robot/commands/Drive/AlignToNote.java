// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.CameraConstants.CameraValues;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.LimelightVision;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;

public class AlignToNote extends Command {
  /** Creates a new AlignToTagSetShootSpeed. */

  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

  private final SwerveSubsystem m_swerve;
  private final TransferSubsystem m_transfer;
  private final CameraValues m_camval;
  private final LimelightVision m_llv;

  private double rotationVal;

  public AlignToNote(
      SwerveSubsystem swerve,
      TransferSubsystem transfer,
      LimelightVision llv,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotSup,
      CameraValues camval)

  {
    m_swerve = swerve;
    m_llv = llv;
    m_transfer = transfer;
    m_camval = camval;
    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotSup;
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_llv.setRearNoteDetectorPipeline();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* Get Values, Deadband */
    double translationVal = translationLimiter.calculate(
        MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.SwerveConstants.stickDeadband));
    double strafeVal = strafeLimiter.calculate(
        MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.SwerveConstants.stickDeadband));
    rotationVal = rotationLimiter.calculate(
        MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.SwerveConstants.stickDeadband));

    // get horizontal angle

    if (LimelightHelpers.getTV(m_camval.camname)) {

      double angleError = LimelightHelpers.getTX(m_camval.camname);

      double distanceApprox = LimelightHelpers.getTY(m_camval.camname);

      rotationVal = m_swerve.m_alignNotePID.calculate(angleError, 0);

    }
    /* Drive */
    m_swerve.drive(
        translationVal *= Constants.SwerveConstants.kmaxSpeed,
        strafeVal *= Constants.SwerveConstants.kmaxSpeed,
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
    return m_transfer.noteAtIntake();
  }
}
