// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignTargetOdometry extends Command {
  /** Creates a new AlignToTagSet */

  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

  private final SwerveSubsystem m_swerve;
  public PIDController m_alignTargetPID = new PIDController(0.03, 0, 0);

  private double rotationVal;

  public AlignTargetOdometry(
      SwerveSubsystem swerve,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotSup) {
    m_swerve = swerve;
    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotSup;
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_alignTargetPID.enableContinuousInput(-180, 180);
    m_alignTargetPID.setTolerance(0.2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* Get Values, Deadband */
    double translationVal = translationLimiter.calculate(
        MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.SwerveConstants.stickDeadband));
    double strafeVal = strafeLimiter.calculate(
        MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.SwerveConstants.stickDeadband));
    rotationVal = rotationLimiter.calculate(
        MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.SwerveConstants.stickDeadband));

    // get horizontal angle
    Pose2d speakerPose = Constants.getActiveSpeakerPose();
    Pose2d robotPose = m_swerve.getPose();
    double XDiff = speakerPose.getX() - robotPose.getX();
    double YDiff = speakerPose.getY() - robotPose.getY();
    double angleRad = Math.atan2(YDiff, XDiff);
    double angle = Units.radiansToDegrees(angleRad);

    double angleError = Math.IEEEremainder(Math.abs(angle-180), 180);
    // SmartDashboard.putNumber("AngleError", angleError);
    double angleErrorRobot = angleError + robotPose.getRotation().getDegrees();
    // SmartDashboard.putNumber("AngleErrorSign", angleErrorRobot);

    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red) {
      rotationVal = m_alignTargetPID.calculate(angleErrorRobot, 0);
    } else {
      rotationVal = m_alignTargetPID.calculate(angleErrorRobot, 180);
    }

    m_swerve.drive(
        translationVal *= Constants.SwerveConstants.kmaxSpeed,
        -(strafeVal *= Constants.SwerveConstants.kmaxSpeed),
        rotationVal *= Constants.SwerveConstants.kmaxAngularVelocity,
        true,
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
    return false;
  }
}
