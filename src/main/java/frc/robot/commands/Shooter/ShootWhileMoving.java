// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CameraConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.utils.AllianceUtil;

public class ShootWhileMoving extends Command {
  private static InterpolatingDoubleTreeMap angleToleranceMap = new InterpolatingDoubleTreeMap();

  static {
    angleToleranceMap.put(1.36, 30.0353);
    angleToleranceMap.put(1.88, 25.0);
    angleToleranceMap.put(3.0, 15.0);
    angleToleranceMap.put(4.6, 10.0);
  }

  private final ArmSubsystem m_arm;
  private final TransferSubsystem m_transfer;
  private final ShooterSubsystem m_shooter;
  private final SwerveSubsystem m_swerve;
  private LimelightVision m_llv;



 

  private PIDController turnToAngleController = new PIDController(1.0, 0, 0.001);

  private Pose2d speakerPose;

  private ChassisSpeeds previouSpeeds = new ChassisSpeeds();

  private LinearFilter accelXFilter = LinearFilter.movingAverage(2);
  private LinearFilter accelYFilter = LinearFilter.movingAverage(2);

  private final double setpointDebounceTime = 0.20;
  private final double feedTime = 0.100;

  private Debouncer setpointDebouncer = new Debouncer(setpointDebounceTime);


  /** Creates a new ShootWhileMoving. */
  public ShootWhileMoving(
  
      ArmSubsystem arm,
      TransferSubsystem transfer,
      ShooterSubsystem shooter,
      SwerveSubsystem swerve,
      LimelightVision llv) {
    m_arm = arm;
    m_transfer = transfer;
    m_shooter = shooter;
    m_swerve = swerve;
    m_llv = llv;


    turnToAngleController.enableContinuousInput(-Math.PI, Math.PI);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    speakerPose = AllianceUtil.getSpeakerPose();

    previouSpeeds = m_swerve.getSpeeds();

    setpointDebouncer.calculate(false);

    accelXFilter.reset();
    accelYFilter.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d robotPose = m_swerve.getPose().getTranslation();
    ChassisSpeeds fieldSpeeds = m_swerve.getSpeeds();

    ChassisSpeeds fieldAcceleration = fieldSpeeds.minus(previouSpeeds).div(0.020);

    double fieldAccelX = accelXFilter.calculate(fieldAcceleration.vxMetersPerSecond);
    double fieldAccelY = accelYFilter.calculate(fieldAcceleration.vyMetersPerSecond);

    SmartDashboard.putNumber("Auto Shoot/Acceleration X", fieldAccelX);
    SmartDashboard.putNumber("Auto Shoot/Acceleration Y", fieldAccelY);

    double distance = m_llv.getDistanceFromSpeakerTag(CameraConstants.frontLeftCamera);

    double shotTime = Constants.shotTimeMap.get(distance);

    double armAngle = Constants.armAngleMap.get(distance);

    Translation2d virtualGoalLocation = new Translation2d();

    int iterations = 0;

    for (int i = 0; i < 5; i++) {
      iterations = i + 1;

      double virtualGoalX = speakerPose.getX()
          - shotTime * (fieldSpeeds.vxMetersPerSecond + fieldAccelX * feedTime * 0.5);
      double virtualGoalY = speakerPose.getY()
          - shotTime * (fieldSpeeds.vyMetersPerSecond + fieldAccelY * feedTime * 0.5);

      virtualGoalLocation = new Translation2d(virtualGoalX, virtualGoalY);

      double newDistance = robotPose.minus(virtualGoalLocation).getNorm();

      double newShotTime = Constants.shotTimeMap.get(newDistance);

      double newArmAngle = Constants.armAngleMap.get(newDistance);

      if (Math.abs(newArmAngle - armAngle) <= 0.05) {
        shotTime = newShotTime;
        armAngle = newArmAngle;
        distance = newDistance;
        break;
      }

      shotTime = newShotTime;
      distance = newDistance;
      armAngle = newArmAngle;
    }

    SmartDashboard.putNumber("Auto Shoot/Iterations", iterations);

    // m_swerve
    //     .getField()
    //     .getObject("Moving Goal")
    //     .setPose(new Pose2d(virtualGoalLocation, new Rotation2d()));

    Pose2d virtGoal =new Pose2d(virtualGoalLocation, new Rotation2d());

    SmartDashboard.putNumberArray("OdometryVG",
        new double[] { virtGoal.getX(), virtGoal.getY(), virtGoal.getRotation().getDegrees() });

    m_arm.setGoal(armAngle);

    SmartDashboard.putNumber("Auto Shoot/Desired Angle", armAngle);

    // Shooter speed
    m_shooter.commandRPM = Constants.shooterRPMMap.get(distance);


    Rotation2d desiredAngle = robotPose.minus(virtualGoalLocation).getAngle().plus(Rotation2d.fromRadians(Math.PI));

    if (AllianceUtil.isRedAlliance()) {
      desiredAngle = desiredAngle.plus(Rotation2d.fromRadians(Math.PI));
    }

    Rotation2d robotAngle = m_swerve.getHeading();

    double angularSpeed = turnToAngleController.calculate(robotAngle.getRadians(), desiredAngle.getRadians());

    angularSpeed = MathUtil.clamp(angularSpeed, -1.0, 1.0);


    double armAngleError = armAngle - m_arm.getAngleDegrees();
    Rotation2d driveAngleError = robotAngle.minus(desiredAngle);

    SmartDashboard.putNumber("Auto Shoot/Drive Angle Error", driveAngleError.getDegrees());

    if (setpointDebouncer.calculate(
        Math.abs(armAngleError) < ArmConstants.autoShootAngleTolerance
            && m_shooter.bothAtSpeed(.1))
        && Math.abs(driveAngleError.getDegrees()) <= angleToleranceMap.get(distance)) {
      m_transfer.transferToShooter();

    }
    previouSpeeds = fieldSpeeds;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_transfer.stopMotor();
    m_shooter.stopMotors();
    turnToAngleController.reset();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
