// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;

public class AutoShoot extends Command {
  private final ArmSubsystem arm;
  private final LimelightVision llv;
  private final ShooterSubsystem shooter;
  private final SwerveSubsystem swerve;
  private final TransferSubsystem transfer;
  private Debouncer armSetpointDebouncer = new Debouncer(ArmConstants.debounceTime);
  private Debouncer shooterSetpointDebouncer = new Debouncer(ShooterConstants.debounceTime);
  private Debouncer swerveOnTargetDebouncer = new Debouncer(SwerveConstants.debounceTime);

  /** Creates a new AutoShoot. */
  public AutoShoot(ArmSubsystem arm, ShooterSubsystem shooter, SwerveSubsystem swerve, TransferSubsystem transfer,
      LimelightVision llv) {
    this.arm = arm;
    this.llv = llv;
    this.shooter = shooter;
    this.swerve = swerve;
    this.transfer = transfer;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm, shooter, transfer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double distance = Units.inchesToMeters(llv.getDistanceFromTag(CameraConstants.frontLeftCamera));

    SmartDashboard.putNumber("DIST", distance);

    double angle = Constants.armAngleMap.get(distance);
    arm.setGoal(angle);

    double shooterRPM = Constants.shooterRPMMap.get(distance);
    shooter.setCommandRPM(shooterRPM);
    shooter.setRunShooter();

    SmartDashboard.putNumber("Auto Shoot/Desired Angle", angle);
    SmartDashboard.putNumber("Auto Shoot/Desired RPM", shooterRPM);

    if (swerveOnTargetDebouncer.calculate(swerve.getOnTarget())
        && armSetpointDebouncer.calculate(arm.atSetpoint())
        && shooterSetpointDebouncer.calculate(shooter.bothAtSpeed(.2)))
      transfer.transferToShooter();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    transfer.stopMotor();
    shooter.setCommandRPM(ShooterConstants.baseRunVelocity);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !transfer.noteAtIntake();
  }
}
