// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class LobShoot extends Command {
  /** Creates a new JogShooter. */
  private ShooterSubsystem m_shooter;
  private SwerveSubsystem m_swerve;
  public double distance;

  public LobShoot(ShooterSubsystem shooter, SwerveSubsystem swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_swerve = swerve;
    
    //addRequirements(m_shooter, m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
        m_shooter.setRunShooter();

  }
  @Override
  public void execute() {
    distance = m_swerve.getDistanceFromSpeaker();     
    
    double rpm = 2700;
    if (distance > 10.0) {
      rpm = 3100;
    } else if (distance < 7.0) {
      rpm = 2600;
    } else {
      rpm = 2600 + ((10.0-distance) / 2) * 500;
    }

    ChassisSpeeds fieldSpeeds = m_swerve.getSpeeds();

    double velocity = Math.sqrt(Math.pow(fieldSpeeds.vxMetersPerSecond, 2) + Math.pow(fieldSpeeds.vxMetersPerSecond, 2));
    double velocityDecreasor = velocity * 150;
    rpm = rpm - velocityDecreasor;

    m_shooter.commandRPM = rpm;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
