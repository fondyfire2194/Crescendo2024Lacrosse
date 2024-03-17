// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class CheckShooterAtSpeed extends Command {
  /** Creates a new CheckShooterAtSpeed. */
  private ShooterSubsystem m_shooter;
  private int loopCtr;
  private double m_pct;

  public CheckShooterAtSpeed(ShooterSubsystem shooter, double pct) {
    m_shooter = shooter;
    m_pct=pct;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    loopCtr = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    loopCtr++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return loopCtr > 10 && m_shooter.bothAtSpeed(m_pct);
  }
}
