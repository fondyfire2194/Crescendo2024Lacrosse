// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class ArmShooterByDistance extends Command {
  /** Creates a new ArmShooterByDistance. */
  double distance;
  int loopctr;

  public ArmShooterByDistance() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    distance = 0;
    loopctr = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    loopctr++;
    if (loopctr == 5) {
      double rpm = Constants.shooterRPMMap.get(distance);
      double angle = Constants.armAngleMap.get(distance);
      SmartDashboard.putNumber("DistRPM", rpm);
      SmartDashboard.putNumber("DistAngle", angle);
      SmartDashboard.putNumber("DistDist", distance);
      loopctr = 0;
      distance += .1;

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return distance >= 7;// Units.feetToMeters(19.25);
  }
}
