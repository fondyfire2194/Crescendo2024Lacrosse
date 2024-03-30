// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.CameraConstants;
import frc.robot.LimelightHelpers;
import frc.robot.commands.Arm.CheckArmAtTarget;
import frc.robot.commands.Drive.AutoPickupNote;
import frc.robot.commands.Shooter.CheckShooterAtSpeed;
import frc.robot.commands.Transfer.TransferIntakeToSensor;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;

/** Add your docs here. */
public class CommandFactory {

        private final SwerveSubsystem m_swerve;

        private final IntakeSubsystem m_intake;

        private final TransferSubsystem m_transfer;

        private final ShooterSubsystem m_shooter;

        private final ArmSubsystem m_arm;

        private final LimelightVision m_llv;

        private final ClimberSubsystem m_climber;

        public CommandFactory(SwerveSubsystem swerve, ShooterSubsystem shooter, ArmSubsystem arm,
                        IntakeSubsystem intake, TransferSubsystem transfer, ClimberSubsystem climber,
                        LimelightVision llv) {
                m_swerve = swerve;
                m_shooter = shooter;
                m_arm = arm;
                m_intake = intake;
                m_transfer = transfer;
                m_climber = climber;
                m_llv = llv;
        }

        public Command autopickup() {
                return new AutoPickupNote(m_swerve, m_transfer, m_intake, m_llv,
                                CameraConstants.rearCamera).asProxy();

        }

        public Command positionArmRunShooterByDistance(double distance) {
                return new ParallelCommandGroup(
                                m_arm.setGoalCommand(Units.degreesToRadians(Constants.armAngleMap.get(distance))),
                                new CheckArmAtTarget(m_arm),
                                m_shooter.startShooterCommand(Constants.shooterRPMMap.get(distance)),
                                new CheckShooterAtSpeed(m_shooter, .2));
        }

        public Command positionArmRunShooterSpecialCase(double armAngleDeg, double shooterSpeed) {
                return new ParallelCommandGroup(
                                m_arm.setGoalCommand(Units.degreesToRadians(armAngleDeg)),
                                m_shooter.startShooterCommand(shooterSpeed),
                                new CheckArmAtTarget(m_arm),
                                //m_shooter.startShooterCommand(shooterSpeed),
                                new CheckShooterAtSpeed(m_shooter, .2));
        }

        public Command positionArmRunShooterAmp(double armAngleDeg, double shooterSpeed) {
                return new ParallelCommandGroup(
                                m_arm.setGoalCommand(Units.degreesToRadians(armAngleDeg)),
                                new CheckArmAtTarget(m_arm),
                                m_shooter.startShooterCommandAmp(shooterSpeed),
                                new CheckShooterAtSpeed(m_shooter, .2));
        }

        public Command runToSensorCommand() {
                return new TransferIntakeToSensor(m_transfer,m_intake);
        }

        public Command alignShootCommand() {
                return Commands.sequence(alignToTag(),
                                m_transfer.transferToShooterCommand());
        }

        public Command alignToTag() {
                return new FunctionalCommand(
                                () -> m_llv.setAlignSpeakerPipeline(),
                                () -> m_swerve.alignToAngle(
                                                LimelightHelpers.getTX(CameraConstants.frontLeftCamera.camname)),
                                (interrupted) -> m_llv.setAprilTag_ALL_Pipeline(),
                                () -> Math.abs(LimelightHelpers.getTX(CameraConstants.frontLeftCamera.camname)) < 1,
                                m_swerve);
        }

        public Command alignShootCommand(double meters) {
                return Commands.parallel(alignToTag(),
                                Commands.run(() -> m_arm.trackDistance(meters)),
                                Commands.run(() -> m_shooter.rpmTrackDistance(meters)));
        }

        public Command clearStickyFaultsCommand() {
                return Commands.parallel(
                                m_arm.clearFaultsCommand(),
                                m_intake.clearFaultsCommand(),
                                m_transfer.clearFaultsCommand(),
                                m_climber.clearFaultsCommand(),
                                m_swerve.clearFaultsCommand());
        }

        public Command rumbleCommand(CommandXboxController controller) {
                return Commands.run(() -> {
                        if (m_swerve.getOnTarget())
                                controller.getHID().setRumble(RumbleType.kLeftRumble, 1.0);
                        else
                                controller.getHID().setRumble(RumbleType.kLeftRumble, 0.0);

                        if (m_transfer.noteAtIntake())
                                controller.getHID().setRumble(RumbleType.kRightRumble, 1.0);
                        else
                                controller.getHID().setRumble(RumbleType.kRightRumble, 0.0);

                }).finallyDo(() -> controller.getHID().setRumble(RumbleType.kBothRumble, 0.0));
        }

}
