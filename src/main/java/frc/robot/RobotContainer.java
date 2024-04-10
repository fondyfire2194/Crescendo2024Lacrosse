// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CameraConstants;
import frc.robot.commands.CommandFactory;
import frc.robot.commands.JogClimber;
import frc.robot.commands.Arm.CheckArmAtTarget;
import frc.robot.commands.Drive.AlignTargetOdometry;
import frc.robot.commands.Drive.AlignTargetOdometryLob;
import frc.robot.commands.Drive.AlignToNote;
import frc.robot.commands.Drive.AutoPickupNote;
import frc.robot.commands.Drive.TeleopSwerve;
import frc.robot.commands.Drive.WheelRadiusCharacterization;
import frc.robot.commands.Shooter.CheckShooterAtSpeed;
import frc.robot.commands.Shooter.LobShoot;
import frc.robot.commands.Shooter.ShootFromDistance;
import frc.robot.commands.Transfer.TransferIntakeToSensor;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import monologue.Logged;

public class RobotContainer implements Logged {
        /* Subsystems */
        final SwerveSubsystem m_swerve = new SwerveSubsystem(false);

        final IntakeSubsystem m_intake = new IntakeSubsystem(false);

        final TransferSubsystem m_transfer = new TransferSubsystem(false);

        final ArmSubsystem m_arm = new ArmSubsystem(false);

        final ClimberSubsystem m_climber = new ClimberSubsystem();

        final PowerDistribution m_pd = new PowerDistribution(1, ModuleType.kRev);

        SendableChooser<Command> autoChooser;

        public final LimelightVision m_llv = new LimelightVision(false);

        public final SendableChooser<Double> m_startDelayChooser = new SendableChooser<Double>();

        public final SendableChooser<String> m_batteryChooser = new SendableChooser<String>();

        public final SendableChooser<Integer> m_cameraChooser = new SendableChooser<Integer>();

        private final CommandXboxController driver = new CommandXboxController(0);

        private final CommandXboxController codriver = new CommandXboxController(1);

        private final CommandXboxController setup = new CommandXboxController(2);

        final ShooterSubsystem m_shooter = new ShooterSubsystem(false);

        public final CommandFactory m_cf = new CommandFactory(m_swerve, m_shooter, m_arm, m_intake, m_transfer,
                        m_climber, m_llv);

        BooleanSupplier fieldCentric;

        BooleanSupplier keepAngle;

        public boolean checkCAN;

        public RobotContainer() {

                registerNamedCommands();

                Pref.deleteUnused();

                Pref.addMissing();

                m_arm.setKPKIKD();

                m_pd.resetTotalEnergy();

                configureShuffleboardAuto();

                m_transfer.setVelPID();

                configureDriverBindings();

                configureCodriverBindings();

                m_intake.setPID();

                configureSetupBindings();

                m_shooter.setTopKpKdKi();

                configureCommandScheduler();

                setDefaultCommands();

                m_shooter.setBottomKpKdKi();
        }

        private void configureDriverBindings() {

                // KEEP IN BUTTON ORDER

                fieldCentric = driver.a();
                keepAngle = () -> false;

                driver.leftTrigger().whileTrue(new ParallelCommandGroup(new AlignTargetOdometry(
                                m_swerve,
                                () -> -driver.getLeftY(),
                                () -> driver.getLeftX(),
                                () -> driver.getRightX()),
                                // .alongWith(m_cf.rumbleCommand(driver)),
                                new ShootFromDistance(m_shooter, m_swerve, m_arm)));

                driver.rightBumper().onTrue(Commands.parallel(
                                m_intake.startIntakeCommand(),
                                new TransferIntakeToSensor(m_transfer, m_intake),
                                m_cf.rumbleCommand(driver),
                                m_arm.setGoalCommand(ArmConstants.pickupAngle))
                                .withTimeout(10));

                // driver.leftBumper().whileTrue(new ParallelCommandGroup(new
                // AlignTargetOdometry(
                // m_swerve,
                // () -> -driver.getLeftY(),
                // () -> driver.getLeftX(),
                // () -> driver.getRightX()),
                // // .alongWith(m_cf.rumbleCommand(driver)),
                // new LobShoot(m_shooter, m_swerve)))
                // .onTrue(m_arm.setGoalCommand(Units.degreesToRadians(Constants.lobArmAngle)));
                driver.leftBumper().whileTrue(new AlignTargetOdometryLob(
                                m_swerve,
                                () -> -driver.getLeftY(),
                                () -> driver.getLeftX(),
                                () -> driver.getRightX()));
                // .onTrue(m_arm.setGoalCommand(Units.degreesToRadians(Constants.lobArmAngle)));

                driver.rightTrigger().onTrue(Commands.sequence(
                                m_transfer.transferToShooterCommand(),
                                // new WaitCommand(3),
                                m_shooter.stopShooterCommand(),
                                m_arm.setGoalCommand(ArmConstants.pickupAngle),
                                m_intake.stopIntakeCommand()));

                driver.b().onTrue(m_shooter.stopShooterCommand());

                driver.x().onTrue(m_shooter.startShooterCommand(3500));

                driver.y().onTrue(Commands.sequence(
                                new ShootFromDistance(m_shooter, m_swerve, m_arm).withTimeout(0.1),
                                new CheckShooterAtSpeed(m_shooter, .2),
                                new CheckArmAtTarget(m_arm),
                                m_transfer.transferToShooterCommand(),
                                m_shooter.stopShooterCommand(),
                                m_arm.setGoalCommand(ArmConstants.pickupAngle),
                                m_intake.stopIntakeCommand()));

                driver.povUp().onTrue(m_shooter.increaseRPMCommand(100));

                driver.povDown().onTrue(m_shooter.decreaseRPMCommand(100));

                driver.povRight().onTrue(Commands.runOnce(() -> m_arm.incrementArmAngle(1)));

                driver.povLeft().onTrue(Commands.runOnce(() -> m_arm.decrementArmAngle(1)));

                driver.start().onTrue(Commands.runOnce(() -> m_swerve.zeroGyro()));

                driver.back().whileTrue(new AlignToNote(m_swerve, m_transfer, m_llv,
                                () -> -driver.getLeftY(),
                                () -> driver.getLeftX(),
                                () -> driver.getRightX(),
                                CameraConstants.rearCamera));

        }

        private void configureCodriverBindings() {
                // CoDriver
                // KEEP IN BUTTON ORDER
                // jogs are in case note gets stuck

                codriver.leftTrigger().whileTrue(m_climber.raiseClimberArmsCommand(0.6))
                                .onFalse(m_climber.stopClimberCommand());

                // codriver.leftBumper().onTrue(m_arm.positionToIntakeUDACommand());
                codriver.leftBumper().whileTrue(new JogClimber(m_climber, codriver));

                codriver.rightTrigger().whileTrue(m_climber.lowerClimberArmsCommand(0.6))
                                .onFalse(m_climber.stopClimberCommand());

                codriver.rightBumper().onTrue(m_cf.positionArmRunShooterSpecialCase(Constants.lobArmAngle,
                                Constants.lobShooterSpeed));

                codriver.a().onTrue(m_cf.positionArmRunShooterSpecialCase(Constants.subwfrArmAngle,
                                Constants.subwfrShooterSpeed));

                codriver.b().onTrue(m_cf.positionArmRunShooterSpecialCase(Constants.safeStageArmAngle,
                                Constants.safeStageShooterSpeed));

                codriver.x().onTrue(m_cf.positionArmRunShooterSpecialCase(Constants.tapeLineArmAngle,
                                Constants.tapeLineShooterSpeed));

                // codriver.y().onTrue(m_cf.positionArmRunShooterSpecialCase(Constants.allianceLineArmAngle,
                // Constants.allianceLineShooterSpeed));
                codriver.y().whileTrue(new ParallelCommandGroup(new AlignTargetOdometry(
                                m_swerve,
                                () -> -driver.getLeftY(),
                                () -> driver.getLeftX(),
                                () -> driver.getRightX()),
                                // .alongWith(m_cf.rumbleCommand(driver)),
                                new LobShoot(m_shooter, m_swerve)))
                                .onTrue(m_arm.setGoalCommand(Units.degreesToRadians(Constants.lobArmAngle)));

                codriver.povUp().onTrue(m_climber.raiseClimberArmsCommand(.3));

                codriver.povDown().onTrue(m_climber.lowerClimberArmsCommand(.3));

                codriver.povLeft().whileTrue(Commands.runOnce(() -> m_transfer.transferMotor.setVoltage(-.5)))
                                .onFalse(Commands.runOnce(() -> m_transfer.transferMotor.setVoltage(0)));

                codriver.povRight().onTrue(new ParallelCommandGroup(
                                m_intake.startIntakeCommand(),
                                new TransferIntakeToSensor(m_transfer, m_intake),
                                m_cf.autopickup()));

                codriver.start().onTrue(Commands.runOnce(() -> m_swerve.resetModuleEncoders()));

                codriver.back().whileTrue(Commands.runOnce(() -> m_intake.intakeMotor.setVoltage(-8)))
                                .onFalse(Commands.runOnce(() -> m_intake.intakeMotor.setVoltage(0)));

        }

        private void configureSetupBindings() {
                // Setup
                // KEEP IN BUTTON ORDER
                // jogs are in case note gets stuck

                // setup.leftTrigger().whileTrue(new JogIntake(m_intake, setup));

                // setup.leftBumper().whileTrue(new JogArm(m_arm, setup));

                // setup.rightTrigger().whileTrue(new JogTransfer(m_transfer, setup));

                setup.rightBumper().whileTrue(new JogClimber(m_climber, setup));

                setup.a().onTrue(m_climber.lockClimberCommand());

                setup.b().onTrue(m_climber.unlockClimberCommand());

                // setup.leftBumper().whileTrue(m_swerve.quasistaticForward());

                // setup.leftTrigger().whileTrue(m_swerve.quasistaticBackward());

                // setup.rightBumper().whileTrue(m_swerve.dynamicForward());

                // setup.rightTrigger().whileTrue(m_swerve.dynamicBackward());

                // setup.a().onTrue(m_arm.setGoalCommand(Units.degreesToRadians(25)));

                // setup.b().onTrue(m_arm.setGoalCommand(Units.degreesToRadians(40)));

                setup.x().onTrue(m_arm.setGoalCommand(Units.degreesToRadians(50)));

                setup.y().onTrue(m_arm.setGoalCommand(Units.degreesToRadians(70)));

                setup.povUp().onTrue(m_arm.positionToIntakeUDACommand());

                // setup.povDown()

                // setup.povLeft()

                // setup.povRight()

                // setup.start()

                // setup.back()
        }

        private void setDefaultCommands() {

                m_swerve.setDefaultCommand(
                                new TeleopSwerve(
                                                m_swerve,
                                                () -> -driver.getLeftY(),
                                                () -> -driver.getLeftX(),
                                                () -> -driver.getRawAxis(4),
                                                fieldCentric,
                                                keepAngle));

        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

        private void configureCommandScheduler() {
                SmartDashboard.putData("CommSchd", CommandScheduler.getInstance());

                // Set the scheduler to log Shuffleboard events for command initialize,
                // interrupt, finish
                CommandScheduler.getInstance()
                                .onCommandInitialize(
                                                command -> Shuffleboard.addEventMarker(
                                                                "Command initialized", command.getName(),
                                                                EventImportance.kNormal));
                CommandScheduler.getInstance()
                                .onCommandInterrupt(
                                                command -> Shuffleboard.addEventMarker(
                                                                "Command interrupted", command.getName(),
                                                                EventImportance.kNormal));
                CommandScheduler.getInstance()
                                .onCommandFinish(
                                                command -> Shuffleboard.addEventMarker(
                                                                "Command finished", command.getName(),
                                                                EventImportance.kNormal));
        }

        private void registerNamedCommands() {

                NamedCommands.registerCommand("Arm Before Note 2", m_cf.positionArmRunShooterSpecialCase(44,
                                3000).asProxy());

                NamedCommands.registerCommand("Arm Note 2", m_cf.positionArmRunShooterSpecialCase(35,
                                3200).asProxy());

                NamedCommands.registerCommand("Halt Intake", m_intake.stopIntakeCommand().asProxy());

                NamedCommands.registerCommand("Prestart Shooter Wheels", m_shooter.startShooterCommand(4000).asProxy());

                NamedCommands.registerCommand("Stop Intake", m_intake.stopIntakeCommand().asProxy());

                NamedCommands.registerCommand(
                                "Start Intake", m_intake.startIntakeCommand().asProxy());

                NamedCommands.registerCommand(
                                "Transfer To Sensor", m_cf.runToSensorCommand().asProxy());

                NamedCommands.registerCommand(
                                "Transfer Stop", m_transfer.stopTransferCommand().asProxy());

                NamedCommands.registerCommand("Arm To Intake",
                                m_arm.setGoalCommand(ArmConstants.pickupAngle).asProxy());

                // NamedCommands.registerCommand("Shooter Low Speed",
                // m_shooter.setRPMCommand(1000, false));

                NamedCommands.registerCommand("Shoot", m_transfer.transferToShooterCommand().asProxy());

                NamedCommands.registerCommand("Align Then Shoot", m_cf.alignShootCommand().asProxy());

                NamedCommands.registerCommand("Arm Shooter SubWfr",
                                m_cf.positionArmRunShooterSpecialCase(Constants.subwfrArmAngle,
                                                Constants.subwfrShooterSpeed).asProxy()); // Constants.subwfrShooterSpeed
                                                                                          // lower speed to decrease
                                                                                          // time

                NamedCommands.registerCommand("Arm Shooter Wing 1",
                                m_cf.positionArmRunShooterSpecialCase(Constants.wing1ArmAngle,
                                                Constants.wing1ShooterSpeed).asProxy());

                NamedCommands.registerCommand("Arm Shooter Wing 2",
                                m_cf.positionArmRunShooterSpecialCase(Constants.wing2ArmAngle,
                                                Constants.wing2ShooterSpeed).asProxy());

                NamedCommands.registerCommand("Arm Shooter Wing 3",
                                m_cf.positionArmRunShooterSpecialCase(Constants.wing3ArmAngle,
                                                Constants.wing3ShooterSpeed).asProxy());

                NamedCommands.registerCommand("Arm Shooter Amp Shoot",
                                m_cf.positionArmRunShooterSpecialCase(Constants.ampStartArmAngle,
                                                Constants.ampStartShooterSpeed).asProxy());

                NamedCommands.registerCommand("Arm Shooter Source",
                                m_cf.positionArmRunShooterSpecialCase(Constants.shotSourceAngle,
                                                Constants.shotSourceSpeed).asProxy());

                NamedCommands.registerCommand("Stop Shooter", m_shooter.stopShooterCommand().asProxy());

                NamedCommands.registerCommand("LocatePickupNote", m_cf.autopickup());

        }

        void configureShuffleboardAuto() {

                m_startDelayChooser.setDefaultOption("0 sec", 0.);
                m_startDelayChooser.addOption("1 sec", 1.);
                m_startDelayChooser.addOption("2 sec", 2.);
                m_startDelayChooser.addOption("3 sec", 3.);
                m_startDelayChooser.addOption("4 sec", 4.);
                m_startDelayChooser.addOption("5 sec", 5.);

                m_batteryChooser.setDefaultOption("A", "A");
                m_batteryChooser.addOption("B", "B");
                m_batteryChooser.addOption("C", "C");
                m_batteryChooser.addOption("D", "D");
                m_batteryChooser.addOption("E", "E");
                m_batteryChooser.addOption("F", "F");

                m_cameraChooser.setDefaultOption("NO Cameras", 0);
                m_cameraChooser.addOption("LeftCamera", 1);
                m_cameraChooser.addOption("RightCamera", 2);
                m_cameraChooser.addOption("BothCameras", 3);

                autoChooser = AutoBuilder.buildAutoChooser();

                // autoChooser.addOption("Source 2 with Vision",
                //                 new SequentialCommandGroup(
                //                                 m_cf.positionArmRunShooterSpecialCase(Constants.subwfrArmAngle,
                //                                                 Constants.subwfrShooterSpeed),
                //                                 m_transfer.transferToShooterCommand(),
                //                                 AutoBuilder.followPath(PathPlannerPath
                //                                                 .fromPathFile("Source To Near Center 4")),
                //                                 new ParallelCommandGroup(m_cf.autopickup(),
                //                                                 m_intake.startIntakeCommand(),
                //                                                 m_arm.setGoalCommand(ArmConstants.pickupAngle),
                //                                                 m_cf.runToSensorCommand()),
                //                                 AutoBuilder.followPath(m_swerve.getPathToNearSource()),
                //                                 AutoBuilder.followPath(
                //                                                 PathPlannerPath.fromPathFile("Near Center 4 to Shoot")),
                //                                 m_cf.positionArmRunShooterSpecialCase(Constants.shotSourceAngle,
                //                                                 Constants.shotSourceSpeed),
                //                                 m_transfer.transferToShooterCommand()));

                Shuffleboard.getTab("Autonomous").add("AutoSelection", autoChooser)
                                .withSize(3, 1).withPosition(0, 0);

                Shuffleboard.getTab("Autonomous").add("DelayChooser", m_startDelayChooser)
                                .withSize(1, 1).withPosition(3, 0);

                Shuffleboard.getTab("Autonomous").add("FrontCameraChooser", m_cameraChooser)
                                .withSize(2, 1).withPosition(4, 0);

                Shuffleboard.getTab("Autonomous").add("BatteryChooser", m_batteryChooser)
                                .withSize(1, 1).withPosition(6, 0);

                Shuffleboard.getTab("Autonomous").addNumber("PDEnergy", () -> m_pd.getTotalEnergy())
                                .withSize(1, 1).withPosition(7, 0);

                boolean stickYFault = false;
                Shuffleboard.getTab("Autonomous").addBoolean("Sticky Fault", () -> stickYFault)
                                .withSize(1, 1).withPosition(0, 2)
                                .withProperties(Map.of("colorWhenTrue", "red", "colorWhenFalse", "black"));

                Shuffleboard.getTab("Autonomous").addNumber("Gyro", () -> m_swerve.getHeadingDegrees())
                                .withSize(1, 1).withPosition(0, 3)
                                .withWidget("Number Slider")
                                .withProperties(Map.of("Min", 0, "Max", 360));

                ShuffleboardLayout shootLayout = Shuffleboard.getTab("Autonomous")
                                .getLayout("Shooter", BuiltInLayouts.kList)
                                .withPosition(4, 1)
                                .withSize(1, 4)
                                .withProperties(Map.of("Label position", "TOP"));

                shootLayout.addNumber("CommandRPM", () -> m_shooter.commandRPM)
                                .withPosition(1, 1).withSize(1, 1);

                shootLayout.addNumber("TopRPM", () -> round2dp(m_shooter.getRPMTop(), 0))
                                .withPosition(2, 2).withSize(1, 1);

                shootLayout.addNumber("BottomRPM", () -> round2dp(m_shooter.getRPMBottom(), 0))
                                .withPosition(1, 2).withSize(1, 1);

                shootLayout.addBoolean("BothAtSpeed", () -> m_shooter.bothAtSpeed(.2))
                                .withSize(1, 1).withPosition(3, 3)
                                .withProperties(Map.of("colorWhenTrue", "green", "colorWhenFalse", "red"));

                shootLayout.addBoolean("RunShooter", () -> m_shooter.getRunShooter())
                                .withSize(1, 1).withPosition(2, 3)
                                .withProperties(Map.of("colorWhenTrue", "green", "colorWhenFalse", "red"));

                ShuffleboardLayout armLayout = Shuffleboard.getTab("Autonomous")
                                .getLayout("Arm", BuiltInLayouts.kList)
                                .withPosition(5, 1)
                                .withSize(1, 4).withProperties(Map.of("Label position", "TOP"));

                armLayout.addNumber("Arm Goal", () -> round2dp(Units.radiansToDegrees(m_arm.getCurrentGoal()), 2))
                                .withSize(1, 1).withPosition(4, 2);

                armLayout.addNumber("Arm Angle", () -> round2dp(m_arm.getAngleDegrees(), 2))
                                .withSize(1, 1).withPosition(4, 1);
                armLayout.addNumber("Arm Amps", () -> round2dp(m_arm.getAmps(), 2))
                                .withSize(1, 1).withPosition(4, 1);

                armLayout.addBoolean("AtGoal", () -> m_arm.atSetpoint())
                                .withSize(1, 1).withPosition(4, 3)
                                .withProperties(Map.of("colorWhenTrue", "green", "colorWhenFalse", "red"));

                ShuffleboardLayout intfrLayout = Shuffleboard.getTab("Autonomous")
                                .getLayout("IntTFR", BuiltInLayouts.kList)
                                .withPosition(6, 1).withSize(1, 4)
                                .withProperties(Map.of("Label position", "TOP"));

                intfrLayout.addNumber("IntakeRPM", () -> round2dp(m_intake.getRPM(), 0))
                                .withSize(1, 1).withPosition(5, 1);

                intfrLayout.addBoolean("RunIntake", () -> m_intake.getRunIntake())
                                .withSize(1, 1).withPosition(5, 2)
                                .withProperties(Map.of("colorWhenTrue", "green", "colorWhenFalse", "red"));

                intfrLayout.addNumber("Intake Amps", () -> round2dp(m_intake.getAmps(), 0))
                                .withSize(1, 1).withPosition(6, 1);

                intfrLayout.addNumber("Transfer RPM", () -> round2dp(m_transfer.getRPM(), 0))
                                .withSize(1, 1).withPosition(6, 1);

                intfrLayout.addBoolean("NoteSensed", () -> m_transfer.noteAtIntake())
                                .withSize(1, 1)
                                .withPosition(6, 2)
                                .withProperties(Map.of("colorWhenTrue", "green", "colorWhenFalse", "red"));

                Shuffleboard.getTab("Autonomous").add("PDP", m_pd)
                                .withPosition(7, 1);

        }

        public static double round2dp(double number, int dp) {
                double temp = Math.pow(10, dp);
                double temp1 = Math.round(number * temp);
                return temp1 / temp;
        }

}
