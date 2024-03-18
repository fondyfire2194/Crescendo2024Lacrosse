package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import frc.lib.config.SwerveModuleConstants;

public final class Constants {

        public static final class CANIDConstants {
                // can ids 4 through 15 are used for swerve modules see SwerveConstants
                public static final int topShooterID = 17;
                public static final int bottomShooterID = 16;

                public static final int intakeID = 18;
                public static final int transferID = 19;
                public static final int armID = 20;
                public static final int armCancoderID = 21;
                public static final int climberID = 22;
                public static final int rearLeftSensor = 24;
                public static final int rearRightSensor = 25;
                public static final int transferDistanceSensorID = 26;

        }

        public static final class SwerveConstants {

                public static final double stickDeadband = 0.05;

                public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

                public static final Measure<Distance> robotWidthWithBumpers = Meters
                                .of(Meters.convertFrom(36, Inches));
                public static final Measure<Distance> robotLengthWithBumpers = Meters
                                .of(Meters.convertFrom(32, Inches));

                /* Drivetrain Constants */
                public static final Measure<Distance> trackWidth = Meters.of(Meters.convertFrom(22.125, Inches));
                public static final Measure<Distance> wheelBase = Meters.of(Meters.convertFrom(27.25, Inches));
                public static final Measure<Distance> wheelDiameter = Meters.of(Meters.convertFrom(4.0, Inches));
                public static final Measure<Distance> wheelCircumference = Meters
                                .of(wheelDiameter.magnitude() * Math.PI);

                public static final double openLoopRamp = 0.25;
                public static final double closedLoopRamp = 0.0;

                public static double mk4iL1DriveGearRatio = 1 / ((14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0));// 8.14.122807

                public static double mk4iL1TurnGearRatio = 1 / ((14.0 / 50.0) * (10.0 / 60.0));// 21.43 1/.046667

                public static double driveGearRatio = mk4iL1DriveGearRatio;

                public static double angleGearRatio = mk4iL1TurnGearRatio;

                public static final Translation2d flModuleOffset = new Translation2d(wheelBase.magnitude() / 2.0,
                                trackWidth.magnitude() / 2.0);
                public static final Translation2d frModuleOffset = new Translation2d(wheelBase.magnitude() / 2.0,
                                -trackWidth.magnitude() / 2.0);
                public static final Translation2d blModuleOffset = new Translation2d(-wheelBase.magnitude() / 2.0,
                                trackWidth.magnitude() / 2.0);
                public static final Translation2d brModuleOffset = new Translation2d(-wheelBase.magnitude() / 2.0,
                                -trackWidth.magnitude() / 2.0);

                public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                                flModuleOffset, frModuleOffset, blModuleOffset, brModuleOffset);

                /* Swerve Voltage Compensation */
                public static final double voltageComp = 12.0;

                /* Swerve Current Limiting */
                public static final int angleContinuousCurrentLimit = 20;
                public static final int driveContinuousCurrentLimit = 30;

                /* Swerve Profiling Values */
                public static final double kmaxTheoreticalSpeed = 3.7;// mps
                public static final double kmaxSpeed = 3.25; // meters per second
                public static final double kmaxAngularVelocity = 1.0 * Math.PI;

                /* Angle Motor PID Values */
                public static final double angleKP = 0.01;
                public static final double angleKI = 0.0;
                public static final double angleKD = 0.0;
                public static final double angleKFF = 0.0;

                /* Drive Motor PID Values */
                public static final double driveKP = 0.0;
                public static final double driveKI = 0.0;
                public static final double driveKD = 0.0;
                public static final double driveKFF = .5 / kmaxTheoreticalSpeed;

                /* Drive Motor Characterization Values */
                public static final double driveKS = 0.667;
                public static final double driveKV = 3.04;
                public static final double driveKA = 0.27;

                /* Drive Motor Conversion Factors */
                public static final double driveConversionPositionFactor = (wheelDiameter.magnitude() * Math.PI)
                                / driveGearRatio;
                public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
                public static final double angleConversionFactor = 360.0 / angleGearRatio;

                /* Neutral Modes */
                public static final IdleMode angleNeutralMode = IdleMode.kBrake;
                public static final IdleMode driveNeutralMode = IdleMode.kBrake;

                /* Motor Inverts */
                public static final boolean driveInvert = false;
                public static final boolean angleInvert = true;

                /* Angle Encoder Invert */
                public static final boolean canCoderInvert = false;

                public static String[] modNames = { "FL ", "FR ", "BL ", "BR " };

                /* Module Specific Constants */
                /* Front Left Module - Module 0 */
                public static final class Mod0 {

                        public static final int driveMotorID = 13;
                        public static final int angleMotorID = 14;
                        public static final int cancoderID = 15;

                        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);// 253
                        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                                        angleMotorID,
                                        cancoderID, angleOffset, false);
                }

                /* Front Right Module - Module 1 */
                public static final class Mod1 {
                        public static final int driveMotorID = 10;
                        public static final int angleMotorID = 11;
                        public static final int cancoderID = 12;

                        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);// 108
                        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                                        angleMotorID,
                                        cancoderID, angleOffset, true);
                }

                /* Back Left Module - Module 2 */
                public static final class Mod2 {

                        public static final int driveMotorID = 7;
                        public static final int angleMotorID = 8;
                        public static final int cancoderID = 9;
                        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);// 207
                        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                                        angleMotorID,
                                        cancoderID, angleOffset, false);
                }

                /* Back Right Module - Module 3 */
                public static final class Mod3 {
                        public static final int driveMotorID = 4;
                        public static final int angleMotorID = 5;
                        public static final int cancoderID = 6;

                        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);// 239
                        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                                        angleMotorID,
                                        cancoderID, angleOffset, true);
                }

                public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
                                new PIDConstants(5.0, 0, 0), // Translation constants
                                new PIDConstants(5.0, 0, 0), // Rotation constants
                                kmaxSpeed,
                                flModuleOffset.getNorm(), // Drive base radius (distance from center to furthest module)
                                new ReplanningConfig());

                public static double alignKp = .02;
                public static double alighKd = 0;

                public static double maxTranslationalAcceleration;

                public static double turnToAngleMaxVelocity;

                public static double debounceTime;

                public static double alignNoteKp = .01;

                public static double alignNoteKd = 0;

        }

        public static final class KeepAngle {
                public static final double kp = 0.30;
                public static final double ki = 0.0;
                public static final double kd = 0.0;
        }

        public static final class FieldConstants {
                public static final double FIELD_WIDTH = 8.21;
                public static final double FIELD_LENGTH = 16.54;

                public static final Pose2d speakerBlueAlliance = new Pose2d(0.0, 5.5, Rotation2d.fromDegrees(0.0));
                public static final Pose2d speakerRedAlliance = new Pose2d(16.54, 5.5, Rotation2d.fromDegrees(180.0));

                public static Pose2d driverStationBlueAlliance = new Pose2d();
                public static Pose2d driverStationRedAlliance = new Pose2d();

                public static final Pose2d blueNote1 = new Pose2d(2.89, 6.99, new Rotation2d());
                public static final Pose2d blueNote2 = new Pose2d(2.89, 5.54, new Rotation2d());
                public static final Pose2d blueNote3 = new Pose2d(2.89, 4.09, new Rotation2d());

                public static final Pose2d centerNote1 = new Pose2d(8.28, 7.45, new Rotation2d());
                public static final Pose2d centerNote2 = new Pose2d(8.28, 5.77, new Rotation2d());
                public static final Pose2d centerNote3 = new Pose2d(8.28, 4.10, new Rotation2d());
                public static final Pose2d centerNote4 = new Pose2d(8.28, 2.44, new Rotation2d());
                public static final Pose2d centerNote5 = new Pose2d(8.28, 0.75, new Rotation2d());

        }

        public static final class AprilTagConstants {
                public static AprilTagFieldLayout layout;
                static {
                        try {
                                layout = AprilTagFieldLayout
                                                .loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
                        } catch (Exception e) {
                                e.printStackTrace();
                        }
                }
        }

        public static final class GlobalConstants {
                public static final int ROBOT_LOOP_HZ = 50;
                /** Robot loop period */
                public static final double ROBOT_LOOP_PERIOD = 1.0 / ROBOT_LOOP_HZ;
        }

        public class PDPConstants {

                public static final int FRONT_LEFT_DRIVE_CHANNEL = 1;
                public static final int FRONT_RIGHT_DRIVE_CHANNEL = 2;
                public static final int BACK_LEFT_DRIVE_CHANNEL = 3;
                public static final int BACK_RIGHT_DRIVE_CHANNEL = 8;

                public static final int FRONT_LEFT_TURN_CHANNEL = 4;
                public static final int FRONT_RIGHT_TURN_CHANNEL = 5;
                public static final int BACK_LEFT_TURN_CHANNEL = 6;
                public static final int BACK_RIGHT_TURN_CHANNEL = 7;

        }

        public static final class CameraConstants {

                public static class CameraValues {
                        public String camname = "name";
                        public String ipaddress = "ip";
                        public double forward;
                        public double side;
                        public double up;
                        public double roll;
                        public double pitch;
                        public double yaw;
                        public boolean isUsed = false;
                        public boolean isActive = false;

                        public CameraValues(
                                        String camname,
                                        String ipaddress,
                                        double forward, double side, double up, double roll, double pitch, double yaw,
                                        boolean isUsed,
                                        boolean isActive) {
                                this.camname = camname;
                                this.ipaddress = ipaddress;
                                this.forward = forward;
                                this.side = side;
                                this.up = up;
                                this.roll = roll;
                                this.pitch = pitch;
                                this.yaw = yaw;
                                this.isUsed = isUsed;
                                this.isActive = isActive;
                        }
                }

                public static CameraValues frontLeftCamera = new CameraValues("limelight-frleft", "10.21.94.5",
                                0,
                                Units.inchesToMeters(6.75),
                                0,
                                0,
                                5, // deg
                                6,
                                true,
                                false);

                public static CameraValues frontRightCamera = new CameraValues("limelight-frright", "10.21.94.6",
                                0,
                                Units.inchesToMeters(-6.75),
                                0,
                                0,
                                5, // deg
                                6,
                                true,
                                false);

                public static CameraValues rearCamera = new CameraValues("limelight-rear", "10.21.94.10",
                                0,
                                Units.inchesToMeters(0),
                                0,
                                0,
                                5,
                                0,
                                true,
                                false);

                public static final double POSE_AMBIGUITY_CUTOFF = 0.05;
                public static final double DISTANCE_CUTOFF = 4.0;

        }

        public static final class ShooterConstants {

                public static final double maxShooterMotorRPM = 5700;
                public static final double minShooterMotorRPM = 1500;

                public static final double shooterConversionVelocityFactor = 1;
                public static final double shooterConversionPositionFactor = 1;
                public static final double topShooterKP = .0;
                public static final double topShooterKI = 0;
                public static final double topShooterKD = 0;
                public static final double topShooterKFF = 1.0 / maxShooterMotorRPM;
                public static final double bottomShooterKP = .0;
                public static final double bottomShooterKI = 0;
                public static final double bottomShooterKD = 0;
                public static final double bottomShooterKFF = 1.0 / maxShooterMotorRPM;
                public static final double voltageComp = 12;
                public static final IdleMode shooterIdleMode = IdleMode.kBrake;
                public static final int shooterContinuousCurrentLimit = 40;

                public static double baseRunVelocity = 1500;
                public static double velocityTolerance = .05;
                public static double gearing;
                public static double circumference;

                public static double jogSpeed = .25;
                public static double debounceTime = .1;

        }

        static double distance_0 = Units.feetToMeters(0);
        static double distance_1 = Units.feetToMeters(8.1);
        static double distance_2 = Units.feetToMeters(12);
        static double distance_3 = Units.feetToMeters(16);
        static double distance_4 = Units.feetToMeters(0);
        static double distance_5 = Units.feetToMeters(0);
        static double distance_6 = Units.feetToMeters(0);
        static double distance_7 = Units.feetToMeters(0);

        /** Arm angle look up table key: meters, values: degrees */
        public static final InterpolatingDoubleTreeMap armAngleMap = new InterpolatingDoubleTreeMap();

        static {
                armAngleMap.put(distance_0, 25.0);
                armAngleMap.put(distance_1, 35.0);
                armAngleMap.put(distance_2, 28.0);
                armAngleMap.put(distance_3, 23.5);
                // armAngleMap.put(distance_4, 44.0);
                // armAngleMap.put(distance_5, 40.5);
                // armAngleMap.put(distance_6, 37.5);
        }
        /** Arm angle look up table key: meters, values: degrees */
        public static final InterpolatingDoubleTreeMap shooterRPMMap = new InterpolatingDoubleTreeMap();

        static {
                shooterRPMMap.put(distance_0, 200.);
                shooterRPMMap.put(distance_1, 3000.);
                shooterRPMMap.put(distance_2, 3000.0);
                shooterRPMMap.put(distance_3, 4200.0);
                // shooterRPMMap.put(distance_4, 4000.0);
                // shooterRPMMap.put(distance_5, 4500.);
                // shooterRPMMap.put(distance_6, 5000.);
        }

        public static double ampArmAngle = 115;// degrees
        public static double ampShooterSpeed = 800;// rpm

        public static double subwfrArmAngle = 60;// degrees
        public static double subwfrShooterSpeed = 3000;// rpm

        public static double farShotSourceAngle = 22.5;
        public static double farShotSourceSpeed = 5500;

        public static double wing1ArmAngle = 35;// degrees
        public static double wing1ShooterSpeed = 3500;// rpm

        public static double wing2ArmAngle = 60;// degrees
        public static double wing2ShooterSpeed = 3000;// rpm

        public static double wing3ArmAngle = 60;// degrees
        public static double wing3ShooterSpeed = 3000;// rpm

        public static double tapeLineArmAngle = 40;
        public static double tapeLineShooterSpeed = 3500;

        public static double safeStageArmAngle = 37;
        public static double safeStageShooterSpeed = 3500;

        public static double allianceLineArmAngle = 24;
        public static double allianceLineShooterSpeed = 4500;

        public static final class ArmConstants {

                public static final double cancoderOffsetRadians = Units.degreesToRadians(15);

                public static final double maxarmMotorRPM = 5700;

                public static final double maxUsableRPM = 4800;

                public static final double NET_GEAR_RATIO = 100;// 50:1 then 2:1

                public static final double DEGREES_PER_ENCODER_REV = 360 / NET_GEAR_RATIO;// 3.6

                public static final double RADIANS_PER_ENCODER_REV = Units.degreesToRadians(DEGREES_PER_ENCODER_REV);// .0314

                public static final double armConversionPositionFactor = RADIANS_PER_ENCODER_REV;

                public static final double armConversionVelocityFactor = armConversionPositionFactor / 60; //

                public static final double MAX_DEGREES_PER_SEC = DEGREES_PER_ENCODER_REV * maxUsableRPM / 60;// 288

                public static final double MAX_RADS_PER_SEC = Units.degreesToRadians(MAX_DEGREES_PER_SEC);// 5 approx

                public static final double armkMaxOutput = .75;// low for test
                public static final double armkMinOutput = -.75;

                public static final double voltageComp = 12;
                public static final IdleMode armIdleMode = IdleMode.kBrake;
                public static final int armContinuousCurrentLimit = 40;
                public static double armMinRadians = Units.degreesToRadians(15);
                public static double armMaxRadians = Units.degreesToRadians(75);
                public static double pickupAngle = Units.degreesToRadians(25);
                public static double midRange = Units.degreesToRadians(35);

                public static double kTrapVelocityRadPerSecond = Units.degreesToRadians(90);

                public static final double kTrapAccelerationRadPerSecSquared = Units.degreesToRadians(120);

                public static final double armKg = 0.55;
                public static final double armKs = 0.22;// 0.11941;
                public static final double armKv = 2.5;// volts per deg per sec so 12/max = 12/5=2.4
                public static final double armKa = .3;

                public static final double armKp = 5;
                public static final double armKi = 0.0;
                public static final double armKd = 0.0;

                public static final int currentLimit = 40;

                public static final double angleTolerance = Units.degreesToRadians(1);
                public static final double autoShootAngleTolerance = .5;
                public static double jogSpeed = .05;

                public static double debounceTime = .25;

                public static double kArmOffsetRads;
        }

        public static final class TransferConstants {

                public static final double maxTransferMotorRPM = 11000;
                public static final double transferConversionVelocityFactor = 1;
                public static final double transferConversionPositionFactor = 1;
                public static final double voltageComp = 12;
                public static final IdleMode transferIdleMode = IdleMode.kBrake;
                public static final int transferContinuousCurrentLimit = 20;
                public static double clearShooterTime = 5;
                public static double noNoteStopTime = 20;
                public static double jogSpeed = 1;

                public static final double transferKp = .00002; // P gains caused oscilliation

                public static final double transferPositionKp = .002; // P gains caused oscilliation
                public static final double transferKi = 0.0;
                public static final double transferKd = 0.0;
                public static final double transferKFF = .95 / maxTransferMotorRPM;
        }

        public static final class IntakeConstants {

                public static final double maxIntakeMotorRPM = 5700;
                public static final double intakeConversionVelocityFactor = 1;
                public static final double intakeConversionPositionFactor = 1;
                public static final double voltageComp = 12;
                public static final IdleMode intakeIdleMode = IdleMode.kBrake;
                public static final int intakeContinuousCurrentLimit = 60;
                public static double jogSpeed = 1;
                public static double reverseRPM = -500;

                public static final double intakeKp = .0001;
                public static final double intakeKi = 0.0;
                public static final double intakeKd = 0.0;
                public static final double intakeKFF = .95 / maxIntakeMotorRPM;

        }

        public static final class ClimberConstants {

                public static final double maxClimberMotorRPM = 5700;
                public static final double climberConversionVelocityFactor = 1;
                public static final double climberConversionPositionFactor = 1;
                public static final double voltageComp = 12;
                public static final IdleMode climberIdleMode = IdleMode.kBrake;
                public static final int climberContinuousCurrentLimit = 60;
                public static double jogSpeed = .25;

        }

}