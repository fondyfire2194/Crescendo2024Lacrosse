package frc.robot.commands.Shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.FieldRelativeAccel;
import frc.robot.utils.FieldRelativeSpeed;

public class SmartShooter extends Command {
    private final ShooterSubsystem m_shooter;

    private final SwerveSubsystem m_drive;
    private final ArmSubsystem m_arm;
    private final LimelightVision m_llv;
    private final XboxController m_driver;

    private final Timer m_timer = new Timer();

    private FieldRelativeSpeed m_fieldRelVel = new FieldRelativeSpeed();
    private FieldRelativeSpeed m_lastFieldRelVel = new FieldRelativeSpeed();

    public static final InterpolatingDoubleTreeMap kTimeTable = new InterpolatingDoubleTreeMap();

    public SmartShooter(ShooterSubsystem shooter, SwerveSubsystem drive, ArmSubsystem arm,
            LimelightVision llv, XboxController driver) {
        m_shooter = shooter;
        m_drive = drive;
        m_arm = arm;
        m_llv = llv;
        m_driver = driver;
        addRequirements(shooter, drive);
    }

    public SmartShooter(ShooterSubsystem shooter, SwerveSubsystem drive, ArmSubsystem arm, LimelightVision llv) {
        m_shooter = shooter;
        m_drive = drive;
        m_arm = arm;
        m_llv = llv;
        m_driver = new XboxController(4);
        addRequirements(shooter, arm);
    }

    @Override
    public void initialize() {
        // m_arm.trackTarget(true);
        m_timer.reset();
        m_timer.start();
        SmartDashboard.putNumber("SetArmAdjust", 0.0);
        SmartDashboard.putNumber("SetShotAdjust", 0);
        SmartDashboard.putBoolean("Adjust Shot?", false);
    }

    @Override
    public void execute() {

        double currentTime = m_timer.get();

        SmartDashboard.putNumber("Current Time", currentTime);

        SmartDashboard.putBoolean("Shooter Running", m_shooter.getRunShooter());

        FieldRelativeSpeed robotVel = new FieldRelativeSpeed(m_drive.getSpeeds(), m_drive.getHeading());

        FieldRelativeAccel robotAccel = new FieldRelativeAccel(robotVel, m_lastFieldRelVel, .02);

        m_lastFieldRelVel = m_fieldRelVel;

        double robotToTagDist = m_llv.getDistanceFromSpeakerTag(CameraConstants.frontLeftCamera);

        SmartDashboard.putNumber("Calculated Dist", robotToTagDist);

        double shotTime = Constants.shotTimeMap.get(robotToTagDist);

        SmartDashboard.putNumber("Fixed Time", shotTime);

        Translation2d movingGoalLocation = new Translation2d();

        Translation2d fixedTarget = m_llv.getAllianceSpeakerTranslation();

        for (int i = 0; i < 5; i++) {

            double virtualGoalX = fixedTarget.getX()
                    - shotTime * (robotVel.vx + robotAccel.ax * ShooterConstants.kAccelCompFactor);
            double virtualGoalY = fixedTarget.getY()
                    - shotTime * (robotVel.vy + robotAccel.ay * ShooterConstants.kAccelCompFactor);

            SmartDashboard.putNumber("Goal X", virtualGoalX);
            SmartDashboard.putNumber("Goal Y", virtualGoalY);

            Translation2d testGoalLocation = new Translation2d(virtualGoalX, virtualGoalY);

            Translation2d toTestGoal = testGoalLocation.minus(m_drive.getPose().getTranslation());

            double newShotTime = Constants.shotTimeMap.get(toTestGoal.getDistance(new Translation2d()));

            if (Math.abs(newShotTime - shotTime) <= 0.010) {
                i = 4;
            }

            if (i == 4) {
                movingGoalLocation = testGoalLocation;
                SmartDashboard.putNumber("NewShotTime", newShotTime);
            } else {
                shotTime = newShotTime;
            }

        }

        double newDist = movingGoalLocation
                .minus(m_drive.getPose().getTranslation()).getDistance(new Translation2d());

        SmartDashboard.putNumber("NewDist", newDist);

        m_drive.alignToAngle(newDist);
        m_arm.trackDistance(newDist);
        m_shooter.commandRPM = m_shooter.rpmTrackDistance(newDist);

        if (SmartDashboard.getBoolean("Adjust Shot?", false)) {
            m_shooter.setCommandRPM(
                    Constants.shooterRPMMap.get(robotToTagDist) + SmartDashboard.getNumber("SetShotAdjust", 0));
            m_arm.setGoal(Constants.armAngleMap.get(newDist) + SmartDashboard.getNumber("SetArmAdjust", 0));
        } else {
            m_shooter.setCommandRPM(Constants.shooterRPMMap.get(newDist));
            m_arm.setGoal(Constants.armAngleMap.get(newDist));

        }

        // if (m_turret.closeToDeadzone()) {
        // m_driver.setRumble(RumbleType.kLeftRumble, 1.0);
        // m_driver.setRumble(RumbleType.kRightRumble, 1.0);
        // } else {
        // m_driver.setRumble(RumbleType.kLeftRumble, 0.0);
        // m_driver.setRumble(RumbleType.kRightRumble, 0.0);
        // }

    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("Shooter Running", false);
        m_shooter.stopMotors();
        m_arm.stop();
        m_timer.stop();
        m_driver.setRumble(RumbleType.kLeftRumble, 0.0);
        m_driver.setRumble(RumbleType.kRightRumble, 0.0);
    }

    private Pose2d calcPoseFromVision(double dL, double tR, double tT, double tL, Translation2d goal) {
        double tG = tR + tT + tL;
        double rX = goal.getX() - dL * Math.cos(tG);
        double rY = goal.getY() - dL * Math.sin(tG);

        return new Pose2d(rX, rY, new Rotation2d(-tR));
    }

}
