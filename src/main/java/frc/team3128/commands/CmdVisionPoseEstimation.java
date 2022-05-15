package frc.team3128.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.team3128.Constants.FieldConstants.*;
import static frc.team3128.Constants.VisionConstants.*;
import frc.team3128.common.hardware.limelight.Limelight;
import frc.team3128.common.hardware.limelight.LimelightKey;
import frc.team3128.subsystems.NAR_Drivetrain;

public class CmdVisionPoseEstimation extends CommandBase {

    private NAR_Drivetrain drive;
    private Limelight shooterLimelight;

    public CmdVisionPoseEstimation(NAR_Drivetrain drive, Limelight shooterLimelight) {
        this.drive = drive;
        this.shooterLimelight = shooterLimelight;

        // Don't addRequirements(drive) because we don't want to require the drivetrain
    }
    
    @Override
    public void execute() {

        if(!shooterLimelight.hasValidTarget()) {
            return;
        }
        //gets the distance from the hub
        double llDistance = shooterLimelight.calculateDistToGroundTarget(TARGET_HEIGHT) - 11; // Distance to the front of the robot
        //calculating the x and y distance of the robot using the distance from the hub
        Pose2d visionEstimate = visionEstimatedPose(llDistance, shooterLimelight.getValue(LimelightKey.HORIZONTAL_OFFSET, 2), drive.getHeading());

        //if there the difference is less than 1 or the estimated position is outside of the field
        if (visionEstimate.getTranslation().getDistance(drive.getPose().getTranslation()) > 1.0 || translationOutOfBounds(visionEstimate.getTranslation())) {
            return;
        }
        //data is added into the kalman filter to correst odometry
        drive.addVisionMeasurement(visionEstimate, Timer.getFPGATimestamp() - LIMELIGHT_LATENCY);
    }
    //calculates x and y pose
    private Pose2d visionEstimatedPose(double visionDist, double tx, double gyroAngle) {
        double distToHubCenter = visionDist + HUB_RADIUS;
        Rotation2d thetaHub = Rotation2d.fromDegrees(gyroAngle - tx);
        Translation2d fieldPos = new Translation2d(
            -distToHubCenter * Math.cos(thetaHub.getRadians()), 
            -distToHubCenter * Math.sin(thetaHub.getRadians()))
                                    .plus(HUB_POSITION);
        return new Pose2d(fieldPos, Rotation2d.fromDegrees(gyroAngle));
    }

    private boolean translationOutOfBounds(Translation2d translation) {
        return translation.getX() > FIELD_X_LENGTH || translation.getX() < 0 || translation.getY() > FIELD_Y_LENGTH || translation.getY() < 0;
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}