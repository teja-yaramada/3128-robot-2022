package frc.team3128.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.team3128.Constants.DriveConstants.*;
import static frc.team3128.Constants.SwerveConstants.*;

import com.kauailabs.navx.frc.AHRS;



public class SwerveSubsystem extends SubsystemBase{

    private static SwerveSubsystem instance;

    private final SwerveModule frontLeftModule = new SwerveModule(
        kFrontLeftDriveMotorPort, 
        kFrontLeftDriveEncoderReversed, 
        kFrontLeftTurningMotorPort, 
        kFrontLeftTurningEncoderReversed, 
        kFrontLeftDriveAbsoluteEncoderPort, 
        kFrontLeftDriveAbsoluteEncoderOffsetRad, 
        kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRightModule = new SwerveModule(
        kFrontRightDriveMotorPort, 
        kFrontRightDriveEncoderReversed, 
        kFrontRightTurningMotorPort, 
        kFrontRightTurningEncoderReversed, 
        kFrontRightDriveAbsoluteEncoderPort, 
        kFrontRightDriveAbsoluteEncoderOffsetRad, 
        kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeftModule = new SwerveModule(
        kBackLeftDriveMotorPort, 
        kBackLeftDriveEncoderReversed, 
        kBackLeftTurningMotorPort, 
        kBackLeftTurningEncoderReversed, 
        kBackLeftDriveAbsoluteEncoderPort, 
        kBackLeftDriveAbsoluteEncoderOffsetRad, 
        kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRightModule = new SwerveModule(
        kBackRightDriveMotorPort, 
        kBackRightDriveEncoderReversed, 
        kBackRightTurningMotorPort, 
        kBackRightTurningEncoderReversed, 
        kBackRightDriveAbsoluteEncoderPort, 
        kBackRightDriveAbsoluteEncoderOffsetRad, 
        kBackRightDriveAbsoluteEncoderReversed);


    private AHRS gyro =  new AHRS(SPI.Port.kMXP);

    private SwerveDriveOdometry odometry;

    public SwerveSubsystem(){
        new Thread(() -> {
            try{
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();

        odometry = new SwerveDriveOdometry(kSwerveDriveKinematics, Rotation2d.fromDegrees(getHeading()));
    }

    public static synchronized SwerveSubsystem getInstance() {
        if (instance == null) {
            instance = new SwerveSubsystem();
        }
        return instance;
    }

    public void zeroHeading(){
        gyro.reset();
    }

    public double getHeading(){
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(getHeading());
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Robot Heading", getHeading());
    }

    public void stop(){
        frontLeftModule.stop();
        frontRightModule.stop();
        backLeftModule.stop();
        backRightModule.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, kPhysicalMaxSpeedMetersPerSecond);
        
        frontLeftModule.setDesiredState(desiredStates[0]);
        frontRightModule.setDesiredState(desiredStates[1]);
        backLeftModule.setDesiredState(desiredStates[2]);
        backRightModule.setDesiredState(desiredStates[3]);
    }

}