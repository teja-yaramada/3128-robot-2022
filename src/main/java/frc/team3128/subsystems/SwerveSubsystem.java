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
import com.ctre.phoenix.sensors.*;
import com.fasterxml.jackson.databind.ser.std.StdArraySerializers.IntArraySerializer;

import frc.team3128.simulation.FieldSim;



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
    private WPI_PigeonIMU pGyro = new WPI_PigeonIMU(4);
    private FieldSim m_sim;

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
        m_sim = new FieldSim(this);
    }

    public static synchronized SwerveSubsystem getInstance() {
        if (instance == null) {
            instance = new SwerveSubsystem();
        }
        return instance;
    }

    public void zeroHeading(){
        pGyro.reset();
    }

    public double getHeading(){
        return Math.IEEEremainder(pGyro.getAngle(), 360);
    }

    public void resetPose(Pose2d poseMeters) {
        odometry.resetPosition(poseMeters, Rotation2d.fromDegrees(getHeading()));
    }

    /**
     * Reset pose to (x = 0, y = 0, theta = 0)
     */
    public void resetPose() {
        resetGyro();
        resetPose(new Pose2d(0, 0, Rotation2d.fromDegrees(getHeading())));
    }

    public void resetGyro() {
        pGyro.reset();
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(getHeading());
    }

    public void resetOdometry(Pose2d pose, Rotation2d rotation) {
        odometry.resetPosition(pose, rotation);

        frontLeftModule.setPose(pose);
        frontRightModule.setPose(pose);
        backLeftModule.setPose(pose);
        backRightModule.setPose(pose);

        frontLeftModule.resetEncoders();
        frontRightModule.resetEncoders();
        backLeftModule.resetEncoders();
        backRightModule.resetEncoders();
    }

    public Pose2d[] getModulePoses() {
        Pose2d[] modulePoses = {
            frontLeftModule.getPose(),
            frontRightModule.getPose(),
            backLeftModule.getPose(),
            backRightModule.getPose()
        };
        return modulePoses;
    }

    @Override
    public void simulationPeriodic() {
        SwerveModuleState[] moduleStates = {
            frontLeftModule.getState(),
            frontRightModule.getState(),
            backLeftModule.getState(),
            backRightModule.getState()
        };

        m_sim.simulationPeriodic();
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