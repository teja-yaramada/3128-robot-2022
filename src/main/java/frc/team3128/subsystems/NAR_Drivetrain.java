package frc.team3128.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3128.Constants;
import frc.team3128.Robot;
import frc.team3128.common.NAR_EMotor;
import frc.team3128.common.hardware.motor.NAR_TalonFX;

/**
 * Daniel's note to self 12/18/21: Things to check:
 * How does positive power to both sides make the drivetrain go forward - FOUND: DifferentialDrive automatically inverts the right side of the drivetrain :(
 * Redo drive characterization using https://docs.wpilib.org/en/stable/docs/software/examples-tutorials/trajectory-tutorial/characterizing-drive.html (current values are way off)
 * Why does end heading of trajectory not work
 * Why does reversing trajectory not work (probably related to above)
 * If nothing else works, do this: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/troubleshooting.html
 * Measure wheel radius correctly
 * Try using drive characterization output trackwidth
 */


public class NAR_Drivetrain extends SubsystemBase {

    public static NAR_Drivetrain instance;

    private NAR_EMotor leftLeader, rightLeader, leftFollower, rightFollower;
    
    private DifferentialDrive robotDrive;
    private DifferentialDrivetrainSim robotDriveSim;

    private DifferentialDriveOdometry odometry;
    private AHRS gyro = new AHRS(SPI.Port.kMXP);;

    private Field2d field;

    public NAR_Drivetrain(){

        leftLeader = new NAR_TalonFX(Constants.DriveConstants.DRIVE_MOTOR_LEFT_LEADER_ID);
        rightLeader = new NAR_TalonFX(Constants.DriveConstants.DRIVE_MOTOR_RIGHT_LEADER_ID);
        leftFollower = new NAR_TalonFX(Constants.DriveConstants.DRIVE_MOTOR_LEFT_FOLLOWER_ID);
        rightFollower = new NAR_TalonFX(Constants.DriveConstants.DRIVE_MOTOR_RIGHT_FOLLOWER_ID);

        leftFollower.follow(leftLeader);
        rightFollower.follow(rightLeader);

        robotDrive = new DifferentialDrive(leftLeader.getMotor(), rightLeader.getMotor());

        if(Robot.isSimulation()){
            robotDriveSim =
            new DifferentialDrivetrainSim(
                Constants.DriveConstants.DRIVE_CHAR,
                Constants.DriveConstants.GEARBOX,
                Constants.DriveConstants.DRIVE_GEARING,
                Constants.DriveConstants.TRACK_WIDTH_METERS,
                Constants.DriveConstants.WHEEL_RADIUS_METERS, 
                null/*VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005)*/);
        }

        resetEncoders();
        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
        field = new Field2d();

        SmartDashboard.putData("Field", field);
        
    }

    public static synchronized NAR_Drivetrain getInstance() {
        if (instance == null) {
            instance = new NAR_Drivetrain();
        }
        return instance;
    }

    @Override
    public void periodic() {
        odometry.update(Rotation2d.fromDegrees(getHeading()), getLeftEncoderDistance(), getRightEncoderDistance());
        field.setRobotPose(getPose());

        SmartDashboard.putNumber("DT x", getPose().getX());
        SmartDashboard.putNumber("DT y", getPose().getY());
        SmartDashboard.putNumber("DT angl", getHeading());

        SmartDashboard.putNumber("DT left power", leftLeader.get());
        SmartDashboard.putNumber("DT right power", rightLeader.get());

        SmartDashboard.putNumber("Left enc distance", getLeftEncoderDistance());
        SmartDashboard.putNumber("Right enc distance", getRightEncoderDistance());
        SmartDashboard.putNumber("Left Speed", getLeftEncoderSpeed());


    }

    public void simulationPeriodic() {
        
        // Set motor voltage inputs
        robotDriveSim.setInputs(
            leftLeader.getMotorOutputVoltage(),
            -rightLeader.getMotorOutputVoltage()
        );

        // Update sim environment
        robotDriveSim.update(0.02);

        // Store simulated motor states
        leftLeader.setQuadSimPosition(-robotDriveSim.getLeftPositionMeters() / Constants.DriveConstants.DRIVE_DIST_PER_TICK);
        leftLeader.setQuadSimVelocity(-robotDriveSim.getLeftVelocityMetersPerSecond()/(Constants.DriveConstants.DRIVE_DIST_PER_TICK * 10));
        rightLeader.setQuadSimPosition(robotDriveSim.getRightPositionMeters() / Constants.DriveConstants.DRIVE_DIST_PER_TICK);
        rightLeader.setQuadSimVelocity(robotDriveSim.getRightVelocityMetersPerSecond()/(Constants.DriveConstants.DRIVE_DIST_PER_TICK * 10));

        SmartDashboard.putNumber("Left Desired Speed", robotDriveSim.getLeftVelocityMetersPerSecond() / (Constants.DriveConstants.ENCODER_DISTANCE_PER_MARK * 10));
        
        // TODO: Abstractify gyro
        int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
        SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
        angle.set(robotDriveSim.getHeading().getDegrees()); // @Nathan: I tested this out, this seems to work. This preserves parity w/ the real robot in angle, odometry
    }
        
    public double getHeading() {
        //gyro.getYaw uses CW as positiveP
        return -gyro.getYaw(); // (Math.IEEEremainder(gyro.getAngle(), 360) + 360) % 360;
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetEncoders() {
        leftLeader.setEncoderPosition(0);
        rightLeader.setEncoderPosition(0);
    }

    /**
     * @return Left leader encoder distance in meters
     */
    public double getLeftEncoderDistance() {
        return -leftLeader.getSelectedSensorPosition() * Constants.DriveConstants.DRIVE_DIST_PER_TICK;
    }

    /**
     * @return Right leader encoder distance in meters
     */
    public double getRightEncoderDistance() {
        return rightLeader.getSelectedSensorPosition() * Constants.DriveConstants.DRIVE_DIST_PER_TICK;
    }

    /**
     * @return the left encoder velocity in meters per second
     */
    public double getLeftEncoderSpeed() {
        return -leftLeader.getSelectedSensorVelocity() * Constants.DriveConstants.DRIVE_DIST_PER_TICK * 10;
    }

    /**
     * @return the right encoder velocity in meters per second
     */
    public double getRightEncoderSpeed() {
        return rightLeader.getSelectedSensorVelocity() * Constants.DriveConstants.DRIVE_DIST_PER_TICK * 10;
    }

    public void arcadeDrive(double x, double y) {
        robotDrive.arcadeDrive(x, y, false); // Don't squareInputs
    }

    public void stop() {
        robotDrive.stopMotor();
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        robotDrive.tankDrive(leftSpeed, rightSpeed);
        robotDrive.feed();
    }
    
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftEncoderSpeed(), getRightEncoderSpeed());
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        robotDrive.tankDrive(leftVolts / RobotController.getBatteryVoltage(), rightVolts / RobotController.getBatteryVoltage());
        robotDrive.feed();
    }

    public void resetPose(Pose2d poseMeters) {
        resetEncoders();
        odometry.resetPosition(poseMeters, Rotation2d.fromDegrees(getHeading()));
    }

    public void resetGyro() {
        gyro.reset();
    }
}

