package frc.team3128.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3128.Robot;
import frc.team3128.common.NAR_EMotor;
import frc.team3128.common.Simulable;
import frc.team3128.hardware.NAR_CANSparkMax;
import frc.team3128.hardware.NAR_Motor;
import frc.team3128.hardware.NAR_TalonFX;
import net.thefletcher.revrobotics.enums.MotorType;

import static frc.team3128.hardware.NAR_Motor.MotorControllerType;

public class NAR_Drivetrain extends SubsystemBase implements Simulable{

    // Initialize the generic motors
    // TODO: Weird difference in speed for different motors

    private NAR_TalonFX leftLeader, rightLeader, leftFollower, rightFollower;

    public static NAR_Drivetrain instance;

    private DifferentialDrive robotDrive;
    private DifferentialDrivetrainSim robotDriveSim;
    private DifferentialDriveOdometry odometry;

    // TODO: Abstractify gyro
    private static AHRS gyro = new AHRS(SPI.Port.kMXP);;

    private static Field2d field;

    public NAR_Drivetrain(){

        // TODO: Initialize motors here from parameters
        leftLeader = (NAR_TalonFX) NAR_Motor.create(Constants.DriveConstants.DRIVE_MOTOR_LEFT_LEADER_ID, MotorControllerType.TALON_FX);
        rightLeader = (NAR_TalonFX) NAR_Motor.create(Constants.DriveConstants.DRIVE_MOTOR_RIGHT_LEADER_ID, MotorControllerType.TALON_FX);
        leftFollower = (NAR_TalonFX) NAR_Motor.create(Constants.DriveConstants.DRIVE_MOTOR_LEFT_FOLLOWER_ID, MotorControllerType.TALON_FX);
        rightFollower = (NAR_TalonFX) NAR_Motor.create(Constants.DriveConstants.DRIVE_MOTOR_RIGHT_FOLLOWER_ID, MotorControllerType.TALON_FX);

        // Not sure what the deal is here
        leftFollower.follow(leftLeader);
        //leftFollower.setInverted(InvertType.FollowMaster);
        rightFollower.follow(rightLeader);
        //rightFollower.setInverted(InvertType.FollowMaster);

        robotDrive = new DifferentialDrive(leftLeader.getMotorController(), rightLeader.getMotorController());

        // if(Robot.isSimulation()){
        //     robotDriveSim =
        //     new DifferentialDrivetrainSim(
        //         Constants.DriveConstants.DRIVE_CHAR,
        //         Constants.DriveConstants.GEARBOX,
        //         Constants.DriveConstants.DRIVE_GEARING,
        //         Constants.DriveConstants.TRACK_WIDTH_METERS,
        //         Constants.DriveConstants.WHEEL_RADIUS_METERS, 
        //         null/*VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005)*/);
        // }

        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
        field = new Field2d();

        SmartDashboard.putData("Field", field);

        resetEncoders();
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
    }

    public void simulationPeriodic() {
        
        // Set motor voltage inputs
        robotDriveSim.setInputs(
            -leftLeader.getMotorController().getMotorOutputVoltage(),
            rightLeader.getMotorController().getMotorOutputVoltage()
        );

        // Update sim environment
        robotDriveSim.update(0.02);

        // Store simulated motor states
        leftLeader.setQuadSimPosition(robotDriveSim.getLeftPositionMeters() / Constants.DriveConstants.ENCODER_DISTANCE_PER_MARK);
        leftLeader.setQuadSimVelocity(robotDriveSim.getLeftVelocityMetersPerSecond()/(Constants.DriveConstants.ENCODER_DISTANCE_PER_MARK * 10));
        rightLeader.setQuadSimPosition(robotDriveSim.getRightPositionMeters() / Constants.DriveConstants.ENCODER_DISTANCE_PER_MARK);
        rightLeader.setQuadSimVelocity(robotDriveSim.getRightVelocityMetersPerSecond()/(Constants.DriveConstants.ENCODER_DISTANCE_PER_MARK * 10));

        SmartDashboard.putNumber("Left Speed", leftLeader.getEncoderVelocity());
        SmartDashboard.putNumber("Left Desired Speed", robotDriveSim.getLeftVelocityMetersPerSecond() / (Constants.DriveConstants.ENCODER_DISTANCE_PER_MARK * 10));
        
        // TODO: Abstractify gyro
        int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
        SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
        angle.set(-robotDriveSim.getHeading().getDegrees());
    }
        
    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360) * (Constants.DriveConstants.GYRO_REVERSED ? -1.0 : 1.0);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public double getLeftEncoderDistance() {
        return leftLeader.getEncoderPosition() * Constants.DriveConstants.ENCODER_DISTANCE_PER_MARK;
    }

    public double getRightEncoderDistance() {
        return rightLeader.getEncoderPosition() * Constants.DriveConstants.ENCODER_DISTANCE_PER_MARK;
    }

    public void arcadeDrive(double x, double y) {
        robotDrive.arcadeDrive(x,y);
    }

    public void resetEncoders() {
        leftLeader.setEncoderPosition(0);
        rightLeader.setEncoderPosition(0);
    }

    public void stop() {
        robotDrive.stopMotor();
    }
}

