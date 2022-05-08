package frc.team3128.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3128.Constants.DriveConstants;
import frc.team3128.Constants.ModuleConstants;
import frc.team3128.common.hardware.motorcontroller.NAR_TalonFX;
import static frc.team3128.Constants.ModuleConstants.*;

public class SwerveModule {
    

    private final NAR_TalonFX driveMotor;
    private final NAR_TalonFX turningMotor;

    private final CANCoder driveEncoder;
    private final CANCoder turningEncoder;

    private final PIDController turningPidController;

    private final CANCoder absoluteEncoder;
    private final boolean absoluteEncoderInverted;
    private final double absoluteEncoderOffsetRad;

    private final double kRot2Meter = kDriveEncoderRot2Meter;
    private final double kRPM2MetPerSec = kDriveEncoderRPM2MetersPerSec;
    private final double kMaxSpeedMetPerSec = kDriveEncoderRPM2MetersPerSec;

    public SwerveModule(int driveMotorID, boolean driveMotorInverted, int turningMotorID, boolean turningMotorInverted,
            int absoluteEncoderID, double absoluteEncoderOfsfsetRad, boolean absoluteEncoderInverted){
                
        driveMotor = new NAR_TalonFX(driveMotorID);
        turningMotor = new NAR_TalonFX(turningMotorID);

        driveMotor.setInverted(driveMotorInverted);
        turningMotor.setInverted(turningMotorInverted);

        absoluteEncoder = new CANCoder(absoluteEncoderID);
        this.absoluteEncoderInverted = absoluteEncoderInverted;
        this.absoluteEncoderOffsetRad = absoluteEncoderOfsfsetRad;

        //driveEncoder = driveMotor.getEncoder();
        //turningEncoder = turningMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(kDriveEncoderRPM2MetersPerSec);
        turningEncoder.setPositionConversionFactor(kDriveEncoderRot2Meter);
        turningEncoder.setVelocityConversionFactor(kDriveEncoderRPM2MetersPerSec);

        turningPidController = new PIDController(kPTurning, kITurning, kDTurning);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
                
    }

    public double getDrivePosition(){
        return driveEncoder.getPosition() * Rot2Meter;
    }

    public double getTurningPosition(){
        return turningEncoder.getPosition() * Rot2Meter;
    }

    public double getDriveVelocity(){
        return driveEncoder.getVelocity() * RPM2MetPerSec;
    }

    public double getTurningVelocity(){
        return turningEncoder.getVelocity() * RPM2MetPerSec;
    }

    public double getAbsoluteEncoderRad(){
        double angle = absoluteEncoder.getAbsolutePosition();
        angle *= Math.PI / 180;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderInverted ? -1.0 : 1.0);
    }

    public void resetEncoders(){
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state){

        if(Math.abs(state.speedMetersPerSecond)<0.001){
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / );
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
    }

    public void stop(){
        driveMotor.set(0);
        turningMotor.set(0);
    }

}
