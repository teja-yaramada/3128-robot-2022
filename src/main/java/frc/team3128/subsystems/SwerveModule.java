package frc.team3128.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.team3128.common.hardware.motorcontroller.NAR_TalonFX;

import static frc.team3128.Constants.ModuleConstants.*;
import static frc.team3128.Constants.DriveConstants.*;

public class SwerveModule {
    

    private final NAR_TalonFX driveMotor;
    private final NAR_TalonFX turningMotor;

    private final PIDController turningPidController;

    private final CANCoder absoluteEncoder;
    private final boolean absoluteEncoderInverted;
    private final double absoluteEncoderOffsetRad;

    private Pose2d modulePose;

    public SwerveModule(int driveMotorID, 
                        boolean driveMotorInverted, 
                        int turningMotorID, 
                        boolean turningMotorInverted,
                        int absoluteEncoderID, 
                        double absoluteEncoderOffsetRad, 
                        boolean absoluteEncoderInverted){
                
        driveMotor = new NAR_TalonFX(driveMotorID);
        turningMotor = new NAR_TalonFX(turningMotorID);

        driveMotor.setInverted(driveMotorInverted);
        turningMotor.setInverted(turningMotorInverted);

        absoluteEncoder = new CANCoder(absoluteEncoderID);
        this.absoluteEncoderInverted = absoluteEncoderInverted;
        this.absoluteEncoderOffsetRad = absoluteEncoderOffsetRad;

        //driveEncoder = driveMotor.getEncoder();
        //turningEncoder = turningMotor.getEncoder();

        turningPidController = new PIDController(kPTurning, kITurning, kDTurning);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        modulePose = new Pose2d();

        resetEncoders();
                
    }

    public double getDrivePosition(){
        return driveMotor.getSelectedSensorPosition() * kDriveEncoderRot2Meter;
    }

    public double getTurningPosition(){
        return turningMotor.getSelectedSensorPosition() * kDriveEncoderRot2Meter;
    }

    public double getDriveVelocity(){
        return driveMotor.getSelectedSensorPosition() * kDriveEncoderRPM2MetersPerSec;
    }

    public double getTurningVelocity(){
        return turningMotor.getSelectedSensorPosition() * kDriveEncoderRPM2MetersPerSec;
    }

    public double getAbsoluteEncoderRad(){
        double angle = absoluteEncoder.getAbsolutePosition();
        angle *= Math.PI / 180;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderInverted ? -1.0 : 1.0);
    }

    public void resetEncoders(){
        driveMotor.setEncoderPosition(0);
        turningMotor.setEncoderPosition(getAbsoluteEncoderRad());
    }

    public Pose2d getPose() {
        return modulePose;
      }
    
      public void setPose(Pose2d pose) {
        modulePose = pose;
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
        driveMotor.set(state.speedMetersPerSecond / MAX_DRIVE_VELOCITY);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
    }

    public void stop(){
        driveMotor.set(0);
        turningMotor.set(0);
    }

}
