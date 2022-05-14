package frc.team3128.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.subsystems.SwerveSubsystem;
import static frc.team3128.Constants.OIConstants.*;
import static frc.team3128.Constants.DriveConstants.*;
import static frc.team3128.Constants.SwerveConstants.*;

public class SwerveArcadeDriveCmd extends CommandBase{

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpeedFunction, ySpeedFunction, zRotationFunction, throttleFunction;
    private final SlewRateLimiter xLimiter, yLimiter, zLimiter;

    public SwerveArcadeDriveCmd(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpeedFunction, 
            Supplier<Double> ySpeedFunction, 
            Supplier<Double> zRotationFunction,
            Supplier<Double> throttleFunction){
        this.swerveSubsystem = swerveSubsystem;
        this.xSpeedFunction = xSpeedFunction;
        this.ySpeedFunction = ySpeedFunction;
        this.throttleFunction = throttleFunction;
        this.zRotationFunction = zRotationFunction;
        this.xLimiter = new SlewRateLimiter(kTeleDriveMaxAccUnitsPerSec);
        this.yLimiter = new SlewRateLimiter(kTeleDriveMaxAccUnitsPerSec);
        this.zLimiter = new SlewRateLimiter(kTeleDriveMaxAngularAccUnitsPerSec);
        addRequirements(swerveSubsystem);

    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        //get joystick inputs
        double xSpeed = xSpeedFunction.get();
        double ySpeed = ySpeedFunction.get();
        double zRotation = zRotationFunction.get();
        double throttle = throttleFunction.get();

        //apply deadband
        xSpeed = Math.abs(xSpeed) > kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > kDeadband ? ySpeed : 0.0;
        zRotation = Math.abs(zRotation) > kDeadband ? zRotation : 0.0;

        //rate limiter
        xSpeed = xLimiter.calculate(xSpeed) * throttle * kTeleDriveMaxVelocityMetersPerSec;
        ySpeed = yLimiter.calculate(ySpeed) * throttle * kTeleDriveMaxVelocityMetersPerSec;
        zRotation = zLimiter.calculate(zRotation) * throttle * kTeleMaxDriveVelocityRadiansPerSec;

        //constructing desired chassis speeds
        ChassisSpeeds chassisSpeeds;
            //relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, zRotation, swerveSubsystem.getRotation2d());

        //Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = kSwerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        //Output each module state to wheels
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted){
        swerveSubsystem.stop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
    
}
