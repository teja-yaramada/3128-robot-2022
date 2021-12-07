package frc.team3128.commands;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team3128.common.limelight.LEDMode;
import frc.team3128.common.limelight.Limelight;
import frc.team3128.common.limelight.LimelightKey;
import frc.team3128.subsystems.Constants;
import frc.team3128.subsystems.NAR_Drivetrain;

public class CmdAlign extends CommandBase {

    NAR_Drivetrain m_drive;
    Limelight m_limelight;

    private Set<Subsystem> requirements;

    private double txThreshold = Constants.VisionContants.TX_THRESHOLD;

    private double goalHorizontalOffset, currHorizontalOffset;
    private double prevError, currError;
    
    private double prevTime, currTime; // seconds
    private int plateauCount, targetFoundCount;

    private int count = 0;

    private boolean isAligned;

    private enum HorizontalOffsetFeedBackDriveState {
        SEARCHING, FEEDBACK;
    }

    private HorizontalOffsetFeedBackDriveState aimState = HorizontalOffsetFeedBackDriveState.SEARCHING;

    public CmdAlign(NAR_Drivetrain drive, Limelight limelight) {
        m_drive = drive;
        m_limelight = limelight;

        this.goalHorizontalOffset = Constants.VisionContants.TX_OFFSET;

        isAligned = false;

        requirements = new HashSet<Subsystem>();
        requirements.add(m_drive);
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return requirements;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_limelight.setLEDMode(LEDMode.ON);
        prevTime = RobotController.getFPGATime() /1e6;
        plateauCount = 0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_limelight.setLEDMode(LEDMode.ON);
        NetworkTableInstance.getDefault().getTable("limelight-sog").getEntry("ledMode").setNumber(3);

        SmartDashboard.putString("Limelight LED mode", m_limelight.getLedMode());;

        currTime = RobotController.getFPGATime() /1e6;

        switch(aimState) {
            case SEARCHING:
                if(m_limelight.hasValidTarget())
                    targetFoundCount++;
                else
                    targetFoundCount = 0;

                if(targetFoundCount > 5) {
                    currHorizontalOffset = m_limelight.getValue(LimelightKey.HORIZONTAL_OFFSET, Constants.VisionContants.SAMPLE_RATE);

                    prevError = goalHorizontalOffset - currHorizontalOffset;

                    aimState = HorizontalOffsetFeedBackDriveState.FEEDBACK;
                }
                break;
            
            case FEEDBACK:
                if(!m_limelight.hasValidTarget()) {
                    aimState = HorizontalOffsetFeedBackDriveState.SEARCHING;
                    break;
                }

                currHorizontalOffset = m_limelight.getValue(LimelightKey.HORIZONTAL_OFFSET, Constants.VisionContants.SAMPLE_RATE);
                currError = goalHorizontalOffset - currHorizontalOffset;

                SmartDashboard.putNumber("CmdAlign currError", currError);

                if (txThreshold < Constants.VisionContants.TX_THRESHOLD_MAX) {
                    txThreshold += (currTime - prevTime) * (Constants.VisionContants.TX_THRESHOLD_INCREMENT);
                }

                SmartDashboard.putNumber("CmdAlign txThreshold", txThreshold);

                double feedbackPower = Constants.VisionContants.VISION_PID_kP * currError + Constants.VisionContants.VISION_PID_kD * (currError - prevError) / (currTime - prevTime);

                if(feedbackPower > 1)
                    feedbackPower = 1;
                else if(feedbackPower < -1)
                    feedbackPower = -1;

                SmartDashboard.putNumber("CmdAlign feedbackPower", feedbackPower);

                m_drive.tankDrive(-feedbackPower, feedbackPower);

                prevError = currError;

                if (Math.abs(currError) < txThreshold) {
                    plateauCount++;

                    SmartDashboard.putNumber("CmdAlign plateauCount", plateauCount);

                    if(plateauCount > Constants.VisionContants.ALIGN_PLATEAU_COUNT) {
                        isAligned = true;
                    }
                }
                else {
                    isAligned = false;
                    plateauCount = 0;
                }

                break;
                
        }

        prevTime = currTime;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_limelight.setLEDMode(LEDMode.OFF);
        m_drive.stop();

        count++;

        SmartDashboard.putString("CmdAlign", "this has ended " + count);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return isAligned;
    }

}