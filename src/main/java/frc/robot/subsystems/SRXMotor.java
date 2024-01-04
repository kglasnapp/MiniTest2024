package frc.robot.subsystems;

import static frc.robot.utilities.Util.logf;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;

public class SRXMotor extends SubsystemBase {

    private int id;
    private String name = "";
    private WPI_TalonSRX motor;
    private CommandXboxController driveController;
    private int controllerAxis;
    private FeedbackDevice feedBackDevice = FeedbackDevice.QuadEncoder;
    private boolean sensorPhase = false;
    private boolean positionMode;
    private PID_SRX pid;

    public SRXMotor(String name, int id, CommandXboxController drivecontroller, int controllerAxis,
            boolean positionMode) {
        this.id = id;
        this.name = name;
        this.driveController = drivecontroller;
        this.controllerAxis = controllerAxis;
        this.positionMode = positionMode;
        motor = new WPI_TalonSRX(this.id);
        motor.configFactoryDefault(20);
        logf("Created SRX Motor %s id:%d mode:%s\n", name, id, positionMode ? "Position" : "Percent");
        motor.setSafetyEnabled(false);
        setSensorPhase(true);
        if (this.positionMode) {
            pid = new PID_SRX(name, .1, 0, 0, 0,0, -.2, .2, true);
            setPositionPID(pid, feedBackDevice);
        }
    }

    @Override
    public void periodic() {
        double enc = motor.getSelectedSensorPosition();
        SmartDashboard.putNumber(name + " enc", enc);
        if (positionMode) {
            double setPoint = driveController.getHID().getPOV() * 1000;
            if (setPoint >= 0) {
                motor.set(ControlMode.Position, setPoint);
                if (Robot.count % 5 == 0) {
                    logf("Motor %s serPoint:%.2f Pos:%.2f Volts:%.2f\n", name, setPoint,
                            enc, motor.getBusVoltage());
                }

            }
        } else {
            double x = driveController.getHID().getRawAxis(controllerAxis);
            motor.set(ControlMode.PercentOutput, x);
            if (Robot.count % 5 == 0 && Math.abs(x) > .03) {
                logf("Motor %s Control:%.2f Pos:%.2f Volts:%.2f\n", name, x, enc,
                        motor.getBusVoltage());
            }
        }
    }

    public void setPositionPID(int pidIdx, PID_SRX pid) {
        // Config the sensor used for Primary PID and sensor direction
        motor.configSelectedFeedbackSensor(feedBackDevice, pidIdx, 20);

        // Ensure sensor is positive when output is positive
        motor.setSensorPhase(sensorPhase);

        // Set based on what direction you want forward/positive to be.
        // This does not affect sensor phase.
        // motor.setInverted(motorInvert);

        /* Config the peak and nominal outputs, 12V means full */
        motor.configNominalOutputForward(0, 20);
        motor.configNominalOutputReverse(0, 20);
        motor.configPeakOutputForward(pid.kMaxOutput, 20);
        motor.configPeakOutputReverse(pid.kMinOutput, 20);

        // Config the allowable closed-loop error, Closed-Loop output will be neutral
        // within this range. See Table in Section 17.2.1 for native units per rotation.
        motor.configAllowableClosedloopError(0, pidIdx, 20);
    }

    public void setSensorPhase(boolean phase) {
        sensorPhase = phase;
        motor.setSensorPhase(phase);
    }

    public void setPositionPID(PID_SRX pid, FeedbackDevice feedBack) {
        feedBackDevice = feedBack;
        setPositionPID(0, pid);
        PIDToMotor(pid, 0, 20);
    }

    public void PIDToMotor(PID_SRX pid, int slot, int timeout) {
        motor.config_kP(slot, pid.kP, timeout);
        motor.config_kI(slot, pid.kI, timeout);
        motor.config_kD(slot, pid.kD, timeout);
        motor.config_kF(slot, pid.kFF, timeout);
        motor.configPeakOutputForward(pid.kMaxOutput);
        motor.configPeakOutputReverse(pid.kMinOutput);
        motor.config_IntegralZone(slot, (int) pid.kIz, timeout);
        motor.configAllowableClosedloopError(slot, pid.allowableCloseLoopError, timeout);
        motor.configMaxIntegralAccumulator(slot, pid.maxIntegralAccumulation, timeout);
        logf("Setup %s PID for %s slot %d %s\n", pid.name, name, slot, pid.getPidData());
    }

    public double getError() {
        return motor.getClosedLoopError(0);
    }

}
