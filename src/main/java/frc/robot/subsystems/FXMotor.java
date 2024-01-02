package frc.robot.subsystems;

import static frc.robot.Robot.count;
import static frc.robot.utilities.Util.logf;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix6.StatusSignal;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;


public class FXMotor {
    private TalonFX motor;
    private TalonFX followMotor;
    private String name;
    private int id;
    private int followId;
    private double lastSpeed = 0;
    private int lastPos = 0;
    private boolean myLogging = false;
    // public ErrorCode errorCode;
    // public ErrorCode errorCodeFollow;
    private StatusSignal<Integer> errorCode;

    private boolean sensorPhase = false;
    private boolean motorInvert = false;
    // private FeedbackDevice feedBackDevice =
    // FeedbackDevice.CTRE_MagEncoder_Relative;

    FXMotor(String name, int id, int followId, boolean logging) {
        this.name = name;
        this.id = id;
        this.followId = followId;
        myLogging = logging;
        motor = new TalonFX(this.id);

        var configuration = new TalonFXConfiguration();

        /*
         * User can optionally change the configs or leave it alone to perform a factory
         * default
         */
        configuration.MotorOutput.Inverted = (motorInvert) ? InvertedValue.CounterClockwise_Positive
                : InvertedValue.Clockwise_Positive;

        motor.getConfigurator().apply(configuration);

        motor.setSafetyEnabled(true);

        errorCode = motor.getFaultField();
        if (errorCode.getValue() != 0) {
            logf("????????? Motor %s Error: %s ??????????\n", name, errorCode.toString());
        }
        if (followId > 0) {
            followMotor = new TalonFX(followId);
            followMotor.setSafetyEnabled(true);
            /* Set up followers to follow leaders */
            motor.setControl(new Follower(followMotor.getDeviceID(), false));
            followMotor.getConfigurator().apply(configuration);
            errorCode = motor.getFaultField();
            // followMotor.set
            if (errorCode.getValue() != 0) {
                logf("????????? Follow Motor %s Error: %s ??????????\n", name, errorCode.toString());
                followId = -followId;
            }

        }

        motor.setPosition(0);
        if (followId > 0)
            logf("Created %s motor ids:<%d,%d> firmware:<%d,%d> voltage:<%.1f,%.1f>\n", name, id, followId,
                    motor.getVersionMajor(), followMotor.getVersionBuild(), motor.getMotorVoltage(),
                    followMotor.getMotorVoltage());
        else
            logf("Created %s motor id:%d firmware:%d voltage:%.1f\n", name, id, motor.getVersionMajor(),
                    motor.getMotorVoltage());

    }

    public String getName() {
        return name;
    }

    public double getPos() {
        StatusSignal<Double> position = motor.getPosition();
        return position.getValue();
    }

    void enableLimitSwitch(boolean forward, boolean reverse) {
        // if (forward)
        // motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
        // LimitSwitchNormal.NormallyOpen);
        // if (reverse)
        // motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
        // LimitSwitchNormal.NormallyOpen);
    }

    boolean getForwardLimitSwitch() {
        return motor.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
    }

    boolean getReverseLimitSwitch() {
        return motor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
    }

    void setBrakeMode(boolean mode) {

        /* Keep a neutral out so we can disable the motor */
        NeutralOut m_brake = new NeutralOut();
        motor.setControl(m_brake);

        if (followId > 0)
            followMotor.setControl(m_brake);
        ;
    }

    void setPos(double position) {
        motor.setPosition(position);
    }

    public void setInverted(boolean invert) {
        this.motorInvert = invert;
        motor.setInverted(invert);
        if (followId > 0) {
            followMotor.setInverted(invert);
        }
    }

    // public void setRampRate(double timeToFull) {
    // motor.configOpenloopRamp(timeToFull);
    // if (followId > 0) {
    // followMotor.configOpenloopRamp(timeToFull);
    // }
    // }

    public double getLastSpeed() {
        return lastSpeed;
    }

    public double getActualSpeed() {
        return motor.getVelocity().getValueAsDouble();
    }

    public TalonFX getMotor() {
        return motor;
    }

    void periodic() {
        if (count % 50 == 0) {
            logPeriodic();
        }
        if (count % 500 == 0)
            updateSmart();
    }

    public void logPeriodic() {
        int pos = (int) getPos();
        if (pos != lastPos) {
            lastPos = pos;
            if (myLogging) {
                if (followId > 0) {
                    logf("%s motor cur:%.2f temp:%.2f vel:%.2f pos:%.0f inv:%b senP:%b\n", name,
                            motor.getStatorCurrent().getValue(), motor.getDeviceTemp().getValue(),
                            motor.getVelocity().getValue(), pos,
                            motorInvert, sensorPhase);
                    logf("%s follow  cur:%.2f temp:%.2f vel:%d pos:%.2f\n", name,
                            followMotor.getStatorCurrent(),
                            followMotor.getDeviceTemp().getValue(), followMotor.getVelocity().getValue(),
                            followMotor.getPosition().getValue());
                } else {
                    logf("%s motor cur:%.2f temp:%.2f vel:%d pos:%d inv:%b senP:%b\n", name,
                            motor.getStatorCurrent().getValue(), motor.getDeviceTemp().getValue(),
                            motor.getVelocity().getValue(),
                            pos, motorInvert, sensorPhase);
                }
            }
        }

    }

    public void setCurrentLimit(int peakAmps, int continousAmps, int durationMilliseconds) {
        /*
         * Peak Current and Duration must be exceeded before current limit is activated.
         * When activated, current will be limited to Continuous Current. Set Peak
         * Current params to 0 if desired behavior is to immediately current-limit.
         */
        // talon.configPeakCurrentLimit(35, 10); /* 35 A */
        // talon.configPeakCurrentDuration(200, 10); /* 200ms */
        // talon.configContinuousCurrentLimit(30, 10); /* 30

        // SupplyCurrentLimitConfiguration(boolean enable, double currentLimit, double
        // triggerThresholdCurrent, double triggerThresholdTime)
        SupplyCurrentLimitConfiguration cl = new SupplyCurrentLimitConfiguration(true, peakAmps, continousAmps,
                durationMilliseconds);
        cl.enable = true;
    }

    public void updateSmart() {
        // SmartDashboard.putNumber(name + " Pos", (int)
        // motor.getSensorCollection().getIntegratedSensorPosition());
        // SmartDashboard.putNumber(name + " Cur", round2(motor.getStatorCurrent()));
    }

    public void setSpeed(double speed) {
        if (speed != lastSpeed) {
            motor.set(speed);
            lastSpeed = speed;
        }
    }

    void zeroEncoder() {
        motor.setPosition(0);
    }

    void setEncoderPosition(double position) {
        motor.setPosition(position);
        ;
    }

    void setPositionPID(PID_SRX pid, FeedbackDevice feedBack) {
        // feedBackDevice = feedBack;
        setPositionPID(motor, 0, pid);
        PIDToFX(motor, pid, 0, 20);
    }

    void setVelocityPID(PID_SRX pid) {
        PIDToFX(motor, pid, 1, 20
        );
    }

    double getMotorVoltage() {
        return motor.getSupplyVoltage().getValue();
    }

    public void PIDToFX(TalonFX srx, PID_SRX pid, int slot, int timeout) {

        // Configure the TalonFX for basic use
        TalonFXConfiguration configs = new TalonFXConfiguration();
        // This TalonFX should be configured with a kP of 1, a kI of 0, a kD of 10, and
        // a kV of 2 on slot 0
        configs.Slot0.kP = 1;
        configs.Slot0.kI = 0;
        configs.Slot0.kD = 10;
        configs.Slot0.kV = 2;

        // Write these configs to the TalonFX
        motor.getConfigurator().apply(configs);

        // srx.config_kP(slot, pid.kP, timeout);
        // srx.config_kI(slot, pid.kI, timeout);
        // srx.config_kD(slot, pid.kD, timeout);
        // srx.config_kF(slot, pid.kFF, timeout);
        // srx.config_IntegralZone(slot, (int) pid.kIz, timeout);
        // srx.configAllowableClosedloopError(slot, pid.allowableCloseLoopError,
        // timeout);
        // srx.configMaxIntegralAccumulator(slot, pid.maxIntegralAccumulation, timeout);
        logf("Setup %s PID for %s slot %d %s\n", pid.name, name, slot, pid.getPidData());
    }

    public double getError() {
        return motor.getClosedLoopError().getValueAsDouble();
    }

    public void logMotorVCS() {
        if (Math.abs(lastSpeed) > .02) {
            logf("%s\n", getMotorsVCS(motor));
            if (followId > 0) {
                logf("%s\n", getMotorsVCS(followMotor));
            }
        }
    }

    public String getMotorsVCS(TalonFX motor) {
        if (Math.abs(lastSpeed) > .02) {
            double voltage = motor.getSupplyVoltage().getValueAsDouble();
            double current = motor.getSupplyCurrent().getValueAsDouble();
            return String.format("%s motor volts:%.2f cur:%.2f power:%.2f sp:%.3f", name,
                    voltage, current, voltage * current, lastSpeed);
        }
        return name + "Not Running";
    }

    public double getMotorCurrent() {
        return motor.getSupplyCurrent().getValue();
    }

    // public void setSensorPhase(boolean phase) {
    // sensorPhase = phase;
    // motor.setSensorPhase(phase);
    // }

    private void setPositionPID(TalonFX talon, int pidIdx, PID_SRX pid) {
        // Config the sensor used for Primary PID and sensor direction

        // talon.configSelectedFeedbackSensor(feedBackDevice, pidIdx,
        // Robot.config.kTimeoutMs);

        // Ensure sensor is positive when output is positive
        // talon.setSensorPhase(sensorPhase);

        // Set based on what direction you want forward/positive to be.
        // This does not affect sensor phase.

        // will need to do for follow motor -- take out and check climber code
        // talon.setInverted(motorInvert);

        /* Config the peak and nominal outputs, 12V means full */
        // talon.configNominalOutputForward(0, Constants.kTimeoutMs);
        // talon.configNominalOutputReverse(0, Constants.kTimeoutMs);
        // talon.configPeakOutputForward(pid.kMaxOutput, Constants.kTimeoutMs);
        // talon.configPeakOutputReverse(pid.kMinOutput, Constants.kTimeoutMs);

        // // Config the allowable closed-loop error, Closed-Loop output will be neutral
        // // within this range. See Table in Section 17.2.1 for native units per
        // rotation.
        // talon.configAllowableClosedloopError(0, pidIdx, Constants.kTimeoutMs);
    }
}