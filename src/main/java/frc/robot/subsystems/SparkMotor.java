package frc.robot.subsystems;

import static frc.robot.Util.logf;
import static frc.robot.Util.round2;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.ShowPID;
import frc.robot.subsystems.LedSubsystem.Leds;
import frc.robot.utilities.LimitSwitch;

/**
 * REV Smart Motion Guide
 * 
 * The SPARK MAX includes a control mode, REV Smart Motion which is used to
 * control the position of the motor, and includes a max velocity and max
 * acceleration parameter to ensure the motor moves in a smooth and predictable
 * way. This is done by generating a motion profile on the fly in SPARK MAX and
 * controlling the velocity of the motor to follow this profile.
 * 
 * Since REV Smart Motion uses the velocity to track a profile, there are only
 * two steps required to configure this mode:
 * 1) Tune a velocity PID loop for the mechanism
 * 2) Configure the smart motion parameters
 * 
 * Tuning the Velocity PID Loop
 * 
 * The most important part of tuning any closed loop control such as the
 * velocity
 * PID, is to graph the inputs and outputs to understand exactly what is
 * happening.
 * For tuning the Velocity PID loop, at a minimum we recommend graphing:
 *
 * 1) The velocity of the mechanism (‘Process variable’)
 * 2) The commanded velocity value (‘Setpoint’)
 * 3) The applied output
 *
 * This example will use ShuffleBoard to graph the above parameters. Make sure
 * to
 * load the shuffleboard.json file in the root of this directory to get the full
 * effect of the GUI layout.
 */

public class SparkMotor extends SubsystemBase {
    private int motorID = 0;
    private LimitSwitch limitSwitch;
    private CANSparkMax motor;
    private SparkPIDController pidController;
    private RelativeEncoder distanceEncoder;
    private PID_MAX pid = new PID_MAX();
    private boolean homed = false;
    private double rotationsPerInch = 1;
    private double current = 0;
    private double lastPower = 99;
    private double lastSetPoint = 0;
    private double lastInches = 0;
    private String name = "";
    final private double MAX_CURRENT = 20;
    private int myCount = 0;
    private CommandXboxController driveController;

    enum STATE {
        IDLE, HOMING, READY, OVERCURRENT, OVERCURRENTSTOPPED
    }

    STATE state = STATE.IDLE;
    STATE lastState = null;

    public SparkMotor(String name, int id, CommandXboxController driveController) {
        this.motorID = id;
        this.driveController = driveController;
        this.name = name;
        motor = new CANSparkMax(motorID, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setInverted(false);
        motor.setSmartCurrentLimit(20);
        limitSwitch = new LimitSwitch(motor, "Spark Test", Leds.ElevatorForward, Leds.ElevatorReverse);
        distanceEncoder = motor.getEncoder();
        distanceEncoder.setPosition(0);
        pidController = motor.getPIDController();
        pid.PIDCoefficientsElevator(pidController);
        pid.PIDToMax();
        logf("Spark Motor Setup kP for :%.6f Conversion:%.2f Counts per Rev:%d\n", pid.kP,
                distanceEncoder.getPositionConversionFactor(), distanceEncoder.getCountsPerRevolution());
    }

    public boolean setMotorPos(double inches) {
        if (inches < 0 || inches > 130) {
            logf("****** Error attempted to set position out of range positon:%.1f\n", inches);
            return false;
        }

        double setPoint = inches * rotationsPerInch;
        lastSetPoint = setPoint;
        lastInches = inches;
        logf("Set position:%.2f set point:%3f\n", inches, setPoint);
        pidController.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
        SmartDashboard.putNumber("Elev SP", setPoint);
        SmartDashboard.putNumber("Elev Inch", inches);
        return true;
    }

    public void setPower(double value) {

        if (lastPower != value || value == 0) {
            logf("Elevator set a new power %.2f\n", value);
            motor.set(value);
            lastPower = value;
        }
    }

    public double getLastPositionInches() {
        return lastInches;
    }

    public double getLastSetPoint() {
        return lastSetPoint;
    }

    public void setBrakeMode(CANSparkMax motor, boolean mode) {
        motor.setIdleMode(mode ? IdleMode.kBrake : IdleMode.kCoast);
        logf("Brake mode: %s\n", motor.getIdleMode());
    }

    public double getElevatorPos() {
        return distanceEncoder.getPosition();
    }

    public double getElevatorLastPos() {
        return lastInches;
    }

    public double getElevatorCurrent() {
        return current;
    }

    public boolean atSetPoint() {
        double error = distanceEncoder.getPosition() - lastSetPoint;
        if (Robot.count % 10 == 5) {
            SmartDashboard.putNumber("EleErr", error);
        }
        // Note error is in revolutions
        return Math.abs(error) < .05;
    }

    public boolean getForwardLimitSwitch() {
        return limitSwitch.getForward();
    }

    public boolean getReverseLimitSwitch() {
        return limitSwitch.getReverse();
    }

    public boolean getHomed() {
        return homed;
    }

    public void setHomed(boolean value) {
        homed = value;
    }

    public void setEncoder(double value) {
        distanceEncoder.setPosition(value);
    }

    double lastSetPointForLogging = 0;

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        double x = driveController.getHID().getPOV();
        if (x >= 0) {
            setMotorPos(x / 45);
            if (Robot.count % 5 == 0) {
                logf("Motor %s POV:%.2f Pos:%.2f Volts:%.2f\n", name, x, distanceEncoder.getPosition(),
                        motor.getBusVoltage());
            }
        }
        // limitSwitch.periodic();
        if (atSetPoint() && lastSetPointForLogging != lastSetPoint) {
            logf("Elevator at set point:%.2f requested set point:%.2f\n", distanceEncoder.getPosition(), lastSetPoint);
            setPower(0);
            lastSetPointForLogging = lastSetPoint;
        }
        current = getElevatorCurrent();
        doHoming(current);
        // pidController.setReference(lastElevatorPosition,
        // CANSparkMax.ControlType.kSmartMotion);
        if (Robot.count % 15 == 8) {
            SmartDashboard.putNumber("ElevCur", round2(current));
            SmartDashboard.putNumber("ElevPos", round2(getElevatorPos()));
            SmartDashboard.putNumber("ElevLastPos", lastInches);
            SmartDashboard.putNumber("ElevPwr", round2(motor.getAppliedOutput()));
            SmartDashboard.putNumber("ElevVel", round2(distanceEncoder.getVelocity()));
        }
        if (RobotContainer.showPID == ShowPID.ELEVATOR && Robot.count % 15 == 12) {
            if (Robot.count % 15 == 12) {

            }
        }
    }

    private void doHoming(double current) {
        if (state != lastState) {
            logf("State Changed state:%s current:%.3f myCount:%d\n", state.toString(), current, myCount);
            SmartDashboard.putString("State", state.toString());
            lastState = state;
        }
        switch (state) {
            case IDLE:
                state = STATE.HOMING;
                break;
            case HOMING:
                if (getReverseLimitSwitch()) {
                    // At home so stop motor and indicate homed
                    setPower(0);
                    setHomed(true);
                    setEncoder(0);
                    state = STATE.READY;
                    logf("Elevator is homed\n");
                    break;
                }
                if (current > MAX_CURRENT) {
                    logf("Overcurrent detected while homing current:%.2f\n", current);
                    myCount = 5; // Wait 100 ms to see if over current remains
                    state = STATE.OVERCURRENT;
                    break;
                }
                setPower(.2);
                break;
            case READY:
                if (current > MAX_CURRENT) {
                    logf("Overcurrent detected while ready current:%.2f\n", current);
                    myCount = 5; // Wait 100 ms to see if over current remains
                    state = STATE.OVERCURRENT;
                }
                break;
            case OVERCURRENT:
                myCount--;
                if (myCount < 0) {
                    setPower(0);
                    myCount = 20; // Wait 400 ms to restart
                    state = STATE.OVERCURRENTSTOPPED;
                }
                break;
            case OVERCURRENTSTOPPED:
                myCount--;
                if (myCount < 0) {
                    if (current > MAX_CURRENT) {
                        // If curent remains high continue to wait
                        logf("***** Elevator current remains high -- current:%.2f\n", current);
                        myCount = 20; // Wait another 400 ms for current to go low
                        break;
                    }
                    // Current seems to have stablize restore last task
                    if (!getHomed()) {
                        state = STATE.IDLE;
                        break;
                    }
                }
        }
    }
}
