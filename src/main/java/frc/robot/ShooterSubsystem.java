package frc.robot;

import static frc.robot.Util.logf;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
public class ShooterSubsystem extends SubsystemBase {
    private Robot robot;

    private ShootMotor lowerMotor;
    private ShootMotor upperMotor;
    private PID_MAX pid = new PID_MAX();
    private final static int OVER_CURRENT = 30;
    private int SHOOTER_MOTOR_ID1 = 21;
    private int SHOOTER_MOTOR_ID3 = 26;


    public ShooterSubsystem(Robot robot) {
        this.upperMotor = new ShootMotor(SHOOTER_MOTOR_ID1, false);
        this.lowerMotor = new ShootMotor(SHOOTER_MOTOR_ID3, true);
        this.robot = robot;
    }

    class ShootMotor {
        CANSparkFlex motor;
        SparkPIDController pidController;
        RelativeEncoder encoder;
        int id;

        ShootMotor(int id, boolean inverted) {
            this.id = id;
            motor = new CANSparkFlex(id, MotorType.kBrushless);
            motor.restoreFactoryDefaults();
            motor.setInverted(inverted);
            setBrakeMode(false);

            motor.setSmartCurrentLimit((int) OVER_CURRENT);
            encoder = motor.getEncoder();

            pidController = motor.getPIDController();
            pid.PIDCoefficientsShoot(pidController);
            pid.PIDToMax();

            setShooterVelocity(0);
        }

        public void setShooterVelocity(double value) {
            if (value >= 0) {
                logf("Set shooter velocity:%.3f\n", value);
            }
            pidController.setReference(value * 6500, CANSparkBase.ControlType.kSmartVelocity);

        }

        void setSpeed(double power) {
            motor.set(power);
        }

        void setBrakeMode(boolean mode) {
            motor.setIdleMode(mode ? IdleMode.kBrake : IdleMode.kCoast);
            logf("For id:%d Brake mode:%s\n", id, motor.getIdleMode());
        }
    }

    void setAllShooterPower(double power) {
        upperMotor.setShooterVelocity(power);
        lowerMotor.setShooterVelocity(power);
    }

    @Override
    public void periodic() {
        if (robot.cont.getAButton()) {
            setAllShooterPower(.5);
        }
        if (robot.cont.getBButton()) {
            setAllShooterPower(0.7);
        }
        if (robot.cont.getYButton()) {
            setAllShooterPower(0.3);
        }
        if (robot.cont.getXButton()) {
            upperMotor.setSpeed(0);
            lowerMotor.setSpeed(0);
        }
    }
}
