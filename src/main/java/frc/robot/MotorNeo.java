package frc.robot;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;

public class MotorNeo {
    private Robot robot;

    private final CANSparkMax motorTest;
    private final WPI_TalonSRX grabberTest;
    double lastSpeed = 0;

    private final RelativeEncoder m_drivingEncoder;

    /**
     * Constructs a MAXSwerveModule and configures the driving and turning motor,
     * encoder, and PID controller. This configuration is specific to the REV
     * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
     * Encoder.
     */
    public MotorNeo(Robot robot) {
        this.robot = robot;

        motorTest = new CANSparkMax(20, MotorType.kBrushless);
        grabberTest = new WPI_TalonSRX(14);
        motorTest.restoreFactoryDefaults();
        grabberTest.configFactoryDefault();
        // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
        m_drivingEncoder = motorTest.getEncoder();
        motorTest.setIdleMode(IdleMode.kBrake);
        ;
        // m_drivingPIDController = m_drivingSparkMax.getPIDController();
        // m_turningPIDController = motorTest.getPIDController();

        // m_turningPIDController.setFeedbackDevice(m_drivingEncoder);

        // Apply position and velocity conversion factors for the driving encoder. The
        // native units for position and velocity are rotations and RPM, respectively,
        // but we want meters and meters per second to use with WPILib's swerve APIs.
        // m_drivingEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
        // m_drivingEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);

        // Apply position and velocity conversion factors for the turning encoder. We
        // want these in radians and radians per second to use with WPILib's swerve
        // APIs.
        // m_turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
        // m_turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

        // Invert the turning encoder, since the output shaft rotates in the opposite
        // direction of
        // the steering motor in the MAXSwerve Module.

        // Enable PID wrap around for the turning motor. This will allow the PID
        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
        // to 10 degrees will go through 0 rather than the other direction which is a
        // longer route.

        // // Set the PID gains for the driving motor. Note these are example gains, and
        // you
        // // may need to tune them for your own robot!
        // m_drivingPIDController.setP(ModuleConstants.kDrivingP);
        // m_drivingPIDController.setI(ModuleConstants.kDrivingI);
        // m_drivingPIDController.setD(ModuleConstants.kDrivingD);
        // m_drivingPIDController.setFF(ModuleConstants.kDrivingFF);
        // m_drivingPIDController.setOutputRange(ModuleConstants.kDrivingMinOutput,
        // ModuleConstants.kDrivingMaxOutput);

        // // Set the PID gains for the turning motor. Note these are example gains, and
        // you
        // // may need to tune them for your own robot!
        // m_turningPIDController.setP(ModuleConstants.kTurningP);
        // m_turningPIDController.setI(ModuleConstants.kTurningI);
        // m_turningPIDController.setD(ModuleConstants.kTurningD);
        // m_turningPIDController.setFF(ModuleConstants.kTurningFF);
        // m_turningPIDController.setOutputRange(ModuleConstants.kTurningMinOutput,
        // ModuleConstants.kTurningMaxOutput);

        // m_drivingSparkMax.setIdleMode(ModuleConstants.kDrivingMotorIdleMode);
        // m_turningSparkMax.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
        // m_drivingSparkMax.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
        // m_turningSparkMax.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

        // // Save the SPARK MAX configurations. If a SPARK MAX browns out during
        // // operation, it will maintain the above configurations.
        // m_drivingSparkMax.burnFlash();
        // m_turningSparkMax.burnFlash();

        // m_chassisAngularOffset = chassisAngularOffset;
        // m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
        // m_drivingEn
    }

    public void periodic() {
        double speed = robot.cont.getLeftX();
        if (speed != lastSpeed) {
            System.out.println(speed);
            motorTest.set(speed);
            lastSpeed = speed;
        }

        //double pov = Robot.cont.getPOV();
        //grabberTest.set(-pov / 360);

        if (robot.cont.getXButton()) {
            grabberTest.set(-0.7);
        } else {
            grabberTest.set(0.0);
        }

        SparkLimitSwitch forwardLimit = motorTest.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

        boolean fLimit = forwardLimit.isPressed();
        SparkLimitSwitch reverseLimit = motorTest.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        boolean rLimit = reverseLimit.isPressed();
        if (robot.count % 10 == 0) {
            System.out.println("For" + fLimit + "Rev" + rLimit);
        }

    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     * 
     *         /** Zeroes all the SwerveModule encoders.
     */
    public void resetEncoders() {
        m_drivingEncoder.setPosition(0);
    }
}
