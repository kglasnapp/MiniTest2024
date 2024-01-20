package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

/**
 * 3ft: 60deg 0.5 power
 * 4ft: 55deg 0.5 power
 * 6ft: 50deg 0.7 power
 * 7ft: 44deg 0.7 power
 * 8ft: 42deg 0.7 power
 */
public class Robot extends TimedRobot {
  final CANSparkBase motor1 = new CANSparkFlex(1, MotorType.kBrushless);
  final CANSparkBase motor2 = new CANSparkFlex(2, MotorType.kBrushless);
  // CANcoder canCoder = new CANcoder(9, "RIO");
  final TalonSRX intake = new TalonSRX(14);
  final TalonFX climber = new TalonFX(10);
  final PositionVoltage request = new PositionVoltage(0).withSlot(0);

  final XboxController cont = new XboxController(2);

  final boolean voltageControl = false;

  final double maxAccel = 10000.0;
  final double maxDeaccel = 4000.0;
  double shooterSpeed = 0.0;

  @Override
  public void robotInit() {
    shooterPID(motor1);
    shooterPID(motor2);

    // Configure the TalonFX for basic use
    TalonFXConfiguration configs = new TalonFXConfiguration();

    // This TalonFX should be configured with a kP of 1, a kI of 0, a kD of 10, and
    // a kV of 2 on slot 0
    configs.Slot0.kP = 1;
    configs.Slot0.kI = 0;
    configs.Slot0.kD = 0;
    //configs.Slot0.kV = 2;

    // Write these configs to the TalonFX
    climber.getConfigurator().apply(configs);
    climber.setSafetyEnabled(true);
  }

  static void shooterPID(CANSparkBase motor) {
    motor.restoreFactoryDefaults();

    var pid = motor.getPIDController();
    pid.setP(5e-4);
    pid.setI(2e-6);
    pid.setD(2e-3);

    // motor.setSmartCurrentLimit(25);

    // pid.setSmartMotionMaxAccel(6000, 0);
    // pid.setSmartMotionMaxVelocity(6000, 0);
  }

  @Override
  public void teleopPeriodic() {
    int pov = cont.getPOV();
    double target = 0.0;

    if (cont.getYButton()) {
      // value = 0.5;
      target = 2000;
    } else if (cont.getBButton()) {
      // value = 0.7;
      target = 3000;
    } else if (cont.getAButton()) {
      // value = 0.85;
      target = 4000;
    } else if (cont.getXButton()) {
      // value = 1.0;
      target = 5000;
    }

    double error = target - shooterSpeed;
    double accel = (error >= 0.0 ? maxAccel : maxDeaccel) * this.getPeriod();
    double delta = Math.max(Math.min(error, accel), -accel);
    shooterSpeed += delta;

    //motor1.set(value * .9);
    //motor2.set(value);

    motor1.getPIDController().setReference(shooterSpeed * 1.1, CANSparkBase.ControlType.kVelocity);
    motor2.getPIDController().setReference(shooterSpeed, CANSparkBase.ControlType.kVelocity);

    double pos = cont.getLeftY();
    // intake.set(TalonSRXControlMode.PercentOutput, speed);
    //climber.set(pos);
    climber.setControl(request.withPosition(pos * 50));
  }
}