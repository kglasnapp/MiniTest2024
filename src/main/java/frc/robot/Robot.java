package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
// import com.ctre.phoenix.sensors.CANCoder;
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
  // TalonSRX intake = new TalonSRX(14);
  final TalonFX climber = new TalonFX(10);
  final PositionVoltage request = new PositionVoltage(0).withSlot(0);

  final XboxController cont = new XboxController(2);

  final boolean voltageControl = false;

  @Override
  public void robotInit() {
    motor1.getPIDController().setP(3e-4);
    motor1.getPIDController().setI(4e-7);
    motor1.getPIDController().setD(1e-3);
    motor2.getPIDController().setP(3e-4);
    motor2.getPIDController().setI(4e-7);
    motor2.getPIDController().setD(1e-3);

    // Configure the TalonFX for basic use
    Slot0Configs configs = new Slot0Configs();

    // This TalonFX should be configured with a kP of 1, a kI of 0, a kD of 10, and
    // a kV of 2 on slot 0
    configs.kP = 1;
    configs.kI = 0;
    configs.kD = 0;
    //configs.Slot0.kV = 2;

    // Write these configs to the TalonFX
    climber.getConfigurator().apply(configs);
    climber.setSafetyEnabled(true);
  }

  @Override
  public void teleopPeriodic() {
    int pov = cont.getPOV();
    double value = 0.0;

    if (cont.getYButton()) {
      value = 0.5;
      // value = 2000;
    } else if (cont.getBButton()) {
      value = 0.7;
      // value = 3000;
    } else if (cont.getAButton()) {
      value = 0.85;
      // value = 4000;
    } else if (cont.getXButton()) {
      value = 1.0;
      // value = 5000;
    }

    motor1.set(value * .9);
    motor2.set(value);

    double speed = cont.getLeftY();
    // intake.set(TalonSRXControlMode.PercentOutput, speed);
    // climber.set(speed);
    climber.setControl(request.withPosition(speed * 200));

    // motor1.getPIDController().setReference(value,
    // CANSparkBase.ControlType.kVelocity);
    // motor2.getPIDController().setReference(value,
    // CANSparkBase.ControlType.kVelocity);
  }
}