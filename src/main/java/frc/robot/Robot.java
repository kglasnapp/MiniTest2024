package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
// import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.hardware.CANcoder;
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
  CANSparkBase motor1 = new CANSparkFlex(1, MotorType.kBrushless);
  CANSparkBase motor2 = new CANSparkFlex(2, MotorType.kBrushless);
  //CANcoder canCoder = new CANcoder(9, "RIO");
  //TalonSRX intake = new TalonSRX(14);

  XboxController cont = new XboxController(2);

  boolean voltageControl = false;

  @Override
  public void robotInit() {
    motor1.getPIDController().setP(3e-4);
    motor1.getPIDController().setI(4e-7);
    motor1.getPIDController().setD(1e-3);
    motor2.getPIDController().setP(3e-4);
    motor2.getPIDController().setI(4e-7);
    motor2.getPIDController().setD(1e-3);
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

    motor1.set(value*.9);
    motor2.set(value);

    double speed = cont.getLeftY();
    //intake.set(TalonSRXControlMode.PercentOutput, speed);

    // motorL.getPIDController().setReference(value,
    // CANSparkBase.ControlType.kVelocity);
    // motorR.getPIDController().setReference(value,
    // CANSparkBase.ControlType.kVelocity);
  }
}