package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.LedSubsystem.Leds;

/**
 * 3ft: 60deg 0.5 power
 * 4ft: 55deg 0.5 power
 * 6ft: 50deg 0.7 power
 * 7ft: 44deg 0.7 power
 * 8ft: 42deg 0.7 power
 **/

public class Robot extends TimedRobot {
  boolean twoMotors = true;
  boolean velocityControl = false;
  CANSparkBase motorR;
  CANSparkBase motorL;

  CANcoder canCoder;
  public XboxController cont = new XboxController(2);
  private RelativeEncoder encR;
  private RelativeEncoder encL;
  public long count = 0;
  //public static MotorNeo neo = new MotorNeo();
  public ShooterSubsystem shooter = new ShooterSubsystem(this);
  // Initializes an AnalogInput on port 0
  AnalogInput analog = new AnalogInput(0);
  LedSubsystem leds = new LedSubsystem();

  @Override
  public void robotInit() {
    // motorL = setupMotor(21);
    // if (twoMotors) {
    // motorR = setupMotor(23);
    // encR = motorL.getEncoder();
    // }
    // //canCoder = new CANcoder(9, "RIO");
    // encL = motorL.getEncoder();
leds.setAllianceLeds();
  }

  CANSparkBase setupMotor(int id) {
    CANSparkBase motor = new CANSparkFlex(id, MotorType.kBrushless);
    if (velocityControl) {
      motor.getPIDController().setP(0.1);
      motor.getPIDController().setD(0.2);
    }
    return motor;
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopPeriodic() {
    //System.out.println("Test " + analog.getVoltage());
      int pov = cont.getPOV();
      if(pov >= 0) {
        leds.setColors(Leds.GrabberOverCurrent, 80,0,0);
      }
  }

  public void teleopPeriodicOwen() {
    //neo.periodic();
    count++;
    int pov = cont.getPOV();
    double value = 0.0;
    double revs = 0;

    if (pov == 0) {
      value = 0.1;
      revs = 2000;
    } else if (pov == 90) {
      value = 0.25;
      revs = 3000;
    } else if (pov == 180) {
      value = 0.5;
      revs = 4000;
    } else if (pov == 270) {
      value = 0.7;
      revs = 5000;
    }
    controlMotor(motorL, "L", encL, revs, value);
    if (twoMotors) {
      controlMotor(motorR, "R", encR, revs, value);
    }
  }

  void controlMotor(CANSparkBase motor, String side, RelativeEncoder enc, double revs, double value) {
    if (velocityControl) {
      motor.getPIDController().setReference(-revs, CANSparkBase.ControlType.kVelocity);
    } else {
      motor.set(-value);
    }
    double velocity = enc.getVelocity();
    if (count % 20 == 0 && velocity > 0) {
      System.out.println("Vel " + side + ":" + velocity);
    }
  }
}