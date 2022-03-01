/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;


import ca.team3161.lib.utils.controls.LogitechDualAction.LogitechControl;
import ca.team3161.lib.utils.Utils;
import ca.team3161.lib.utils.controls.Gamepad;
// import ca.team3161.lib.utils.SmartDashboardTuner;
import ca.team3161.lib.utils.controls.LogitechDualAction;
import ca.team3161.lib.utils.controls.SquaredJoystickMode;
import ca.team3161.lib.utils.controls.LogitechDualAction.LogitechAxis;
import ca.team3161.lib.utils.controls.LogitechDualAction.LogitechButton;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

// import ca.team3161.lib.robot.BlinkinLEDController;
// import ca.team3161.lib.robot.BlinkinLEDController.Pattern;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import static java.lang.Math.tan;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

// import java.beans.Encoder;

// import edu.wpi.first.wpilibj.AnalogGyro;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
// import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj.motorcontrol.Spark;



public class Robot extends TimedRobot {
  /**
   * Change the I2C port below to match the connection of your color sensor
   */
  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  /**
   * A Rev Color Sensor V3 object is constructed with an I2C port as a 
   * parameter. The device will be automatically initialized with default 
   * parameters.
   */
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  private final LogitechDualAction operator = new LogitechDualAction(0);
  public static final LogitechAxis Y_AXIS = LogitechAxis.Y;
  // public static final LogitechAxis X_AXIS = LogitechAxis.X;

  private double kp = 0.000550;
  private double ki = 0.000055;
  // private final double kd = -0.0002;  Value that was being tested as of February 3, 2022.
  private double kd = 0.000020;


  // fender and launchpad shots
  public boolean shootFender = false;


  public boolean shootLaunchpad = false;

  private final PIDController shooterPid = new PIDController(kp, ki, kd);
  // private SmartDashboardTuner shooterPidTuner;

  // public static final LogitechControl RIGHT_STICK = LogitechControl.RIGHT_STICK;
  public static final LogitechControl LEFT_STICK = LogitechControl.LEFT_STICK;
  public static final LogitechControl RIGHT_STICK = LogitechControl.RIGHT_STICK;
  // public static BlinkinLEDController led = new BlinkinLEDController(0);

  // public final WPI_TalonFX shooter = new WPI_TalonFX(1);
  volatile double setPoint;
  volatile double setPointHood;

  // Ultrasonics
  // Ultrasonic ultrasonic = new Ultrasonic(0, 0); // define ultrasonic ports once plugged in

  volatile double currentOutput;
  // AnalogGyro gyro = new AnalogGyro(0);
  // // 300 degree full rotation for some reason?

  private Spark ledControl;

  // public void getSetPoint(double LEFT_STICK, String Y_AXIS) {

  // }

  public TalonSRX TurretMoter = new TalonSRX(3);
  public TalonFX shooterMotor = new TalonFX(1);
  public TalonSRX hoodMotor = new TalonSRX(4);
  
  // PID for the turret motor.
  // public PIDController turretPID = new PIDController();

  public boolean checkCenter = false;
  public double speed = 1;
  public double margin = 5;
  public boolean hood = false;
  public boolean shoot;
  public boolean launch = false;
  public double ledValue;
  volatile double setPointShooter = 0;


  @Override
  public void robotInit() {
    // Ultrasonic.setAutomaticMode(true);
    // gyro.reset();

    // this.shooterPidTuner = new SmartDashboardTuner("Smart Kp", 0.00001, v -> shooterPid.setP(v));
    // this.shooterPidTuner.start();

    this.operator.setMode(LEFT_STICK, Y_AXIS, new SquaredJoystickMode());
    // shooter.setNeutralMode(NeutralMode.Coast);
    
    setPoint = 0;
    SmartDashboard.putNumber("KP", kp);
    SmartDashboard.putNumber("KI", ki);
    SmartDashboard.putNumber("KD", kd);
    SmartDashboard.putNumber("Setpoint", setPoint);
    SmartDashboard.putNumber("shoot", setPointShooter);
    SmartDashboard.putNumber("Septoint for the hood", setPointHood);
    // hoodMotor.setSelectedSensorPosition(0);
    // this.led.setLEDPattern(Pattern.CONFETTI);
    this.ledControl = new Spark(0);
    System.out.println("WORKING******************************");

    SmartDashboard.putNumber("LED PWM Value", this.ledControl.get());
  }

  @Override
  public void teleopInit() {
    // this.led.setLEDPattern(Pattern.LARSON_RED);
    this.operator.start();

    // for current
    this.operator.bind(LogitechButton.A, z-> {
      if (z) {
          shootLaunchpad = false;
        } else {
          shootLaunchpad = true;
        }
    });

    this.operator.bind(LogitechButton.Y, b ->{
      if(b){
        if (shootFender){
          shootFender = false;
        } else {
          shootFender = true;
        }
      }
    });

    // previous
    this.operator.bind(LogitechButton.B, b ->{
      if(b){
        if (shoot){
          shoot = false;
        } else {
          shoot = true;
        }
      }
    });



    this.operator.bind(LogitechButton.X, q ->{
      if(q){
        if(hood){
          hood = false;
        }else{
          hood = true;
        }
      }
    });
    /*
    this.operator.bind(LogitechButton.Y, p ->{
      if(p){
        TurretMoter.setSelectedSensorPosition(0);
      }
    }
    );
    */
    setPoint = 0;
    setPointShooter = 0;
    setPointHood = 0;
  }

  @Override
  public void disabledInit() {
    this.operator.cancel();
  }

  @Override
  public void robotPeriodic() {
    /**
     * The method GetColor() returns a normalized color value from the sensor and can be
     * useful if outputting the color to an RGB LED or similar. To
     * read the raw color, use GetRawColor().
     * 
     * The color sensor works best when within a few inches from an object in
     * well lit conditions (the built in LED is a big help here!). The farther
     * an object is the more light from the surroundings will bleed into the 
     * measurements and make it difficult to accurately determine its color.
    //  */

    //double turretEncoderReadingPosition = this.TurretMoter.getSelectedSensorPosition();
    //double turretEncoderReadingVelocity = this.TurretMoter.getSelectedSensorVelocity();
    this.ledValue = SmartDashboard.getNumber("LED PWM Value", 0);
    this.ledControl.set(this.ledValue);
    Color detectedColor = m_colorSensor.getColor();
    double IR = m_colorSensor.getIR();
    int proximity = m_colorSensor.getProximity();
    
    double turretEncoderReadingPosition = TurretMoter.getSelectedSensorPosition();
    double turretEncoderReadingVelocity = TurretMoter.getSelectedSensorVelocity();
    double turretVel = this.shooterMotor.getSelectedSensorVelocity();
    double turretHoodPosition = this.hoodMotor.getSelectedSensorPosition();

    if(hood){
      TurretMoter.setSelectedSensorPosition(0);
      hoodMotor.setSelectedSensorPosition(0);
    }

    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the 
     * sensor.
     */
    
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tv = table.getEntry("tv");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    //read values periodically
    double v = tv.getDouble(0.0);
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);


    if(setPoint < -400000){
      System.out.println("CLAMPING THE SETPOINT");
      setPoint = -400000;
    }else if(setPoint > 400000){
      System.out.println("CLAMPING THE SETPOINT");
      setPoint = 400000;
    }

    if(setPointHood < 0){
      System.out.println("Claming hood Setpoint");
      setPointHood = 0;
    }else if(setPointHood > 550000){
      System.out.println("Clamping the hood setpoint");
      setPointHood = 550000;
    }
    // Turret turning to setpoints of roughly 180 degrees (90 each side)
    // PID seems to be needed

    /*
    if (turretEncoderReadingPosition >= -500000 && turretEncoderReadingPosition <= 500000) {
    TurretMoter.set(ControlMode.PercentOutput, this.operator.getValue(LEFT_STICK, Y_AXIS));
    }else if (turretEncoderReadingPosition < -500000){
      if(this.operator.getValue(LEFT_STICK, Y_AXIS) > 0.1){
        TurretMoter.set(ControlMode.PercentOutput, this.operator.getValue(LEFT_STICK, Y_AXIS));
      }else{
        TurretMoter.set(ControlMode.PercentOutput, 0);
      }
    }else if(turretEncoderReadingPosition > 500000){
      if(this.operator.getValue(LEFT_STICK, Y_AXIS) < -0.1){
        TurretMoter.set(ControlMode.PercentOutput, this.operator.getValue(LEFT_STICK, Y_AXIS));
      }else{
        TurretMoter.set(ControlMode.PercentOutput, 0);
      }
    }
    */


    // System.out.println(setPoint);

    // turning the turret to a rough setpoint without PID or limelight
    // tested and it works
    
    // if(turretEncoderReadingPosition >= setPoint - 50000 && turretEncoderReadingPosition <= setPoint + 30000){
    //   System.out.println("ex1");
    //   TurretMoter.set(ControlMode.PercentOutput, 0);
    // }else if(turretEncoderReadingPosition <= setPoint - 50000){
    //   System.out.println("ex2");
    //   TurretMoter.set(ControlMode.PercentOutput, 0.3);
    // }else if (turretEncoderReadingPosition >= setPoint + 50000){
    //   System.out.println("ex1");
    //   TurretMoter.set(ControlMode.PercentOutput, -0.3);
    // }

    if(shoot){
      System.out.println(setPointShooter);

      // pid for shooter
      // if(setPointShooter != 0){
      //   currentOutput = shooterPid.calculate(turretEncoderReadingVelocity, setPointShooter);
      //   currentOutput = Utils.normalizePwm(currentOutput);
      //   System.out.println(currentOutput);
      //   this.shooterMotor.set(ControlMode.PercentOutput, currentOutput);
      // }
  
      // shooter without pid
      shooterMotor.set(ControlMode.PercentOutput, setPointShooter);
      SmartDashboard.putNumber("Current Motor Output", turretVel);
      
      

      if(turretEncoderReadingPosition >= setPoint - 50000 && turretEncoderReadingPosition <= setPoint + 30000){
        System.out.println("ex1");
        TurretMoter.set(ControlMode.PercentOutput, 0);
      }else if(turretEncoderReadingPosition <= setPoint - 50000){
        System.out.println("ex2");
        TurretMoter.set(ControlMode.PercentOutput, 0.3);
      }else if (turretEncoderReadingPosition >= setPoint + 50000){
        System.out.println("ex3");
        TurretMoter.set(ControlMode.PercentOutput, -0.3);
      }
      
      if(turretHoodPosition >= setPointHood - 20000 && turretHoodPosition <= setPointHood + 20000){
        hoodMotor.set(ControlMode.PercentOutput, 0);
      }else if(turretHoodPosition <= setPointHood - 20000){
        System.out.println("ex4");
        hoodMotor.set(ControlMode.PercentOutput, 0.3);
      }else if (turretHoodPosition >= setPointHood + 20000){
        System.out.println("ex5");
        hoodMotor.set(ControlMode.PercentOutput, -0.3);
      }

      // hoodMotor.set(ControlMode.PercentOutput, .2);
      
      
    }


    // THE BELOW SHOULD WORK FOR TWO BUTTON SHOT

    /*
    // future fender and far launchpad shot
    if(shootFender){
      System.out.println("FENDER SHOT: ");
      setPointHood = 100000;
      setPointShooter = 0.33;
      setPoint = 0;
      System.out.println("SETPOINT HOOD: " + setPointHood);
      System.out.println("SETPOINT SHOOTER " + setPointShooter);
      System.out.println("SETPOINT FOR THE TURRET TURNING " + setPoint);

      shooterMotor.set(ControlMode.PercentOutput, setPointShooter);

      if(turretEncoderReadingPosition >= setPoint - 30000 && turretEncoderReadingPosition <= setPoint + 30000){
        TurretMoter.set(ControlMode.PercentOutput, 0);
      }else if(turretEncoderReadingPosition <= setPoint - 30000){
        TurretMoter.set(ControlMode.PercentOutput, 0.3);
      }else if (turretEncoderReadingPosition >= setPoint + 30000){
        TurretMoter.set(ControlMode.PercentOutput, -0.3);
      }
      
      if(turretHoodPosition >= setPointHood - 20000 && turretHoodPosition <= setPointHood + 20000){
        hoodMotor.set(ControlMode.PercentOutput, 0);
      }else if(turretHoodPosition <= setPointHood - 20000){
        hoodMotor.set(ControlMode.PercentOutput, 0.3);
      }else if (turretHoodPosition >= setPointHood + 20000){
        hoodMotor.set(ControlMode.PercentOutput, -0.3);
      }

    } else if(shootLaunchpad){
      System.out.println("LAUNCHPAD SHOT: ");
      setPointHood = 300000;
      setPointShooter = 0.65;
      setPoint = 202200;
      System.out.println("SETPOINT HOOD: " + setPointHood);
      System.out.println("SETPOINT SHOOTER " + setPointShooter);
      System.out.println("SETPOINT FOR THE TURRET TURNING " + setPoint);

      shooterMotor.set(ControlMode.PercentOutput, setPointShooter);

      if(turretEncoderReadingPosition >= setPoint - 30000 && turretEncoderReadingPosition <= setPoint + 30000){
        TurretMoter.set(ControlMode.PercentOutput, 0);
      }else if(turretEncoderReadingPosition <= setPoint - 30000){
        TurretMoter.set(ControlMode.PercentOutput, 0.3);
      }else if (turretEncoderReadingPosition >= setPoint + 30000){
        TurretMoter.set(ControlMode.PercentOutput, -0.3);
      }
      
      if(turretHoodPosition >= setPointHood - 20000 && turretHoodPosition <= setPointHood + 20000){
        hoodMotor.set(ControlMode.PercentOutput, 0);
      }else if(turretHoodPosition <= setPointHood - 20000){
        hoodMotor.set(ControlMode.PercentOutput, 0.3);
      }else if (turretHoodPosition >= setPointHood + 20000){
        hoodMotor.set(ControlMode.PercentOutput, -0.3);
      }

    }else{
      System.out.println("SHOOTER IS DISABLED");
      shooterMotor.set(ControlMode.PercentOutput, 0);
    }
    */


    // turret with limelight without pid
    // TO TEST
    /*
    double txToTicks = x * 5555;
    double target = turretEncoderReadingPosition + txToTicks;
    SmartDashboard.putNumber("txtoticks", txToTicks);
    SmartDashboard.putNumber("Target value", target);
    if(turretEncoderReadingPosition >= target - 40000 && turretEncoderReadingPosition <= target + 40000){
      TurretMoter.set(ControlMode.PercentOutput, 0);
    }else if(turretEncoderReadingPosition <= target - 40000){
      TurretMoter.set(ControlMode.PercentOutput, .5);
    }else if (turretEncoderReadingPosition >= target + 40000){
      TurretMoter.set(ControlMode.PercentOutput, -.5);
    }

    */




    // finding distance
    double a1 = 0.0;
    double a2 = y;
    double h2 = 97.5; // HEIGHT OF TARGET
    double h1 = 38; // HEIGHT OF LIMELIGHT

    double heightDif = h2-h1;
    double totalAngle = a2;
    double rs = Math.tan(Math.toRadians(totalAngle));

    // double currentDistance = 140;
    // double value1 = Math.toDegrees(Math.atan((h2-h1)/currentDistance));


    double totalDistance = heightDif / rs;

    // double ultrasonicDistance = ultrasonic.getRangeInches();

    // finding the gyro angles
    // double angle = gyro.getAngle();

    //post to smart dashboard periodically
    
    SmartDashboard.putNumber("Encoder Reading Position", turretEncoderReadingPosition);
    SmartDashboard.putNumber("Encoder Reading Velocity", turretEncoderReadingVelocity);
    // SmartDashboard.putNumber("Total distance", totalDistance);
    // SmartDashboard.putNumber("DA BABY", value1);
    // SmartDashboard.putNumber("Mounting Angle",value1 - y);
    // SmartDashboard.putNumber("Velocity", sensorVel);
    setPoint = SmartDashboard.getNumber("Setpoint in degrees", setPoint)*5555;
    // System.out.println(SmartDashboard.getNumber("Setpoint in degrees", setPoint));
    SmartDashboard.putNumber("Setpoint in degrees", setPoint/5555);

    setPointShooter = SmartDashboard.getNumber("shoot", setPointShooter);
    // System.out.println(SmartDashboard.getNumber("Septoint for the shooter", setPointShooter));
    SmartDashboard.putNumber("shoot", setPointShooter);
    // System.out.println(setPointShooter);

    setPointHood = SmartDashboard.getNumber("Septoint for the hood", setPointHood);
    // System.out.println(SmartDashboard.getNumber("Septoint for the hood", setPointHood));
    SmartDashboard.putNumber("Setpoint for the hood", setPointHood);


    SmartDashboard.putNumber("Hood encoder value", turretHoodPosition);
    
    /*
    SmartDashboard.putNumber("Current Output", currentOutput);
    SmartDashboard.putNumber("Distance", totalDistance);
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    // kp = SmartDashboard.getNumber("KP", kp);
    // ki = SmartDashboard.getNumber("KI", ki);
    // kd = SmartDashboard.getNumber("KD", kd);

    // shooterPid.setPID(kp, ki, kd);

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("IR", IR);    
    SmartDashboard.putNumber("Proximity", proximity);
    // SmartDashboard.putNumber("Encoder", sensorVel);

    */

    // SmartDashboard.putNumber("Distance Ultrasonic", ultrasonicDistance);

    // SmartDashboard.putNumber("Gyro Value", angle);
  }
}

// TODO Find height of limelight, target of limelight, plug in ultrasonic and set ports, check gryo/navx port

