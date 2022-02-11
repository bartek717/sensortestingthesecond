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


import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import static java.lang.Math.tan;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

// import java.beans.Encoder;

// import edu.wpi.first.wpilibj.AnalogGyro;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
// import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;


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

  private final PIDController shooterPid = new PIDController(kp, ki, kd);
  // private SmartDashboardTuner shooterPidTuner;

  // public static final LogitechControl RIGHT_STICK = LogitechControl.RIGHT_STICK;
  public static final LogitechControl LEFT_STICK = LogitechControl.LEFT_STICK;
  public static final LogitechControl RIGHT_STICK = LogitechControl.RIGHT_STICK;

  public final WPI_TalonFX shooter = new WPI_TalonFX(1);
  volatile double setPoint;

  // Ultrasonics
  // Ultrasonic ultrasonic = new Ultrasonic(0, 0); // define ultrasonic ports once plugged in

  volatile double currentOutput;
  // AnalogGyro gyro = new AnalogGyro(0);
  // // 300 degree full rotation for some reason?

  // public void getSetPoint(double LEFT_STICK, String Y_AXIS) {

  // }

  public VictorSPX hoodMotor = new VictorSPX(6);


  public double v = 0;
  public double x = 0;
  public double y = 0;
  public double area = 0;

  @Override
  public void robotInit() {
    // Ultrasonic.setAutomaticMode(true);
    // gyro.reset();

    // this.shooterPidTuner = new SmartDashboardTuner("Smart Kp", 0.00001, v -> shooterPid.setP(v));
    // this.shooterPidTuner.start();

    this.operator.setMode(LEFT_STICK, Y_AXIS, new SquaredJoystickMode());
    shooter.setNeutralMode(NeutralMode.Coast);
    setPoint = 0;
    SmartDashboard.putNumber("KP", kp);
    SmartDashboard.putNumber("KI", ki);
    SmartDashboard.putNumber("KD", kd);
    SmartDashboard.putNumber("Setpoint", setPoint);
  }

  @Override
  public void teleopInit() {
    this.operator.start();

    this.operator.bind(LogitechButton.A, v -> {
      if (v) {
          setPoint = SmartDashboard.getNumber("Setpoint", 0);
        } else {
          setPoint = 0;
          shooter.set(0);
        }
    });

    // double speed = 0.2;

    // this.operator.bind(LogitechButton.B, b ->{
    //   if(b){
    //     if (v == 1){
    //       if (x > 5){
    //         hoodMotor.set(ControlMode.PercentOutput, speed);
    //       } else if (x < -5){
    //         hoodMotor.set(ControlMode.PercentOutput, -speed);
    //       } else {
    //         hoodMotor.set(ControlMode.PercentOutput, 0);
    //       }
    //     }
    //   } else {
    //     hoodMotor.set(ControlMode.PercentOutput, 0);
    //   }
    // });
    
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


    // if (this.operator.getButton(LogitechButton.A) && buttonPresses < 2) {
    //   setPoint = 0.3;
    //   buttonPresses += 1;
    // } else {
    //   setPoint = 0;
    //   buttonPresses = 0;
    // }


    double sensorVel = this.shooter.getSelectedSensorVelocity();

    // if (operator.getButton(LogitechButton.A)){
    if (setPoint != 0) {
      currentOutput = shooterPid.calculate(sensorVel, setPoint);
      currentOutput = Utils.normalizePwm(currentOutput);
      System.out.println(currentOutput);
      this.shooter.set(currentOutput);
      // this.shooter.set(-this.operator.getValue(LEFT_STICK, Y_AXIS));
    }
    // }

    // this.shooter.set(ControlMode.PercentOutput, this.operator.getValue(RIGHT_STICK, Y_AXIS));
    

    Color detectedColor = m_colorSensor.getColor();
    double IR = m_colorSensor.getIR();
    int proximity = m_colorSensor.getProximity();
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
    v = tv.getDouble(0.0);
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);

    // finding distance
    double a1 = 0.0;
    double a2 = y;
    double h2 = 97.5; // HEIGHT OF TARGET
    double h1 = 38; // HEIGHT OF LIMELIGHT

    double heightDif = h2-h1;
    double totalAngle = a1+a2;
    double rs = tan(totalAngle);

    double currentDistance = 140;
    double value1 = Math.toDegrees(Math.atan((h2-h1)/currentDistance));


    double totalDistance = heightDif / rs;

    // double ultrasonicDistance = ultrasonic.getRangeInches();

    // finding the gyro angles
    // double angle = gyro.getAngle();

    //post to smart dashboard periodically
    SmartDashboard.putNumber("DA BABY", value1);
    SmartDashboard.putNumber("Mounting Angle",value1 - y);
    SmartDashboard.putNumber("Velocity", sensorVel);
    // setPoint = -SmartDashboard.getNumber("Setpoint", setPoint);
    // SmartDashboard.putNumber("Setpoint", setPoint);
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
    SmartDashboard.putNumber("Encoder", sensorVel);



    // SmartDashboard.putNumber("Distance Ultrasonic", ultrasonicDistance);

    // SmartDashboard.putNumber("Gyro Value", angle);
  }
}

// TODO Find height of limelight, target of limelight, plug in ultrasonic and set ports, check gryo/navx port

