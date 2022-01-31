/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

// import ca.team3161.lib.utils.controls.LogitechDualAction.LogitechButton;
import ca.team3161.lib.utils.controls.LogitechDualAction.LogitechControl;
import ca.team3161.lib.utils.controls.LogitechDualAction;
import ca.team3161.lib.utils.controls.SquaredJoystickMode;
import ca.team3161.lib.utils.controls.LogitechDualAction.LogitechAxis;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import static java.lang.Math.tan;
import edu.wpi.first.wpilibj.AnalogGyro;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;


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

  // public static final LogitechControl RIGHT_STICK = LogitechControl.RIGHT_STICK;
  public static final LogitechControl LEFT_STICK = LogitechControl.LEFT_STICK;

  public final PWMTalonFX shooter = new PWMTalonFX(0);

  // Ultrasonics
  Ultrasonic ultrasonic = new Ultrasonic(0, 0); // define ultrasonic ports once plugged in

  AnalogGyro gyro = new AnalogGyro(0);
  // 300 degree full rotation for some reason?

  @Override
  public void robotInit() {
    Ultrasonic.setAutomaticMode(true);
    gyro.reset();

    this.operator.setMode(LEFT_STICK, Y_AXIS, new SquaredJoystickMode());

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
     */


    this.shooter.set(this.operator.getValue(LEFT_STICK, Y_AXIS));

    

    Color detectedColor = m_colorSensor.getColor();
    double IR = m_colorSensor.getIR();
    int proximity = m_colorSensor.getProximity();
    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the 
     * sensor.
     */

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    // finding distance
    double a1 = 0.0;
    double a2 = y;
    double h2 = 0; // HEIGHT OF TARGET
    double h1 = 0; // HEIGHT OF LIMELIGHT

    double heightDif = h2-h1;
    double totalAngle = a1+a2;
    double rs = tan(totalAngle);

    double totalDistance = heightDif / rs;

    double ultrasonicDistance = ultrasonic.getRangeInches();

    // finding the gyro angles
    double angle = gyro.getAngle();

    //post to smart dashboard periodically
    SmartDashboard.putNumber("Distance", totalDistance);
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("IR", IR);    
    SmartDashboard.putNumber("Proximity", proximity);

    SmartDashboard.putNumber("Distance Ultrasonic", ultrasonicDistance);

    SmartDashboard.putNumber("Gyro Value", angle);
  }
}

// TODO Find height of limelight, target of limelight, plug in ultrasonic and set ports, check gryo/navx port
