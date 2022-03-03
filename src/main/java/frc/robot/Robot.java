// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.Timer;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.networktables.*;
import edu.wpi.first.math.controller.PIDController;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  // 參數定義
  double pick_speed = 0.6; // 撿球速度
  double neo_shoot_speed = 12000; // 射球轉速
  double joy_limite = 0.025; // 過濾Xbox香菇頭抖動值
  boolean pick_switch = false; // 撿球開關
  boolean dribble_switch = false; // 運球開關
  boolean shoot_switch = false; // 射球開關
  boolean climb_switch = false; // 攀爬開關
  boolean check_basket_switch = false; // 找框開關
  boolean check_ball_switch = false; // 找球開關
  boolean is_basket = false; // 是否找到球框
  boolean is_ball = false; // 是否找到球
  boolean auto_status = false; // 目前是否為自動狀態

  // NetworkTable limelight 相關設定
  NetworkTable limelight_table;
  double limelight_area;
  double limelight_center_x;
  double limelight_center_D;
  double limelight_output_x;
  double limelight_output_D;
  double limelight_middle_x = 0; // 回傳之X座標之目標值
  double limelight_middle_D = 50; // 回傳之Y座標之目標值
  double limelight_range_x = 0.1;
  double limelight_range_D = 5;

  // x_pid_index
  double limelight_x_kP = 0.003;
  double limelight_x_kI = 0.0003;
  double limelight_x_kD = 0;

  // y_pid_index
  double limelight_D_kP = 0.0065;
  double limelight_D_kI = 0.0004;
  double limelight_D_kD = 0;
  PIDController limelight_pidController_x = new PIDController(limelight_x_kP, limelight_x_kI, limelight_x_kD);
  PIDController limelight_pidController_D = new PIDController(limelight_D_kP, limelight_D_kI, limelight_D_kD);

  // PIDController limelight_pidController_y = new PIDController(limelight_y_kP,
  // limelight_y_kI, limelight_y_kD);

  // NetworkTable raspberrypi 相關設定
  NetworkTable table_raspberrypi;
  double defaultValue = 0;
  double raspberrypi_area;
  double raspberrypi_center_x;
  double raspberrypi_center_y;
  double raspberrypi_output_x;
  double raspberrypi_output_y;
  double raspberrypi_center_middle_x;
  double raspberrypi_middle_x = 0; // 回傳之X座標之目標值
  double raspberrypi_middle_y = 0; // 回傳之Y座標之目標值
  double raspberrypi_range_x = 10;
  double raspberrypi_range_y = 10;

  // x_pid_index
  double raspberrypi_x_kP = 0;
  double raspberrypi_x_kI = 0;
  double raspberrypi_x_kD = 0;

  // y_pid_index
  double raspberrypi_y_kP = 0;
  double raspberrypi_y_kI = 0;
  double raspberrypi_y_kD = 0;
  PIDController raspberrypi_pidController_x = new PIDController(raspberrypi_x_kP, raspberrypi_x_kI, raspberrypi_x_kD);
  // PIDController raspberrypi_pidController_y = new
  // PIDController(raspberrypi_y_kP, raspberrypi_y_kI, raspberrypi_y_kD);

  // 周邊設備連接宣告
  private final XboxController m_driverController_1 = new XboxController(0); // 飛控搖桿
  private final XboxController m_driverController_2 = new XboxController(1); // XBOX搖桿
  private final Spark m_motor_pick = new Spark(0); // 撿球------PWM:1
  private final Spark m_motor_dribble = new Spark(1); // 運球------PWM:2
  private final CANSparkMax m_motor21 = new CANSparkMax(21, MotorType.kBrushless); // 左後------CANID:3
  private final CANSparkMax m_motor22 = new CANSparkMax(22, MotorType.kBrushless); // 左前------CANID:3
  private final CANSparkMax m_motor23 = new CANSparkMax(23, MotorType.kBrushless); // 右後------CANID:4
  private final CANSparkMax m_motor24 = new CANSparkMax(24, MotorType.kBrushless); // 右前------CANID:4
  private final CANSparkMax m_motor25 = new CANSparkMax(25, MotorType.kBrushless); // 右前------CANID:4
  private final CANSparkMax m_motor26 = new CANSparkMax(26, MotorType.kBrushless); // 右前------CANID:4
  private final CANSparkMax m_motor27 = new CANSparkMax(27, MotorType.kBrushless); // 右前------CANID:6
  private final CANSparkMax m_motor28 = new CANSparkMax(28, MotorType.kBrushless); // 右前------CANID:6
  private final DigitalOutput lidarlite_trigger = new DigitalOutput(0); // 光達 LIDAR-Lite Trigger Pin------DIO:0
  private final DigitalInput lidarlite_monitor = new DigitalInput(1); // 光達 LIDAR-Lite Monitor Pin------DIO:1
  private final DigitalInput forwardlimitswith_dribble = new DigitalInput(2); // 微動開關運球停止------DIO:2
  private final DigitalInput forwardlimitswith_climb = new DigitalInput(3); // 微動開關登山者左------DIO:3
  // private final DigitalInput forwardlimitswith_climb_right = new
  // DigitalInput(4); // 微動開關登山者右------DIO:4
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  public final RelativeEncoder m_encoder25 = m_motor25.getEncoder();
  public final RelativeEncoder m_encoder26 = m_motor26.getEncoder();
  private SparkMaxPIDController m_pidController25;
  private SparkMaxPIDController m_pidController26;
  public double kP = 6e-5;
  public double kI = 0;
  public double kD = 0;
  public double kIz = 0;
  public double kFF = 0.000015;
  public double kMaxOutput = 1;
  public double kMinOutput = -1;
  public double start_time_auto;
  public double start_time_shoot;
  public double start_time_climb;
  public double start_time_led;
  public double start_time_checkbasket;
  public double climb_clock = 1;

  // Dashboard 自動化顯示
  // private static final String kDefaultAuto = "Default";
  // private static final String kCustomAuto = "My Auto";
  // private String m_autoSelected;
  // private final SendableChooser<String> m_chooser = new SendableChooser<>();

  //
  private void led_init() {
    m_led = new AddressableLED(2); // LED控制器
    m_ledBuffer = new AddressableLEDBuffer(18); // led數量
    m_led.setLength(m_ledBuffer.getLength());
    led_set(255, 97, 0, m_led, m_ledBuffer);
  }

  // 設定led
  private void led_set(int r, int g, int b, AddressableLED m_led, AddressableLEDBuffer m_ledBuffer) {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setRGB(i, r, g, b);
    }
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  // 光達初始化
  private Counter m_LIDAR;

  private void lidar_init() {
    lidarlite_trigger.set(false);
    m_LIDAR = new Counter(lidarlite_monitor); // plug the lidar into DIO
    m_LIDAR.setMaxPeriod(1.00); // set the max period that can be measured
    m_LIDAR.setSemiPeriodMode(true); // Set the counter to period measurement
    m_LIDAR.reset();
  }

  private final double lidarlite_off = 5;

  // 光達測距離
  public double lidar_getDistinct() {
    double dist;
    if (m_LIDAR.get() < 1)
      dist = 0;
    else
      // convert to distance. sensor is high 10 us for every
      dist = (m_LIDAR.getPeriod() * 1000000.0 / 10.0) - lidarlite_off;
    return dist;
  }

  // 停止各種馬達
  private void stop_motor(int[] motor_number) {
    for (int number : motor_number) {
      switch (number) {
        case 0: // 全停
          m_motor_pick.set(0);
          m_motor_dribble.set(0);
          m_motor21.set(0);
          m_motor22.set(0);
          m_motor23.set(0);
          m_motor24.set(0);
          break;
        case 1: // 撿球
          m_motor_pick.set(0);
          break;
        case 2: // 運球
          m_motor_dribble.set(0);
          break;
        case 3: // 移動
          m_motor21.set(0);
          m_motor22.set(0);
          break;
        case 4: // 移動
          m_motor23.set(0);
          m_motor24.set(0);
          break;
        case 5: // 射球
          m_motor25.set(0);
          m_motor26.set(0);
          break;
        case 6: // 攀爬
          m_motor27.set(0);
          m_motor28.set(0);
          break;
        default:
          break;
      }
    }
  }

  // 移動
  private void carmove(double in_speed, double in_turn_ratio) {
    double left_speed = 0;
    double right_speed = 0;
    double min_speed = 0.8;

    if (in_speed == 0) {
      left_speed = -in_turn_ratio;
      right_speed = in_turn_ratio;
    } else {
      in_turn_ratio = (in_turn_ratio <= -min_speed ? -min_speed
          : (in_turn_ratio >= min_speed ? min_speed : in_turn_ratio)); // 避免轉到 0
      left_speed = in_speed - in_turn_ratio;
      left_speed = (left_speed >= 1 ? 1 : (left_speed <= -1 ? -1 : left_speed)); // 避免超過1或-1
      right_speed = in_speed + in_turn_ratio;
      right_speed = (right_speed >= 1 ? 1 : (right_speed <= -1 ? -1 : right_speed)); // 避免超過1或-1
    }

    m_motor21.set(-left_speed);
    m_motor22.set(-left_speed);
    m_motor23.set(right_speed);
    m_motor24.set(right_speed);
  }

  // 撿球
  private void pickball(boolean run, double speed) {
    if (run) {
      m_motor_pick.set(-speed);
    } else {
      int[] stopindex = { 1 };
      stop_motor(stopindex);
    }
  }

  // 運球
  private void dribbleball(boolean run) {
    if (run) {
      m_motor_dribble.set(-1);
    } else {
      int[] stopindex = { 2 };
      stop_motor(stopindex);
    }
  }

  // 射球
  private void shootball_by_neo(boolean run, double speed) {
    if (run) {
      m_pidController25.setReference(neo_shoot_speed, CANSparkMax.ControlType.kSmartVelocity);
      m_pidController26.setReference(neo_shoot_speed, CANSparkMax.ControlType.kVelocity);
      if (m_encoder25.getVelocity() >= 0) {
        m_motor_dribble.set(-1);
      } else {
        int[] stopindex = { 2 };
        stop_motor(stopindex);
      }
    } else {
      int[] stopindex = { 5 };
      stop_motor(stopindex);
    }
  }

  // 攀爬
  private void climb(boolean run, double speed) {
    if (run) {
      m_motor27.set(speed);
      m_motor28.set(speed);
    } else {
      int[] stopindex = { 6 };
      stop_motor(stopindex);
    }
  }

  // 設定y值
  private void set_D_to_car() {
    limelight_middle_D = lidar_getDistinct();
  }

  // limelight 瞄準球框
  private void check_basket(boolean run) {
    limelight_pidController_D.setSetpoint(limelight_middle_D);
    limelight_table = NetworkTableInstance.getDefault().getTable("limelight");
    limelight_area = limelight_table.getEntry("ta").getDouble(0);
    limelight_center_x = limelight_table.getEntry("tx").getDouble(0);
    limelight_center_D = limelight_table.getEntry("ty").getDouble(0);
    limelight_output_x = limelight_pidController_x.calculate(limelight_center_x);
    limelight_output_D = limelight_pidController_D.calculate(limelight_center_D);
    if (run) {
      // System.out.println(limelight_center_y);
      if (limelight_area != 0) {
        // is_basket = false;
        if (Math.abs(limelight_center_x) >= limelight_middle_x + limelight_range_x) {
          // m_motor21.set(-limelight_output_x);
          // m_motor22.set(-limelight_output_x);
          // m_motor23.set(-limelight_output_x);
          // m_motor24.set(-limelight_output_x);
          is_basket = false;
        } else {
          is_basket = true;
          // if (Math.abs(limelight_center_D) >= limelight_middle_D + limelight_range_D) {
          // m_motor21.set(limelight_output_D);
          // m_motor22.set(limelight_output_D);
          // m_motor23.set(-limelight_output_D);
          // m_motor24.set(-limelight_output_D);
          // }
        }

        // 前後左右一起調
        // if (Math.abs(limelight_center_x) >= limelight_middle_x + limelight_range_x ||
        // Math.abs(limelight_center_D) >= limelight_middle_D + limelight_range_D &&
        // limelight_area != 0) {
        // m_motor21.set((-limelight_center_x + limelight_output_D) / 2);
        // m_motor22.set((-limelight_center_x + limelight_output_D) / 2);
        // m_motor23.set((-limelight_center_x - limelight_output_D) / 2);
        // m_motor24.set((-limelight_center_x - limelight_output_D) / 2);
        // } else {
        // is_basket = true;
        // }
      }
    }
  }

  private void check_ball(boolean run) {
    table_raspberrypi = NetworkTableInstance.getDefault().getTable("Vision");
    raspberrypi_area = table_raspberrypi.getEntry("area").getDouble(0);
    raspberrypi_center_x = table_raspberrypi.getEntry("center_x").getDouble(0);
    raspberrypi_center_y = table_raspberrypi.getEntry("center_y").getDouble(0);
    raspberrypi_center_middle_x = table_raspberrypi.getEntry("middle").getDouble(0);
    raspberrypi_output_x = raspberrypi_pidController_x.calculate(raspberrypi_center_x);
    // System.out.println("raspberrypi_area: " + raspberrypi_area);
    // System.out.println("raspberrypi_center_x: " + raspberrypi_center_x);
    // System.out.println("raspberrypi_center_y: " + raspberrypi_center_y);
    // System.out.println("raspberrypi_center_middle_x: " +
    // raspberrypi_center_middle_x);
    if (run) {
      if (raspberrypi_area != 0) {
        if (Math.abs(raspberrypi_center_x) >= raspberrypi_middle_x + raspberrypi_range_x) {
          m_motor21.set(-raspberrypi_output_x);
          m_motor22.set(-raspberrypi_output_x);
          m_motor23.set(-raspberrypi_output_x);
          m_motor24.set(-raspberrypi_output_x);
        } else {
          is_ball = true;
        }
      }

    } else {

    }
  }

  // 用LED顯示狀態
  private void show_status() {
    if (!climb_switch) {
      if (lidar_getDistinct() <= 50) {
        // while (lidar_getDistinct() <= 50) {
        //   if (Timer.getFPGATimestamp() - start_time_led >= 0.1) {
        //     start_time_led = Timer.getFPGATimestamp();
        //     led_set(255, 0, 0, m_led, m_ledBuffer);
        //   }
        // }
      } else {
        if (shoot_switch) { // 藍到紫
          int r = 0;
          // while (r <= 127 && shoot_switch) {
          //   if (Timer.getFPGATimestamp() - start_time_led >= 0.1) {
          //     start_time_led = Timer.getFPGATimestamp();
          //     led_set(255, 0, 0, m_led, m_ledBuffer);
          //     r += 5;
          //   }
          // }
        } else if (dribble_switch) {
          led_set(255, 255, 0, m_led, m_ledBuffer); // 黃燈
        } else {
          led_set(0, 0, 255, m_led, m_ledBuffer);
        }
      }
    } else if (climb_switch) {
      System.out.println("Show");      
      led_set(0, 255, 0, m_led, m_ledBuffer);
    } else if (check_basket_switch) {
      // while (check_basket_switch && Timer.getFPGATimestamp() - start_time_led >= Math.abs(limelight_output_x)) {
      //   start_time_led = Timer.getFPGATimestamp();
      //   led_set(238, 130, 238, m_led, m_ledBuffer);
      // }
    }
  }

  private void show_dashboard() {
    // Dashboard初始設定
    // m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    // m_chooser.addOption("My Auto", kCustomAuto);
    // SmartDashboard.putData("Auto choices", m_chooser);
    SmartDashboard.putBoolean("Drib limit", forwardlimitswith_dribble.get());
    SmartDashboard.putBoolean("climb switch", forwardlimitswith_climb.get());
    // SmartDashboard.putBoolean("climb switch right",
    // forwardlimitswith_climb_right.get());
    SmartDashboard.putBoolean("Pick status", pick_switch);
    SmartDashboard.putBoolean("Drib status", dribble_switch);
    SmartDashboard.putBoolean("Shoot status", shoot_switch);
    SmartDashboard.putBoolean("Climb status", climb_switch);
    SmartDashboard.putBoolean("Check ball", check_ball_switch);
    SmartDashboard.putBoolean("Check basket", check_basket_switch);
    SmartDashboard.putBoolean("Auto Status", auto_status);
    SmartDashboard.putNumber("Climb Clock", climb_clock);
  }

  @Override
  public void robotInit() {
    show_dashboard();
    lidar_init(); // 初始化光達
    // 初始化LED
<<<<<<< HEAD
    led_init();

=======
    m_led_2.setLength(m_ledBuffer.getLength());
    // m_led_3.setLength(m_ledBuffer.getLength());
    led_set(255, 97, 0);//橘燈
>>>>>>> 7780eb8faa670d1a9948ec6cf56b000b08296fca
    // 初始化NEO
    m_motor25.restoreFactoryDefaults();
    m_motor26.restoreFactoryDefaults();
    m_pidController25 = m_motor25.getPIDController();
    m_pidController26 = m_motor26.getPIDController();
    m_pidController25.setP(kP);
    m_pidController25.setI(kI);
    m_pidController25.setD(kD);
    m_pidController25.setIZone(kIz);
    m_pidController25.setFF(kFF);
    m_pidController25.setOutputRange(kMinOutput, kMaxOutput);
    m_pidController26.setP(kP);
    m_pidController26.setI(kI);
    m_pidController26.setD(kD);
    m_pidController26.setIZone(kIz);
    m_pidController26.setFF(kFF);
    m_pidController26.setOutputRange(kMinOutput, kMaxOutput);
    m_motor25.setInverted(true);
    // PID控制設定
    limelight_pidController_x.setSetpoint(limelight_middle_x);
    limelight_pidController_D.setSetpoint(limelight_middle_D);
    // 初始化DashBoard
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    // m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    // System.out.println("Auto selected: " + m_autoSelected);
    start_time_auto = Timer.getFPGATimestamp();
    // 設定初始值
    pick_switch = false;
    dribble_switch = false;
    shoot_switch = true;
    climb_switch = false;
    check_ball_switch = false;
    check_basket_switch = false;
    auto_status = true;
    is_basket = false;
    is_ball = false;
    pick_speed = 0.6;
    neo_shoot_speed = 12000;
    limelight_middle_x = 0;
    limelight_middle_D = 50;
    // 設定一般鏡頭
    table_raspberrypi = NetworkTableInstance.getDefault().getTable("Vision");
    raspberrypi_center_middle_x = table_raspberrypi.getEntry("middle").getDouble(0);

    led_set(255, 97, 0, m_led, m_ledBuffer);
    // led_set(255, 97, 0, m_led_3, m_ledBuffer_3);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    show_dashboard();
    m_pidController25.setReference(neo_shoot_speed, CANSparkMax.ControlType.kVelocity);
    m_pidController26.setReference(neo_shoot_speed, CANSparkMax.ControlType.kVelocity);
    if (m_encoder25.getVelocity() >= 0 && auto_status) {
      m_motor_dribble.set(-1);
      if (Timer.getFPGATimestamp() - start_time_auto > 7 && Timer.getFPGATimestamp() - start_time_auto < 12) {
        m_motor21.set(-0.15);
        m_motor22.set(-0.15);
        m_motor23.set(0.15);
        m_motor24.set(0.15);
      } else if (Timer.getFPGATimestamp() - start_time_auto >= 12) {
        auto_status = false;
        m_motor21.set(0);
        m_motor22.set(0);
        m_motor23.set(0);
        m_motor24.set(0);
        int[] stopindex = { 2, 5 };
        stop_motor(stopindex);
      }
    } else {
      int[] stopindex = { 2 };
      stop_motor(stopindex);
    }

    // switch (m_autoSelected) {
    // case kCustomAuto:
    // // Put custom auto code here
    // break;
    // case kDefaultAuto:
    // default:
    // // Put default auto code here
    // break;
    // }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    show_dashboard();
    // 設定初始值
    pick_switch = false;
    dribble_switch = false;
    shoot_switch = false;
    climb_switch = false;
    check_ball_switch = false;
    check_basket_switch = false;
    is_basket = false;
    is_ball = false;
    pick_speed = 0.6;
    neo_shoot_speed = 12000;
    limelight_middle_x = 0;
    limelight_middle_D = 50;
    // 設定一般鏡頭
    table_raspberrypi = NetworkTableInstance.getDefault().getTable("Vision");
    raspberrypi_center_middle_x = table_raspberrypi.getEntry("middle").getDouble(0);
    start_time_auto = Timer.getFPGATimestamp();
    start_time_led = Timer.getFPGATimestamp();
    led_set(255, 97, 0, m_led, m_ledBuffer);
    // led_set(255, 97, 0, m_led_3, m_ledBuffer_3);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    show_status();
    limelight_pidController_x = new PIDController(limelight_x_kP, limelight_x_kI, limelight_x_kD);

    // limelight_pidController_y = new PIDController(limelight_y_kP, limelight_y_kI,
    // limelight_y_kD);
    raspberrypi_pidController_x = new PIDController(raspberrypi_x_kP, raspberrypi_x_kI, raspberrypi_x_kD);
    // raspberrypi_pidController_y = new PIDController(raspberrypi_y_kP,
    // raspberrypi_y_kI, raspberrypi_y_kD);
    // 移動車
    double joy_speed = -m_driverController_1.getRawAxis(0) * 0.7;
    double joy_turn = m_driverController_1.getRawAxis(1) * 0.5;

    // if (Math.abs(joy_speed) < 0.03) {
    // joy_speed = 0;
    // }
    // if (Math.abs(joy_turn) < 0.03) {
    // joy_turn = 0;
    // }
    carmove(joy_speed, joy_turn);

    // 撿球
    if (m_driverController_2.getXButtonReleased()) {
      pick_switch = !pick_switch;
    }

    if (m_driverController_2.getPOV() == 90) {
      pick_speed += 0.05;
    }

    if (m_driverController_2.getPOV() == 270) {
      pick_speed -= 0.05;
    }
    pickball(pick_switch, pick_speed);

    // 射球
    if (m_driverController_2.getYButtonReleased()) {
      shoot_switch = !shoot_switch; // 切換射球開關
      if (shoot_switch) {
        dribble_switch = false;
        start_time_shoot = Timer.getFPGATimestamp();
      }
    } else {
      if (!forwardlimitswith_dribble.get()) {
        dribble_switch = false;
      } else {
        dribble_switch = true;
      }
    }
    dribbleball(dribble_switch);

    // 射球馬達增減速
    if (m_driverController_2.getPOV() == 0) {
      neo_shoot_speed += 500;
    }
    if (m_driverController_2.getPOV() == 180) {
      if (neo_shoot_speed > 0) {
        neo_shoot_speed -= 500;
      }
    }
    shootball_by_neo(shoot_switch, neo_shoot_speed);

    // 攀爬
    if (m_driverController_2.getAButtonReleased()) {
      climb_switch = !climb_switch;
    }

    if (climb_switch) {
      if (forwardlimitswith_climb.get()) {
        if (m_driverController_2.getLeftTriggerAxis() > 0) { // 順時鐘
          climb(true, m_driverController_2.getLeftTriggerAxis() * 0.4);
          climb_clock = 1;
        } else if (m_driverController_2.getRightTriggerAxis() > 0) { // 逆時鐘
          climb(true, -m_driverController_2.getRightTriggerAxis() * 0.4);
          climb_clock = -1;
        } else {
          climb(false, 0);
        }
      } else {
        if (!forwardlimitswith_climb.get()) {
          climb(true, 0.06 * -climb_clock);
          Timer.delay(0.3);
          climb_switch = false;
        }
        climb(false, 0);
      }
    }

    // // 馬達全停
    // if (m_driverController_2.getLeftBumperReleased()) {
    // int[] stopindex = { 0 };
    // stop_motor(stopindex);
    // }

    // 設定距離
    if (m_driverController_2.getStartButton()) {
      set_D_to_car();
    }

    // limelight 找框
    if (m_driverController_2.getLeftBumperReleased()) {
      check_basket_switch = !check_basket_switch;
      if (check_basket_switch) {
        is_basket = false;
      }
    }
    check_basket(check_basket_switch);

    // camaera 找球
    if (m_driverController_2.getRightBumperReleased()) {
      check_ball_switch = !check_ball_switch;
      if (check_ball_switch) {
        is_ball = false;
      }
    }
    check_ball(check_ball_switch);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // 設定初始值
    pick_switch = false;
    dribble_switch = false;
    shoot_switch = false;
    climb_switch = false;
    check_ball_switch = false;
    check_basket_switch = false;

  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    show_dashboard();
    show_status();
    // 撿球
    if (m_driverController_2.getXButtonReleased()) {
      pick_switch = !pick_switch;
    }

    // 射球
    if (m_driverController_2.getYButtonReleased()) {
      shoot_switch = !shoot_switch; // 切換射球開關
      if (shoot_switch == true) {
        dribble_switch = true;
      }
    }

    // 運球
    if (!shoot_switch && !forwardlimitswith_dribble.get()) {
      dribble_switch = false;
    } else {
      if (m_driverController_2.getBButtonReleased()) {
        dribble_switch = !dribble_switch;
      }
    }

    // 攀爬
    if (m_driverController_2.getAButtonReleased()) {
      climb_switch = !climb_switch;
    }

    if (climb_switch) {
      if (forwardlimitswith_climb.get()) {
        if (m_driverController_2.getLeftTriggerAxis() > 0) { // 順時鐘
          climb(true, m_driverController_2.getLeftTriggerAxis() * 0.4);
          climb_clock = 1;
        } else if (m_driverController_2.getRightTriggerAxis() > 0) { // 逆時鐘
          climb(true, -m_driverController_2.getRightTriggerAxis() * 0.4);
          climb_clock = -1;
        } else {
          climb(false, 0);
        }
      } else {
        if (!forwardlimitswith_climb.get()) {
          climb(true, 0.06 * -climb_clock);
          Timer.delay(0.3);
          climb_switch = false;
        }
        climb(false, 0);
      }
    }
  }
}
