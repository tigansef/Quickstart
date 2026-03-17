package org.firstinspires.ftc.teamcode.writtenCode;

import static org.firstinspires.ftc.teamcode.writtenCode.auto.autoFarRed.endPose;
import static org.firstinspires.ftc.teamcode.writtenCode.controllers.IntakeController.IntakeStatus.OFF;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.writtenCode.controllers.ForbarController;
import org.firstinspires.ftc.teamcode.writtenCode.controllers.HoodController;
import org.firstinspires.ftc.teamcode.writtenCode.controllers.IntakeController;
import org.firstinspires.ftc.teamcode.writtenCode.controllers.PIDFController;
import org.firstinspires.ftc.teamcode.writtenCode.controllers.StopperController;
import org.firstinspires.ftc.teamcode.writtenCode.controllers.TransferController;
import org.firstinspires.ftc.teamcode.writtenCode.controllers.TurretController;

@Configurable
@Config
@TeleOp(name="TeleOpCode Red ", group="Linear OpMode")
public class TeleOpCodeRed extends LinearOpMode {

    public void setMotorRunningMode(DcMotor leftFront, DcMotor leftBack, DcMotor rightFront,
                                    DcMotor rightBack, DcMotor.RunMode runningMode) {
        leftFront.setMode(runningMode);
        rightFront.setMode(runningMode);
        leftBack.setMode(runningMode);
        rightBack.setMode(runningMode);
    }
    public void setMotorZeroPowerBehaviour(DcMotor leftFront, DcMotor leftBack, DcMotor rightFront,
                                           DcMotor rightBack, DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        leftFront.setZeroPowerBehavior(zeroPowerBehavior);
        rightFront.setZeroPowerBehavior(zeroPowerBehavior);
        leftBack.setZeroPowerBehavior(zeroPowerBehavior);
        rightBack.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void robotCentricDrive(DcMotor leftFront, DcMotor leftBack,
                                  DcMotor rightFront, DcMotor rightBack,
                                  double leftTrigger, double rightTrigger, double rate) {

        double y = -gamepad2.left_stick_y;
        double x = gamepad2.left_stick_x * 1.05;
        double rx = gamepad2.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double leftFrontPower = (y + x + rx) / denominator * rate;
        double leftBackPower = (y - x + rx) / denominator * rate;
        double rightFrontPower = (y - x - rx) / denominator * rate;
        double rightBackPower = (y + x - rx) / denominator * rate;

        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);
    }

    ElapsedTime GlobalTimer = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime shootTime = new ElapsedTime();
    private final ElapsedTime shootEndTime = new ElapsedTime();
    private final ElapsedTime shootTransferTime = new ElapsedTime();

    private Telemetry dashTelemetry;

    private Follower follower;

    private PIDFController controller;
    private DcMotorEx motor;
    private DcMotorEx motor1;
    private Servo ghidajShoot;

    boolean updateShooting = false;
    boolean flywheelOn = false;
    int noTurret = 1;
    boolean flywheelEnd = false;
    boolean shootTransfer = false;
    boolean turretOn = false;
    public static double delayShoot = 0.3;
    public static double shootTest = 1900;
    public static double shootEndCounter = 0.2;

    public static Pose startingPose = new Pose(90, 9, 0);

    public static double TURRET_OFFSET_X = -2.118;
    public static double TURRET_OFFSET_Y = 0.0;

    public static double turretOffset = 0.0;

    public static double robot_pose_x, robot_pose_y, robot_angle;
    public static double goal_pose_x = 144, goal_pose_y = 144;

    public static double turret_field_x, turret_field_y;

    public static double distanceToGoal;
    public static double turret_target_position = 0.5;
    public static double degreesToTurn;

    private RobotMap map;

    public double rate = 1;

    public static double hoodTargetPos = 0.34;

    public static double targetVelocity, velocity;
    public static double P = 0.009, I = 0, kV = 0.00038, kS = 0.06, kD = 0;

    // Turret physical limits: 170° left and 170° right (340° total)
    // Mapped onto the servo's 355° range centered at 0.5
    // 170° out of 355° total = 170/355 = 0.4789 servo units per side
    private static final double TURRET_MAX_DEGREES = 170.0;
    private static final double SERVO_RANGE        = TURRET_MAX_DEGREES / 355.0; // 0.4789

    private ColorSensor sensor1, sensor2, sensor3;
    int a1, a2, a3;
    private DistanceSensor dist1, dist2, dist3;
    double d1, d2, d3;
    boolean treibile;

    private final ElapsedTime treibileTimer = new ElapsedTime();
    private boolean treibileTimerRunning = false;

    @Override
    public void runOpMode() throws InterruptedException {
        dashTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(endPose);
        follower.update();

        RobotMap robot = new RobotMap(hardwareMap);

        IntakeController intakeController = new IntakeController(robot);
        TransferController transferController = new TransferController(robot);
        HoodController hoodController = new HoodController(robot);
        StopperController stopperController = new StopperController(robot);
        TurretController turretController = new TurretController(robot);
        ForbarController forbarController = new ForbarController(robot);

        motor = hardwareMap.get(DcMotorEx.class, "flywheelMotorR");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor1 = hardwareMap.get(DcMotorEx.class, "flywheelMotorL");
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);

        controller = new PIDFController(P, I, kD, 0.0);

        intakeController.update();
        transferController.update();
        hoodController.update(1);
        stopperController.update();
        turretController.update(turret_target_position);
        forbarController.update();

        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        DcMotor leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        MotorConfigurationType mct1, mct2, mct3, mct4;
        mct1 = rightBack.getMotorType().clone();
        mct1.setAchieveableMaxRPMFraction(1.0);
        rightBack.setMotorType(mct1);

        mct2 = rightFront.getMotorType().clone();
        mct2.setAchieveableMaxRPMFraction(1.0);
        rightFront.setMotorType(mct2);

        mct3 = leftFront.getMotorType().clone();
        mct3.setAchieveableMaxRPMFraction(1.0);
        leftFront.setMotorType(mct3);

        mct4 = leftBack.getMotorType().clone();
        mct4.setAchieveableMaxRPMFraction(1.0);
        leftBack.setMotorType(mct4);

        setMotorRunningMode(leftFront, leftBack, rightFront, rightBack,
                DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        setMotorZeroPowerBehaviour(leftFront, leftBack, rightFront, rightBack,
                DcMotor.ZeroPowerBehavior.BRAKE);

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        sensor1 = hardwareMap.get(ColorSensor.class, "sensor1");
        sensor2 = hardwareMap.get(ColorSensor.class, "sensor2");
        sensor3 = hardwareMap.get(ColorSensor.class, "sensor3");
        sensor1.enableLed(true);
        sensor2.enableLed(true);
        sensor3.enableLed(true);
        dist1 = hardwareMap.get(DistanceSensor.class, "sensor1");
        dist2 = hardwareMap.get(DistanceSensor.class, "sensor2");
        dist3 = hardwareMap.get(DistanceSensor.class, "sensor3");
        treibile = false;

        waitForStart();
        GlobalTimer.reset();

        while (opModeIsActive()) {

            if (isStopRequested()) return;

            follower.update();

            robotCentricDrive(leftFront, leftBack, rightFront, rightBack,
                    gamepad1.left_trigger, gamepad1.right_trigger, rate);

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            // --- Turret aiming ---
            robot_pose_x = follower.getPose().getX();
            robot_pose_y = follower.getPose().getY();
            robot_angle  = follower.getPose().getHeading();

            turret_field_x = robot_pose_x
                    + TURRET_OFFSET_X * Math.cos(robot_angle)
                    - TURRET_OFFSET_Y * Math.sin(robot_angle);
            turret_field_y = robot_pose_y
                    + TURRET_OFFSET_X * Math.sin(robot_angle)
                    + TURRET_OFFSET_Y * Math.cos(robot_angle);

            distanceToGoal = getDistanceToGoal(turret_field_x, turret_field_y, goal_pose_x, goal_pose_y);

            double bearingToGoal = Math.atan2(
                    goal_pose_y - turret_field_y,
                    goal_pose_x - turret_field_x
            );
            double angleToTurn = normalizeAngle(bearingToGoal - robot_angle);
            degreesToTurn = Math.toDegrees(angleToTurn);

            // Clamp to ±170° — beyond this hold at edge
            double clampedDegrees = Math.max(-TURRET_MAX_DEGREES, Math.min(TURRET_MAX_DEGREES, degreesToTurn));

            // Map (-170, +170) → (0.5 - 0.4789, 0.5 + 0.4789)
            // center (0°) → 0.5, +170° → ~0.9789, -170° → ~0.0211
            turret_target_position = 0.5 + (clampedDegrees / 355.0);

            // Apply fine-tune offset
            turret_target_position += turretOffset;

            // Safety clamp — prevents servo damage if offset pushes out of range
            turret_target_position = Math.max(0.0, Math.min(1.0, turret_target_position));
            // --- End turret aiming ---

            if (currentGamepad2.dpad_left && previousGamepad2.dpad_left) {
                turretOffset -= 0.005;
            }
            if (currentGamepad2.dpad_right && previousGamepad2.dpad_right) {
                turretOffset += 0.005;
            }

            if (currentGamepad2.right_stick_button != previousGamepad2.right_stick_button) {
                switch (noTurret) {
                    case 1:
                        noTurret = -1;
                        break;
                    case -1:
                        noTurret = 1;
                        break;
                }
            }
            if (noTurret == -1) {
                turretOn = false;
            }

            controller.setPIDF(P, I, kD, kV * targetVelocity + kS);
            velocity = motor1.getVelocity();
            motor.setPower(controller.calculate(targetVelocity - velocity));
            motor1.setPower(controller.calculate(targetVelocity - velocity));

            if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
                switch (intakeController.currentStatus) {
                    case OFF:
                        intakeController.currentStatus = IntakeController.IntakeStatus.COLLECT;
                        transferController.currentStatus = TransferController.TransferStatus.COLLECT;
                        forbarController.currentStatus = ForbarController.ForbarStatus.COLLECT;
                        break;
                    case COLLECT:
                        intakeController.currentStatus = OFF;
                        transferController.currentStatus = TransferController.TransferStatus.OFF;
                        forbarController.currentStatus = ForbarController.ForbarStatus.UP;
                        break;
                    case TRANSFER:
                        intakeController.currentStatus = OFF;
                        transferController.currentStatus = TransferController.TransferStatus.OFF;
                        break;
                }
            }

            if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
                switch (intakeController.currentStatus) {
                    case OFF:
                        intakeController.currentStatus = IntakeController.IntakeStatus.REVERSE;
                        transferController.currentStatus = TransferController.TransferStatus.REVERSE;
                        forbarController.currentStatus = ForbarController.ForbarStatus.UP;
                        break;
                    case REVERSE:
                        intakeController.currentStatus = OFF;
                        transferController.currentStatus = TransferController.TransferStatus.OFF;
                        forbarController.currentStatus = ForbarController.ForbarStatus.UP;
                        break;
                }
            }

            if (currentGamepad2.a != previousGamepad2.a) {
                turretOn = true;
                updateShooting = true;
                targetVelocity = shootTest;
                intakeController.currentStatus = IntakeController.IntakeStatus.COLLECT;
                transferController.currentStatus = TransferController.TransferStatus.COLLECT;
                hoodController.currentStatus = HoodController.HoodStatus.RUNTO;
            }

            if (currentGamepad2.b != previousGamepad2.b && shootEndTime.seconds() > shootEndCounter) {
                intakeController.currentStatus = IntakeController.IntakeStatus.COLLECT;
                transferController.currentStatus = TransferController.TransferStatus.COLLECT;
                shootTime.reset();
                flywheelOn = true;
            }

            if (shootTime.seconds() > delayShoot && flywheelOn) {
                stopperController.currentStatus = StopperController.StopperStatus.SHOOT;
                shootTransferTime.reset();
                shootTransfer = true;
                flywheelOn = false;
            }

            if (currentGamepad2.b != previousGamepad2.b && shootTransfer && shootTransferTime.seconds() > 0.2) {
                stopperController.currentStatus = StopperController.StopperStatus.NOSHOOT;
                intakeController.currentStatus = OFF;
                transferController.currentStatus = TransferController.TransferStatus.OFF;
                flywheelOn = false;
                flywheelEnd = true;
                updateShooting = false;
                shootEndTime.reset();
                shootTransfer = false;
            }

            if (turretOn)
                turretController.currentStatus = TurretController.TurretStatus.RUNTO;
            else
                turretController.currentStatus = TurretController.TurretStatus.INIT;

            if (d1 < 9 && d2 < 3 && d3 < 4.5) treibile = true;
            else {
                treibile = false;
                treibileTimerRunning = false;
            }

            if (treibile) {
                if (!treibileTimerRunning) {
                    treibileTimer.reset();
                    treibileTimerRunning = true;
                }
                if (treibileTimer.seconds() > 0.2
                        && intakeController.currentStatus == IntakeController.IntakeStatus.COLLECT) {
                    forbarController.currentStatus = ForbarController.ForbarStatus.UP;
                }
            }

            if (stopperController.currentStatus == StopperController.StopperStatus.SHOOT) {
                gamepad2.rumble(1, 1, 100);
            }

            d1 = dist1.getDistance(DistanceUnit.CM);
            d2 = dist2.getDistance(DistanceUnit.CM);
            d3 = dist3.getDistance(DistanceUnit.CM);
            a1 = sensor1.alpha();
            a2 = sensor2.alpha();
            a3 = sensor3.alpha();

            intakeController.update();
            transferController.update();
            hoodController.update(hoodTargetPos);
            stopperController.update();
            turretController.update(turret_target_position);
            forbarController.update();

            dashTelemetry.addData("distanceToGoal",    distanceToGoal);
            dashTelemetry.addData("turret_field_x",    turret_field_x);
            dashTelemetry.addData("turret_field_y",    turret_field_y);
            dashTelemetry.addData("degreesToTurn",     degreesToTurn);
            dashTelemetry.addData("clampedDegrees",    clampedDegrees);
            dashTelemetry.addData("turret_target_pos", turret_target_position);
            dashTelemetry.addData("turretOffset",      turretOffset);
            dashTelemetry.addData("robot_angle_deg",   Math.toDegrees(robot_angle));
            dashTelemetry.addData("targetVel",         targetVelocity);
            dashTelemetry.addData("curVel",            velocity);
            dashTelemetry.addData("position",          follower.getPose());
            telemetry.addData("Sensor1 Alpha Dist", a1 + " " + d1);
            telemetry.addData("Sensor2 Alpha Dist", a2 + " " + d2);
            telemetry.addData("Sensor3 Alpha Dist", a3 + " " + d3);
            telemetry.addData("trei bile", treibile);
            dashTelemetry.update();
            telemetry.update();
        }
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI)  angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    public double getDistanceToGoal(double robotX, double robotY, double goalX, double goalY) {
        double dx = goalX - robotX;
        double dy = goalY - robotY;
        return Math.sqrt(dx * dx + dy * dy);
    }
}