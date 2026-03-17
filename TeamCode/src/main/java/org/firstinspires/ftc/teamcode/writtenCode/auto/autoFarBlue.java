package org.firstinspires.ftc.teamcode.writtenCode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.writtenCode.RobotMap;
import org.firstinspires.ftc.teamcode.writtenCode.controllers.BrakesController;
import org.firstinspires.ftc.teamcode.writtenCode.controllers.ForbarController;
import org.firstinspires.ftc.teamcode.writtenCode.controllers.HoodController;
import org.firstinspires.ftc.teamcode.writtenCode.controllers.IntakeController;
import org.firstinspires.ftc.teamcode.writtenCode.controllers.PIDFController;
import org.firstinspires.ftc.teamcode.writtenCode.controllers.StopperController;
import org.firstinspires.ftc.teamcode.writtenCode.controllers.TransferController;
import org.firstinspires.ftc.teamcode.writtenCode.controllers.TurretController;

@Autonomous(name = "Auto Far Blue", group = "Autonomous")
public class autoFarBlue extends OpMode {

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    public static Pose endPose;
    ElapsedTime timerShoot = new ElapsedTime();
    ElapsedTime timerDelay = new ElapsedTime();

    boolean turretOn = false;

    public static double targetVelocity, velocity;
    public static double P = 0.009, I = 0, kV = 0.00038, kS = 0.06, kD = 0;

    private PIDFController controller;
    private DcMotorEx motor;
    private DcMotorEx motor1;

    public static double turret_target_position = 0.5;

    public static double TURRET_OFFSET_X = -2.118;
    public static double TURRET_OFFSET_Y = 0.0;
    public static double turretOffset = 0.0;

    public static double robot_pose_x, robot_pose_y, robot_angle;
    public static double goal_pose_x = 0, goal_pose_y = 144;

    public static double turret_field_x, turret_field_y;
    public static double distanceToGoal;
    public static double degreesToTurn;

    public static double hoodTargetPos = 0.34;

    // Turret physical limits: 170 deg left and 170 deg right (340 deg total)
    // Mapped onto the servo's 355 deg range centered at 0.5
    // 170 deg out of 355 deg total = 170/355 = 0.4789 servo units per side
    private static final double TURRET_MAX_DEGREES = 170.0;
    private static final double SERVO_RANGE        = TURRET_MAX_DEGREES / 355.0; // 0.4789

    private int pathState;

    /* ---------------- START POSE ---------------- */
    private final Pose startPose = new Pose(54, 9.000, Math.toRadians(180));

    /* ---------------- PATH CONTAINER ---------------- */
    private Paths paths;

    /* ---------------- STATE MACHINE ---------------- */
    public void autonomousPathUpdate() {
        switch (pathState) {

            // Spin up flywheel, shoot pre-load, then collect on Path1
            case 0:
                targetVelocity = 1900;
                hoodController.currentStatus = HoodController.HoodStatus.RUNTO;
                hoodTargetPos = 0.34;
                intakeController.currentStatus = IntakeController.IntakeStatus.COLLECT;
                transferController.currentStatus = TransferController.TransferStatus.COLLECT;
                if (opmodeTimer.getElapsedTimeSeconds() > 2)
                    stopperController.currentStatus = StopperController.StopperStatus.SHOOT;
                if (opmodeTimer.getElapsedTimeSeconds() > 3.5) {
                    stopperController.currentStatus = StopperController.StopperStatus.NOSHOOT;
                    follower.followPath(paths.Path1);
                    intakeController.currentStatus = IntakeController.IntakeStatus.COLLECT;
                    transferController.currentStatus = TransferController.TransferStatus.COLLECT;
                    forbarController.currentStatus = ForbarController.ForbarStatus.COLLECT;
                    setPathState(1);
                }
                break;

            // Collecting on Path1 (start -> 12.337, 35.181), then go shoot on Path2
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path2);
                    targetVelocity = 1870;
                    intakeController.currentStatus = IntakeController.IntakeStatus.OFF;
                    transferController.currentStatus = TransferController.TransferStatus.OFF;
                    forbarController.currentStatus = ForbarController.ForbarStatus.UP;
                    timerShoot.reset();
                    setPathState(2);
                }
                break;

            // Shoot at Path2 end (12.337, 35.181 -> 54.107, 12.471), then collect on Path3
            case 2:
                if (!follower.isBusy()) {
                    intakeController.currentStatus = IntakeController.IntakeStatus.COLLECT;
                    transferController.currentStatus = TransferController.TransferStatus.COLLECT;
                    if (timerShoot.seconds() > 0.15)
                        stopperController.currentStatus = StopperController.StopperStatus.SHOOT;
                    if (timerShoot.seconds() > 3.65) {
                        stopperController.currentStatus = StopperController.StopperStatus.NOSHOOT;
                        follower.followPath(paths.Path3);
                        intakeController.currentStatus = IntakeController.IntakeStatus.COLLECT;
                        transferController.currentStatus = TransferController.TransferStatus.COLLECT;
                        forbarController.currentStatus = ForbarController.ForbarStatus.COLLECT;
                        setPathState(3);
                    }
                }
                break;

            // Collecting on Path3 (54.107, 12.471 -> 11.809, 9.632), then 1s delay on case 4
            case 3:
                if (!follower.isBusy()) {
                    timerDelay.reset();
                    setPathState(4);
                }
                break;

            // 1-second delay, then go shoot on Path4
            case 4:
                if (timerDelay.seconds() > 0.2) {
                    follower.followPath(paths.Path4);
                    intakeController.currentStatus = IntakeController.IntakeStatus.OFF;
                    transferController.currentStatus = TransferController.TransferStatus.OFF;
                    forbarController.currentStatus = ForbarController.ForbarStatus.UP;
                    timerShoot.reset();
                    setPathState(5);
                }
                break;

            // Shoot at Path4 end (11.809, 9.632 -> 54.320, 12.413), then collect on Path5
            case 5:
                if (!follower.isBusy()) {
                    intakeController.currentStatus = IntakeController.IntakeStatus.COLLECT;
                    transferController.currentStatus = TransferController.TransferStatus.COLLECT;
                    if (timerShoot.seconds() > 0.15)
                        stopperController.currentStatus = StopperController.StopperStatus.SHOOT;
                    if (timerShoot.seconds() > 3.05) {
                        stopperController.currentStatus = StopperController.StopperStatus.NOSHOOT;
                        follower.followPath(paths.Path5);
                        intakeController.currentStatus = IntakeController.IntakeStatus.COLLECT;
                        transferController.currentStatus = TransferController.TransferStatus.COLLECT;
                        forbarController.currentStatus = ForbarController.ForbarStatus.COLLECT;
                        setPathState(6);
                    }
                }
                break;

            // Collecting on Path5 (54.320, 12.413 -> 10.844, 16.865) done, then 1s delay on case 7
            case 6:
                if (!follower.isBusy()) {
                    timerDelay.reset();
                    setPathState(7);
                }
                break;

            // 1-second delay, then go shoot on Path6
            case 7:
                if (timerDelay.seconds() > 0.2) {
                    follower.followPath(paths.Path6);
                    intakeController.currentStatus = IntakeController.IntakeStatus.OFF;
                    transferController.currentStatus = TransferController.TransferStatus.OFF;
                    forbarController.currentStatus = ForbarController.ForbarStatus.UP;
                    timerShoot.reset();
                    setPathState(8);
                }
                break;

            // Shoot at Path6 end (10.844, 16.865 -> 54.272, 12.309), then collect on Path7
            case 8:
                if (!follower.isBusy()) {
                    intakeController.currentStatus = IntakeController.IntakeStatus.COLLECT;
                    transferController.currentStatus = TransferController.TransferStatus.COLLECT;
                    if (timerShoot.seconds() > 0.15)
                        stopperController.currentStatus = StopperController.StopperStatus.SHOOT;
                    if (timerShoot.seconds() > 3.15) {
                        stopperController.currentStatus = StopperController.StopperStatus.NOSHOOT;
                        follower.followPath(paths.Path7);
                        intakeController.currentStatus = IntakeController.IntakeStatus.COLLECT;
                        transferController.currentStatus = TransferController.TransferStatus.COLLECT;
                        forbarController.currentStatus = ForbarController.ForbarStatus.COLLECT;
                        setPathState(9);
                    }
                }
                break;

            // Collecting on Path7 (54.272, 12.309 -> 10.531, 12.336) done, then 1s delay on case 10
            case 9:
                if (!follower.isBusy()) {
                    timerDelay.reset();
                    setPathState(10);
                }
                break;

            // 1-second delay, then go shoot on Path8
            case 10:
                if (timerDelay.seconds() > 0.2) {
                    follower.followPath(paths.Path8);
                    intakeController.currentStatus = IntakeController.IntakeStatus.OFF;
                    transferController.currentStatus = TransferController.TransferStatus.OFF;
                    forbarController.currentStatus = ForbarController.ForbarStatus.UP;
                    timerShoot.reset();
                    setPathState(11);
                }
                break;

            // Shoot at Path8 end (10.531, 12.336 -> 54.462, 12.410), then park on Path9
            case 11:
                if (!follower.isBusy()) {
                    intakeController.currentStatus = IntakeController.IntakeStatus.COLLECT;
                    transferController.currentStatus = TransferController.TransferStatus.COLLECT;
                    if (timerShoot.seconds() > 0.15)
                        stopperController.currentStatus = StopperController.StopperStatus.SHOOT;
                    if (timerShoot.seconds() > 3.65) {
                        targetVelocity = 0;
                        stopperController.currentStatus = StopperController.StopperStatus.NOSHOOT;
                        follower.followPath(paths.Path9);
                        intakeController.currentStatus = IntakeController.IntakeStatus.OFF;
                        transferController.currentStatus = TransferController.TransferStatus.OFF;
                        forbarController.currentStatus = ForbarController.ForbarStatus.UP;
                        setPathState(12);
                    }
                }
                break;

            // Park (Path9: 54.462, 12.410 -> 39.260, 12.312)
            case 12:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    /* ---------------- HELPERS ---------------- */
    private void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
    }

    /* ---------------- CONTROLLERS ---------------- */
    private IntakeController intakeController;
    private TransferController transferController;
    private HoodController hoodController;
    private StopperController stopperController;
    private BrakesController brakesController;
    private TurretController turretController;
    private ForbarController forbarController;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        motor = hardwareMap.get(DcMotorEx.class, "flywheelMotorR");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor1 = hardwareMap.get(DcMotorEx.class, "flywheelMotorL");
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);

        controller = new PIDFController(P, I, kD, 0.0);

        RobotMap robot = new RobotMap(hardwareMap);

        intakeController = new IntakeController(robot);
        transferController = new TransferController(robot);
        hoodController = new HoodController(robot);
        stopperController = new StopperController(robot);
        brakesController = new BrakesController(robot);
        turretController = new TurretController(robot);
        forbarController = new ForbarController(robot);

        intakeController.update();
        transferController.update();
        hoodController.update(hoodTargetPos);
        stopperController.update();
        turretController.update(turret_target_position);
        forbarController.update();

        pathTimer = new Timer();
        opmodeTimer = new Timer();

        paths = new Paths(follower);
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        controller.setPIDF(P, I, kD, kV * targetVelocity + kS);
        velocity = motor1.getVelocity();
        motor.setPower(controller.calculate(targetVelocity - velocity));
        motor1.setPower(controller.calculate(targetVelocity - velocity));

        // --- Turret aiming (identical formula to TeleOp) ---
        robot_pose_x = follower.getPose().getX();
        robot_pose_y = follower.getPose().getY();
        robot_angle  = follower.getPose().getHeading();

        turret_field_x = robot_pose_x
                + TURRET_OFFSET_X * Math.cos(robot_angle)
                - TURRET_OFFSET_Y * Math.sin(robot_angle);
        turret_field_y = robot_pose_y
                + TURRET_OFFSET_X * Math.sin(robot_angle)
                + TURRET_OFFSET_Y * Math.cos(robot_angle);

        distanceToGoal = Math.sqrt(
                Math.pow(goal_pose_x - turret_field_x, 2) +
                        Math.pow(goal_pose_y - turret_field_y, 2)
        );

        double bearingToGoal = Math.atan2(
                goal_pose_y - turret_field_y,
                goal_pose_x - turret_field_x
        );
        double angleToTurn = normalizeAngle(bearingToGoal - robot_angle);
        degreesToTurn = Math.toDegrees(angleToTurn);

        double clampedDegrees = Math.max(-TURRET_MAX_DEGREES, Math.min(TURRET_MAX_DEGREES, degreesToTurn));
        turret_target_position = 0.5 + (clampedDegrees / 355.0);
        turret_target_position += turretOffset;
        turret_target_position = Math.max(0.0, Math.min(1.0, turret_target_position));
        // --- End turret aiming ---

        endPose = follower.getPose();

        turretOn = true;
        if (turretOn)
            turretController.currentStatus = TurretController.TurretStatus.RUNTO;
        else
            turretController.currentStatus = TurretController.TurretStatus.INIT;

        intakeController.update();
        transferController.update();
        hoodController.update(hoodTargetPos);
        stopperController.update();
        turretController.update(turret_target_position);
        forbarController.update();

        telemetry.addData("endpose", endPose);
        telemetry.addData("timerShoot", timerShoot.seconds());
        telemetry.addData("timerDelay", timerDelay.seconds());
        telemetry.addData("Path State", pathState);
        telemetry.addData("distanceToGoal", distanceToGoal);
        telemetry.addData("degreesToTurn", degreesToTurn);
        telemetry.addData("clampedDegrees", clampedDegrees);
        telemetry.addData("turret_target_pos", turret_target_position);
        telemetry.addData("turretOffset", turretOffset);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    public static Pose getEndPose() {
        return endPose;
    }

    @Override
    public void stop() {
        endPose = follower.getPose();
        follower.breakFollowing();
    }

    /* ---------------- PATH DEFINITIONS ---------------- */


    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(54.000, 9.000),
                                    new Pose(58.358, 41.803),
                                    new Pose(12.337, 35.181)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(12.337, 35.181),

                                    new Pose(54.107, 12.471)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(54.107, 12.471),

                                    new Pose(11.809, 9.632)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(11.809, 9.632),

                                    new Pose(54.320, 12.413)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(54.320, 12.413),

                                    new Pose(10.844, 16.865)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(10.844, 16.865),

                                    new Pose(54.272, 12.309)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(54.272, 12.309),

                                    new Pose(10.735, 21.758)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(10.735, 21.758),

                                    new Pose(54.462, 12.410)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(54.462, 12.410),

                                    new Pose(39.260, 12.312)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();
        }
    }

}