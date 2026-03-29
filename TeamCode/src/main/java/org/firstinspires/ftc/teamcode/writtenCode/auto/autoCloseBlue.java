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

@Autonomous(name = "Auto Close Blue", group = "Autonomous")
public class autoCloseBlue extends OpMode {

    private double normalizeAngle(double angle) {
        while (angle > Math.PI)  angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public static double hoodAngleClose(double dist) {
        return clamp(-0.00427769 * dist + 1.20635, 0.5, 1.0);
    }

    public static double flywheelSpeedClose(double dist) {
        return clamp(
                0.000100043 * Math.pow(dist, 4)
                        - 0.032534  * Math.pow(dist, 3)
                        + 3.91068   * Math.pow(dist, 2)
                        - 201.78771 * dist
                        + 5195.50308,
                1250, 1900
        );
    }

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    public static Pose endPose;

    ElapsedTime timerShoot = new ElapsedTime();
    ElapsedTime timerDelay = new ElapsedTime();

    public static double targetVelocity, velocity;
    public static double P = 0.009, I = 0, kV = 0.00038, kS = 0.06, kD = 0;

    private PIDFController controller;
    private DcMotorEx motor;
    private DcMotorEx motor1;

    public static double turret_target_position = 0.5;
    public static double TURRET_OFFSET_X = -2.118;
    public static double TURRET_OFFSET_Y =  0.0;
    public static double turretOffset    =  0.0;

    public static double robot_pose_x, robot_pose_y, robot_angle;
    public static double goal_pose_x = 3, goal_pose_y = 142;
    public static double turret_field_x, turret_field_y;
    public static double distanceToGoal;
    public static double degreesToTurn;
    public static double hoodTargetPos = 0.55;

    private static final double TURRET_MAX_DEGREES = 170.0;

    private int pathState;

    private final Pose startPose = new Pose(32.000, 136.000, Math.toRadians(90));

    private Paths paths;

    private IntakeController   intakeController;
    private TransferController transferController;
    private HoodController     hoodController;
    private StopperController  stopperController;
    private BrakesController   brakesController;
    private TurretController   turretController;
    private ForbarController   forbarController;

    // =========================================================================
    //  STATE MACHINE
    //
    //  Shoot sequence — always two cases:
    //    ODD  case: wait !isBusy → open stopper, reset timer, advance
    //    EVEN case: t>0.15s intake+transfer ON  |  t>1.0s stopper closes, next path
    //
    //  Full sequence:
    //    Path1  → shoot → Path2 collect → Path3 → shoot → Path4 collect → Path5
    //    → shoot → Path6 collect (1.5s gate wait) → Path7 → shoot
    //    → Path8 collect (no wait) → Path9 → shoot
    //    → Path13 collect (no wait) → Path14 → shoot
    //    → Path10 collect → Path11 → shoot → park Path12
    // =========================================================================
    public void autonomousPathUpdate() {
        switch (pathState) {

            // ── Case 0: spin up, follow Path1 (32,136 → 52,84) ──────────────
            case 0:
                hoodController.currentStatus     = HoodController.HoodStatus.RUNTO;
                intakeController.currentStatus   = IntakeController.IntakeStatus.OFF;
                transferController.currentStatus = TransferController.TransferStatus.OFF;
                forbarController.currentStatus   = ForbarController.ForbarStatus.UP;
                follower.followPath(paths.Path1);
                setPathState(1);
                break;

            // ── Case 1: arrived 52,84 (end Path1) → open stopper, start timer
            case 1:
                if (!follower.isBusy()) {
                    stopperController.currentStatus = StopperController.StopperStatus.SHOOT;
                    timerShoot.reset();
                    setPathState(2);
                }
                break;

            // ── Case 2: shoot; t>0.15s intake ON; t>1.0s close, collect Path2
            case 2:
                if (timerShoot.seconds() > 0.15) {
                    intakeController.currentStatus   = IntakeController.IntakeStatus.COLLECT;
                    transferController.currentStatus = TransferController.TransferStatus.COLLECT;
                }
                if (timerShoot.seconds() > 1.0) {
                    stopperController.currentStatus  = StopperController.StopperStatus.NOSHOOT;
                    follower.followPath(paths.Path2, 0.8, true);
                    intakeController.currentStatus   = IntakeController.IntakeStatus.COLLECT;
                    transferController.currentStatus = TransferController.TransferStatus.COLLECT;
                    forbarController.currentStatus   = ForbarController.ForbarStatus.COLLECT;
                    setPathState(3);
                }
                break;

            // ── Case 3: collecting Path2 (52,84 → 25,83.8); done → delay ────
            case 3:
                if (!follower.isBusy()) {
                    timerDelay.reset();
                    setPathState(4);
                }
                break;

            // ── Case 4: delay, follow Path3 back to 52,84 ───────────────────
            case 4:
                if (timerDelay.seconds() > 0.2) {
                    follower.followPath(paths.Path3);
                    intakeController.currentStatus   = IntakeController.IntakeStatus.OFF;
                    transferController.currentStatus = TransferController.TransferStatus.OFF;
                    forbarController.currentStatus   = ForbarController.ForbarStatus.UP;
                    setPathState(5);
                }
                break;

            // ── Case 5: arrived 52,84 (end Path3) → open stopper, start timer
            case 5:
                if (!follower.isBusy()) {
                    stopperController.currentStatus = StopperController.StopperStatus.SHOOT;
                    timerShoot.reset();
                    setPathState(6);
                }
                break;

            // ── Case 6: shoot; t>0.15s intake ON; t>1.0s close, collect Path4
            case 6:
                if (timerShoot.seconds() > 0.15) {
                    intakeController.currentStatus   = IntakeController.IntakeStatus.COLLECT;
                    transferController.currentStatus = TransferController.TransferStatus.COLLECT;
                }
                if (timerShoot.seconds() > 1.0) {
                    stopperController.currentStatus  = StopperController.StopperStatus.NOSHOOT;
                    follower.followPath(paths.Path4);
                    intakeController.currentStatus   = IntakeController.IntakeStatus.COLLECT;
                    transferController.currentStatus = TransferController.TransferStatus.COLLECT;
                    forbarController.currentStatus   = ForbarController.ForbarStatus.COLLECT;
                    setPathState(7);
                }
                break;

            // ── Case 7: collecting Path4 Bezier (52,84 → 24.882,60.038); done → delay
            case 7:
                if (!follower.isBusy()) {
                    timerDelay.reset();
                    setPathState(8);
                }
                break;

            // ── Case 8: delay, follow Path5 back to 52,84 ───────────────────
            case 8:
                if (timerDelay.seconds() > 0.2) {
                    follower.followPath(paths.Path5);
                    intakeController.currentStatus   = IntakeController.IntakeStatus.OFF;
                    transferController.currentStatus = TransferController.TransferStatus.OFF;
                    forbarController.currentStatus   = ForbarController.ForbarStatus.UP;
                    setPathState(9);
                }
                break;

            // ── Case 9: arrived 52,84 (end Path5) → open stopper, start timer
            case 9:
                if (!follower.isBusy()) {
                    stopperController.currentStatus = StopperController.StopperStatus.SHOOT;
                    timerShoot.reset();
                    setPathState(10);
                }
                break;

            // ── Case 10: shoot; t>0.15s intake ON; t>1.0s close, collect Path6 (gate)
            case 10:
                if (timerShoot.seconds() > 0.15) {
                    intakeController.currentStatus   = IntakeController.IntakeStatus.COLLECT;
                    transferController.currentStatus = TransferController.TransferStatus.COLLECT;
                }
                if (timerShoot.seconds() > 1.0) {
                    stopperController.currentStatus  = StopperController.StopperStatus.NOSHOOT;
                    follower.followPath(paths.Path6);
                    intakeController.currentStatus   = IntakeController.IntakeStatus.COLLECT;
                    transferController.currentStatus = TransferController.TransferStatus.COLLECT;
                    forbarController.currentStatus   = ForbarController.ForbarStatus.COLLECT;
                    setPathState(11);
                }
                break;

            // ── Case 11: collecting Path6 (52,84 → 15,62); done → 1.5s gate wait
            case 11:
                if (!follower.isBusy()) {
                    timerDelay.reset();
                    setPathState(12);
                }
                break;

            // ── Case 12: 1.5s gate wait at 15,62, then follow Path7 back to 52,84
            case 12:
                if (timerDelay.seconds() > 1.5) {
                    follower.followPath(paths.Path7);
                    intakeController.currentStatus   = IntakeController.IntakeStatus.OFF;
                    transferController.currentStatus = TransferController.TransferStatus.OFF;
                    forbarController.currentStatus   = ForbarController.ForbarStatus.UP;
                    setPathState(13);
                }
                break;

            // ── Case 13: arrived 52,84 (end Path7) → open stopper, start timer
            case 13:
                if (!follower.isBusy()) {
                    stopperController.currentStatus = StopperController.StopperStatus.SHOOT;
                    timerShoot.reset();
                    setPathState(14);
                }
                break;

            // ── Case 14: shoot; t>0.15s intake ON; t>1.0s close, collect Path8 (no wait)
            case 14:
                if (timerShoot.seconds() > 0.15) {
                    intakeController.currentStatus   = IntakeController.IntakeStatus.COLLECT;
                    transferController.currentStatus = TransferController.TransferStatus.COLLECT;
                }
                if (timerShoot.seconds() > 1.0) {
                    stopperController.currentStatus  = StopperController.StopperStatus.NOSHOOT;
                    follower.followPath(paths.Path8);
                    intakeController.currentStatus   = IntakeController.IntakeStatus.COLLECT;
                    transferController.currentStatus = TransferController.TransferStatus.COLLECT;
                    forbarController.currentStatus   = ForbarController.ForbarStatus.COLLECT;
                    setPathState(15);
                }
                break;

            // ── Case 15: collecting Path8 Bezier (52,84 → 24.356,35.791); done → delay
            case 15:
                if (!follower.isBusy()) {
                    timerDelay.reset();
                    setPathState(16);
                }
                break;

            // ── Case 16: delay, follow Path9 back to 52,84
            case 16:
                if (timerDelay.seconds() > 0.2) {
                    follower.followPath(paths.Path9);
                    intakeController.currentStatus   = IntakeController.IntakeStatus.OFF;
                    transferController.currentStatus = TransferController.TransferStatus.OFF;
                    forbarController.currentStatus   = ForbarController.ForbarStatus.UP;
                    setPathState(17);
                }
                break;

            // ── Case 17: arrived 52,84 (end Path9) → open stopper, start timer
            case 17:
                if (!follower.isBusy()) {
                    stopperController.currentStatus = StopperController.StopperStatus.SHOOT;
                    timerShoot.reset();
                    setPathState(18);
                }
                break;

            // ── Case 18: shoot; t>0.15s intake ON; t>1.0s close, collect Path10 (no wait)
            case 18:
                if (timerShoot.seconds() > 0.15) {
                    intakeController.currentStatus   = IntakeController.IntakeStatus.COLLECT;
                    transferController.currentStatus = TransferController.TransferStatus.COLLECT;
                }
                if (timerShoot.seconds() > 1.0) {
                    stopperController.currentStatus  = StopperController.StopperStatus.NOSHOOT;
                    follower.followPath(paths.Path10);
                    intakeController.currentStatus   = IntakeController.IntakeStatus.COLLECT;
                    transferController.currentStatus = TransferController.TransferStatus.COLLECT;
                    forbarController.currentStatus   = ForbarController.ForbarStatus.COLLECT;
                    setPathState(19);
                }
                break;

            // ── Case 19: collecting Path10 Bezier (52,84 → 8.855,41.721); done → delay
            case 19:
                if (!follower.isBusy()) {
                    timerDelay.reset();
                    setPathState(20);
                }
                break;

            // ── Case 20: delay, follow Path11 back to 52,84
            case 20:
                if (timerDelay.seconds() > 0.2) {
                    follower.followPath(paths.Path11);
                    intakeController.currentStatus   = IntakeController.IntakeStatus.OFF;
                    transferController.currentStatus = TransferController.TransferStatus.OFF;
                    forbarController.currentStatus   = ForbarController.ForbarStatus.UP;
                    setPathState(21);
                }
                break;

            // ── Case 21: arrived 52,84 (end Path11) → open stopper, start timer
            case 21:
                if (!follower.isBusy()) {
                    stopperController.currentStatus = StopperController.StopperStatus.SHOOT;
                    timerShoot.reset();
                    setPathState(22);
                }
                break;

            // ── Case 22: shoot; t>0.15s intake ON; t>1.0s close, collect Path12 (no wait)
            case 22:
                if (timerShoot.seconds() > 0.15) {
                    intakeController.currentStatus   = IntakeController.IntakeStatus.COLLECT;
                    transferController.currentStatus = TransferController.TransferStatus.COLLECT;
                }
                if (timerShoot.seconds() > 1.0) {
                    stopperController.currentStatus  = StopperController.StopperStatus.NOSHOOT;
                    follower.followPath(paths.Path12);
                    intakeController.currentStatus   = IntakeController.IntakeStatus.COLLECT;
                    transferController.currentStatus = TransferController.TransferStatus.COLLECT;
                    forbarController.currentStatus   = ForbarController.ForbarStatus.COLLECT;
                    setPathState(23);
                }
                break;

            // ── Case 23: collecting Path12 Bezier (52,84 → 9.050,41.818); done → delay
            case 23:
                if (!follower.isBusy()) {
                    timerDelay.reset();
                    setPathState(24);
                }
                break;

            // ── Case 24: delay, follow Path13 back to 52,84
            case 24:
                if (timerDelay.seconds() > 0.2) {
                    follower.followPath(paths.Path13);
                    intakeController.currentStatus   = IntakeController.IntakeStatus.OFF;
                    transferController.currentStatus = TransferController.TransferStatus.OFF;
                    forbarController.currentStatus   = ForbarController.ForbarStatus.UP;
                    setPathState(25);
                }
                break;

            // ── Case 25: arrived 52,84 (end Path13) → open stopper, start timer
            case 25:
                if (!follower.isBusy()) {
                    stopperController.currentStatus = StopperController.StopperStatus.SHOOT;
                    timerShoot.reset();
                    setPathState(26);
                }
                break;

            // ── Case 26: shoot; t>0.15s intake ON; t>1.0s close, park Path14
            case 26:
                if (timerShoot.seconds() > 0.15) {
                    intakeController.currentStatus   = IntakeController.IntakeStatus.COLLECT;
                    transferController.currentStatus = TransferController.TransferStatus.COLLECT;
                }
                if (timerShoot.seconds() > 1.0) {
                    targetVelocity = 0;
                    stopperController.currentStatus  = StopperController.StopperStatus.NOSHOOT;
                    follower.followPath(paths.Path14);
                    intakeController.currentStatus   = IntakeController.IntakeStatus.OFF;
                    transferController.currentStatus = TransferController.TransferStatus.OFF;
                    forbarController.currentStatus   = ForbarController.ForbarStatus.UP;
                    setPathState(27);
                }
                break;

            // ── Case 27: park (52,64 → 47.704,79.697); done ─────────────────
            case 27:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    private void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
    }

    // =========================================================================
    //  LIFECYCLE
    // =========================================================================
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        motor  = hardwareMap.get(DcMotorEx.class, "flywheelMotorR");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor1 = hardwareMap.get(DcMotorEx.class, "flywheelMotorL");
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);

        controller = new PIDFController(P, I, kD, 0.0);

        RobotMap robot = new RobotMap(hardwareMap);

        intakeController   = new IntakeController(robot);
        transferController = new TransferController(robot);
        hoodController     = new HoodController(robot);
        stopperController  = new StopperController(robot);
        brakesController   = new BrakesController(robot);
        turretController   = new TurretController(robot);
        forbarController   = new ForbarController(robot);

        intakeController.update();
        transferController.update();
        hoodController.update(hoodTargetPos);
        stopperController.update();
        turretController.update(turret_target_position);
        forbarController.update();

        pathTimer   = new Timer();
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
        double power = controller.calculate(targetVelocity - velocity);
        motor.setPower(power);
        motor1.setPower(power);

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

        double clampedDegrees = Math.max(-TURRET_MAX_DEGREES,
                Math.min(TURRET_MAX_DEGREES, degreesToTurn));
        turret_target_position = 0.5 + (clampedDegrees / 355.0);
        turret_target_position += turretOffset;
        turret_target_position = Math.max(0.0, Math.min(1.0, turret_target_position));

        targetVelocity = flywheelSpeedClose(distanceToGoal)-40;
        hoodTargetPos  = hoodAngleClose(distanceToGoal);

        endPose = follower.getPose();

        turretController.currentStatus = TurretController.TurretStatus.RUNTO;

        intakeController.update();
        transferController.update();
        hoodController.update(hoodTargetPos);
        stopperController.update();
        turretController.update(turret_target_position);
        forbarController.update();

        telemetry.addData("Path State",        pathState);
        telemetry.addData("timerShoot",        timerShoot.seconds());
        telemetry.addData("timerDelay",        timerDelay.seconds());
        telemetry.addData("distanceToGoal",    distanceToGoal);
        telemetry.addData("targetVelocity",    targetVelocity);
        telemetry.addData("velocity",          velocity);
        telemetry.addData("hoodTargetPos",     hoodTargetPos);
        telemetry.addData("degreesToTurn",     degreesToTurn);
        telemetry.addData("turret_target_pos", turret_target_position);
        telemetry.addData("turretOffset",      turretOffset);
        telemetry.addData("X",                 follower.getPose().getX());
        telemetry.addData("Y",                 follower.getPose().getY());
        telemetry.addData("Heading",           Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    public static Pose getEndPose() { return endPose; }

    @Override
    public void stop() {
        endPose = follower.getPose();
        follower.breakFollowing();
    }

    // =========================================================================
    //  PATH DEFINITIONS
    // =========================================================================
    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6;
        public PathChain Path7, Path8, Path9, Path10, Path11, Path12;
        public PathChain Path13, Path14;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(32.000, 136.000), new Pose(52.000, 84.000))
            ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180)).build();

            Path2 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(52.000, 84.000), new Pose(26.681, 83.808))
            ).setConstantHeadingInterpolation(Math.toRadians(180)).build();

            Path3 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(26.681, 83.808), new Pose(52.000, 84.000))
            ).setConstantHeadingInterpolation(Math.toRadians(180)).build();

            Path4 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(52.000, 84.000),
                            new Pose(55.906, 55.198),
                            new Pose(24.882, 60.038))
            ).setConstantHeadingInterpolation(Math.toRadians(180)).build();

            Path5 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(24.882, 60.038), new Pose(52.000, 84.000))
            ).setConstantHeadingInterpolation(Math.toRadians(180)).build();

            Path6 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(52.000, 84.000), new Pose(15.000, 62.000))
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(152)).build();

            Path7 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(15.000, 62.000), new Pose(52.000, 84.000))
            ).setLinearHeadingInterpolation(Math.toRadians(152), Math.toRadians(180)).build();

            Path8 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(52.000, 84.000),
                            new Pose(53.546, 29.194),
                            new Pose(24.356, 35.791))
            ).setConstantHeadingInterpolation(Math.toRadians(180)).build();

            Path9 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(24.356, 35.791), new Pose(52.000, 84.000))
            ).setConstantHeadingInterpolation(Math.toRadians(180)).build();

            Path10 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(52.000, 84.000),
                            new Pose(10.685, 65.626),
                            new Pose(8.855, 41.721))
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270)).build();

            Path11 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(8.855, 41.721), new Pose(52.000, 84.000))
            ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180)).build();

            Path12 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(52.000, 84.000),
                            new Pose(11.572, 65.747),
                            new Pose(8.026, 16.108))
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270)).build();

            Path13 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(8.026, 16.108), new Pose(52.000, 84.000))
            ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180)).build();

            Path14 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(52.000, 84.000), new Pose(47.704, 79.697))
            ).setConstantHeadingInterpolation(Math.toRadians(180)).build();
        }
    }
}