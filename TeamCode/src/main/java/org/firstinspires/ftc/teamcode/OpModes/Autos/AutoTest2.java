package org.firstinspires.ftc.teamcode.OpModes.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class AutoTest2 extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;
    private int pathState;
    private DcMotorEx flywheel1;
    private DcMotorEx flywheel2;
    private DcMotorEx launcher;
    private DcMotorEx intake;
    private final Pose startPose = new Pose(20, 122, Math.toRadians(135));
    private final Pose shootPose = new Pose(56.148939736477416, 87.10958304886576, Math.toRadians(135));
    private final Pose ballPickup1 = new Pose(15, 84, Math.toRadians(180));
    private final Pose openGate = new Pose(15, 70, Math.toRadians(180));
    private final Pose gatePathControlPoint = new Pose(35, 76);
    private PathChain startToShoot;
    private PathChain shootToPickup;
    private PathChain pickupToGate;
    private PathChain gateToShoot;
    public void pathBuilder() {
        startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        shootToPickup = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, ballPickup1))
                .setLinearHeadingInterpolation(ballPickup1.getHeading(), ballPickup1.getHeading())
                .build();
        pickupToGate = follower.pathBuilder()
                .addPath(new BezierCurve(ballPickup1, gatePathControlPoint, openGate))
                .setLinearHeadingInterpolation(ballPickup1.getHeading(), openGate.getHeading())
                .build();
        gateToShoot = follower.pathBuilder()
                .addPath(new BezierLine(openGate, shootPose))
                .setLinearHeadingInterpolation(openGate.getHeading(), shootPose.getHeading())
                .build();
    }
    private static final double LAUNCHER_POWER           = 0.4;   // reduce if still overshooting
    private static final double TARGET_FLYWHEEL_VELOCITY = 1420;   // ticks/sec – tune up/down as needed
    private static final double FIRE_DURATION            = 0.5;  // seconds per ball
    private static final double PAUSE_DURATION           = 2;   // seconds between balls
    private static final double FLYWHEEL_READY_TIMEOUT   = 4.0;   // fallback timeout (seconds)

    private boolean flywheelsAtSpeed() {
        return Math.abs(flywheel1.getVelocity()) >= TARGET_FLYWHEEL_VELOCITY * 0.95
                && Math.abs(flywheel2.getVelocity()) >= TARGET_FLYWHEEL_VELOCITY * 0.95;
    }

    private void spinFlywheels() {
        flywheel1.setVelocity(TARGET_FLYWHEEL_VELOCITY);
        flywheel2.setVelocity(TARGET_FLYWHEEL_VELOCITY);
    }

    private void stopFlywheels() {
        flywheel1.setVelocity(0);
        flywheel2.setVelocity(0);
    }

    public void updatePathState() {
        switch (pathState) {
            case 0:
                follower.followPath(startToShoot);
                spinFlywheels();
                intake.setPower(-1);
                pathTimer.resetTimer();
                pathState = 1;
                break;
            case 1:
                if (follower.atPose(shootPose, 2, 2) || pathTimer.getElapsedTimeSeconds() > 5) {
                    pathTimer.resetTimer();
                    pathState = 2;
                }
                break;
            case 2:
                if (flywheelsAtSpeed() || pathTimer.getElapsedTimeSeconds() >= FLYWHEEL_READY_TIMEOUT) {
                    launcher.setPower(LAUNCHER_POWER);
                    pathTimer.resetTimer();
                    pathState = 3;
                }
                break;
            case 3:
                if (pathTimer.getElapsedTimeSeconds() >= FIRE_DURATION) {
                    launcher.setPower(0);
                    pathTimer.resetTimer();
                    pathState = 4;
                }
                break;
            case 4:
                if (pathTimer.getElapsedTimeSeconds() >= PAUSE_DURATION) {
                    launcher.setPower(LAUNCHER_POWER);
                    pathTimer.resetTimer();
                    pathState = 5;
                }
                break;
            case 5:
                if (pathTimer.getElapsedTimeSeconds() >= FIRE_DURATION) {
                    launcher.setPower(0);
                    pathTimer.resetTimer();
                    pathState = 6;
                }
                break;
            case 6:
                if (pathTimer.getElapsedTimeSeconds() >= PAUSE_DURATION) {
                    launcher.setPower(LAUNCHER_POWER);
                    pathTimer.resetTimer();
                    pathState = 7;
                }
                break;
            case 7:
                if (pathTimer.getElapsedTimeSeconds() >= FIRE_DURATION) {
                    launcher.setPower(0);
                    stopFlywheels();
                    intake.setPower(-1);
                    follower.followPath(shootToPickup);
                    pathTimer.resetTimer();
                    pathState = 8;
                }
                break;
            case 8:
                if (follower.atPose(ballPickup1, 2, 2) || pathTimer.getElapsedTimeSeconds() > 5) {
                    follower.followPath(pickupToGate);
                    pathTimer.resetTimer();
                    pathState = 9;
                }
                break;
            case 9:
                if (follower.atPose(openGate, 2, 2) || pathTimer.getElapsedTimeSeconds() > 5) {
                    follower.followPath(gateToShoot);
                    spinFlywheels();
                    pathTimer.resetTimer();
                    pathState = 10;
                }
                break;
            case 10:
                if (!follower.isBusy() || follower.atPose(shootPose, 2, 2) || pathTimer.getElapsedTimeSeconds() > 5) {
                    pathTimer.resetTimer();
                    pathState = 11;
                }
                break;
            case 11:
                if (flywheelsAtSpeed() || pathTimer.getElapsedTimeSeconds() >= FLYWHEEL_READY_TIMEOUT) {
                    launcher.setPower(LAUNCHER_POWER);
                    pathTimer.resetTimer();
                    pathState = 12;
                }
                break;
            case 12:
                if (pathTimer.getElapsedTimeSeconds() >= FIRE_DURATION) {
                    launcher.setPower(0);
                    pathTimer.resetTimer();
                    pathState = 13;
                }
                break;
            case 13:
                if (pathTimer.getElapsedTimeSeconds() >= PAUSE_DURATION) {
                    launcher.setPower(LAUNCHER_POWER);
                    pathTimer.resetTimer();
                    pathState = 14;
                }
                break;
            case 14:
                if (pathTimer.getElapsedTimeSeconds() >= FIRE_DURATION) {
                    launcher.setPower(0);
                    pathTimer.resetTimer();
                    pathState = 15;
                }
                break;
            case 15:
                if (pathTimer.getElapsedTimeSeconds() >= PAUSE_DURATION) {
                    launcher.setPower(LAUNCHER_POWER);
                    pathTimer.resetTimer();
                    pathState = 16;
                }
                break;
            case 16:
                if (pathTimer.getElapsedTimeSeconds() >= FIRE_DURATION) {
                    launcher.setPower(0);
                    stopFlywheels();
                    pathState = 17;
                }
                break;
            case 17:
                break;
        }
    }
    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        pathBuilder();
        follower.setPose(startPose);
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        launcher = hardwareMap.get(DcMotorEx.class, "Launcher");
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake = hardwareMap.get(DcMotorEx.class, "Intake");
        stopFlywheels();
    }
    public void start() {
        pathTimer.resetTimer();
        opModeTimer.resetTimer();
        pathState = 0;
        stopFlywheels();
        launcher.setPower(0.0);
    }
    @Override
    public void stop() {
        stopFlywheels();
        launcher.setPower(0.0);
    }
    @Override
    public void loop() {
        follower.update();
        updatePathState();
        telemetry.addLine("Path Timer:" + pathTimer.getElapsedTime());
        telemetry.addLine("Path State:" + pathState);
        telemetry.addData("FW1 velocity", flywheel1.getVelocity());
        telemetry.addData("FW2 velocity", flywheel2.getVelocity());
        telemetry.addData("Flywheels ready", flywheelsAtSpeed());
        telemetry.update();
    }
}
