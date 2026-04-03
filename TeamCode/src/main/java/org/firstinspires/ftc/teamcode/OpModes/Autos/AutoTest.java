package org.firstinspires.ftc.teamcode.OpModes.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotParts.Flywheel;
import org.firstinspires.ftc.teamcode.RobotParts.Intake;
import org.firstinspires.ftc.teamcode.RobotParts.Launcher;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class AutoTest extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer, shootTimer;
    private int pathState;
    private Intake intake;
    private Flywheel flywheel;
    private Launcher launcher;
    private int timesShot;
    private boolean shootReset;
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
    public void updatePathState() {
        switch (pathState) {
            case 0:
                pathTimer.resetTimer();
                follower.followPath(startToShoot);
                pathState = 4;
                break;
            case 1:
                if (pathTimer.getElapsedTimeSeconds() > 2 || follower.atPose(shootPose, 2, 2)) {
                    pathTimer.resetTimer();
                    follower.followPath(shootToPickup);
                    pathState = 2;
                }
                break;
            case 2:
                if (pathTimer.getElapsedTimeSeconds() > 3 || follower.atPose(openGate, 2, 2)) {
                    pathTimer.resetTimer();
                    follower.followPath(pickupToGate);
                    pathState = 3;
                }
                break;
            case 3:
                if (pathTimer.getElapsedTimeSeconds() > 2 || follower.atPose(shootPose, 2, 2)) {
                    pathTimer.resetTimer();
                    follower.followPath(gateToShoot);
                    pathState = 4;
                }
                break;
            case 4:
                if (pathTimer.getElapsedTimeSeconds() > 3.5 || follower.atPose(shootPose, 2, 2)) {
                    launcher.on();
                    if (shootReset) {
                        shootTimer.resetTimer();
                        shootReset = false;
                    }
                    if (shootTimer.getElapsedTimeSeconds() > 2) {
                        telemetry.addLine("shootTimer works");
                        launcher.off();
                        if (timesShot == 0) {
                            pathState = 1;
                        } else if (timesShot == 1) {
                            pathState = 5;
                        }
                        pathTimer.resetTimer();
                        timesShot++;
                        shootReset = true;
                    }
                }
                break;
        }
    }
    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();
        shootTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        pathBuilder();
        follower.setPose(startPose);
        intake = new Intake(hardwareMap);
        flywheel = new Flywheel(hardwareMap, 64, 26, 14);
        launcher = new Launcher(hardwareMap);
    }
    public void start() {
        pathTimer.resetTimer();
        opModeTimer.resetTimer();
        pathState = 0;
        timesShot = 0;
        flywheel.on();
        intake.on();
        shootReset = true;
    }
    @Override
    public void loop() {
        follower.update();
        updatePathState();
        telemetry.addLine("Path Timer: " + pathTimer.getElapsedTimeSeconds());
        telemetry.addLine("Path State: " + pathState);
        telemetry.addLine("Times Shot: " + timesShot);
        telemetry.addLine("ShootTimer: " + shootTimer.getElapsedTimeSeconds());
        telemetry.update();
    }
}
