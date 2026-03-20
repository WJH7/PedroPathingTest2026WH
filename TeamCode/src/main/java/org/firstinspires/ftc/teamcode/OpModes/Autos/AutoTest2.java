package org.firstinspires.ftc.teamcode.OpModes.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class AutoTest2 extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;
    private int pathState;
    private int nextStateAfterShooting;
    private DcMotorEx flywheel1;
    private DcMotorEx flywheel2;
    private DcMotorEx launcher;
    private DcMotorEx intake;

    private final Pose startPose          = new Pose(20, 122, Math.toRadians(135));
    private final Pose shootPose          = new Pose(56.148939736477416, 91, Math.toRadians(141));
    private final Pose shootPose2         = new Pose(53, 91, Math.toRadians(138)); // second round — 3 units left
    private final Pose finalShootPose     = new Pose(51, 85, Math.toRadians(135)); // tune as needed
    private final Pose ballPickup1        = new Pose(15, 84, Math.toRadians(180));
    private final Pose shootTurnPose      = new Pose(53, 87, Math.toRadians(180)); // turn in place before driving to pickup
    private final Pose openGate           = new Pose(15, 70, Math.toRadians(180));
    private final Pose gatePathControlPoint = new Pose(35, 76);

    private PathChain startToShoot;
    private PathChain shootToPickup;
    private PathChain pickupToGate;
    private PathChain gateToShoot;
    private PathChain shootToFinalShoot;

    public void pathBuilder() {
        startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        shootToPickup = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, shootTurnPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), Math.toRadians(180))
                .addPath(new BezierLine(shootTurnPose, ballPickup1))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        pickupToGate = follower.pathBuilder()
                .addPath(new BezierCurve(ballPickup1, gatePathControlPoint, openGate))
                .setLinearHeadingInterpolation(ballPickup1.getHeading(), openGate.getHeading())
                .build();
        gateToShoot = follower.pathBuilder()
                .addPath(new BezierLine(openGate, shootPose2))
                .setLinearHeadingInterpolation(openGate.getHeading(), shootPose2.getHeading())
                .build();
        shootToFinalShoot = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, finalShootPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), finalShootPose.getHeading())
                .build();
    }

    private static final int    NUM_SHOTS                 = 3;
    private static final double LAUNCHER_POWER            = 0.45;   // reduce if still overshooting
    private static final double TARGET_FLYWHEEL_VELOCITY  = 1400;  // ticks/sec — tune up/down as needed
    private static final double FIRE_DURATION             = 0.5;   // seconds per ball
    private static final double PAUSE_DURATION            = 1.75;   // seconds between balls

    // Flywheel PIDF — tune KF first until steady-state, then KP for small corrections
    private static final double FW_KF = 0.00045; // feedforward: base power per tick/sec of target
    private static final double FW_KP = 0.001;  // proportional correction on top of feedforward
    private static final double FW_KI = 0.00002; // integral — keep small, windup causes oscillation
    private static final double FW_KD = 0.00001; // derivative — keep small or zero
    private static final double FW_MAX_INTEGRAL = 0.3; // clamp to prevent windup
    private boolean flywheelsEnabled = false;
    private double fwIntegral    = 0;
    private double fwLastError   = 0;
    private long   fwLastNanos   = 0;
    // PID debug values — read by loop() for telemetry
    private double fwVel1        = 0;
    private double fwVel2        = 0;
    private double fwAvgVel      = 0;
    private double fwError       = 0;
    private double fwPTerm       = 0;
    private double fwITerm       = 0;
    private double fwDTerm       = 0;
    private double fwOutput      = 0;

    private void updateFlywheelPID() {
        fwVel1 = Math.abs(flywheel1.getVelocity());
        fwVel2 = Math.abs(flywheel2.getVelocity());
        if (!flywheelsEnabled) {
            flywheel1.setPower(0);
            flywheel2.setPower(0);
            fwIntegral  = 0;
            fwLastError = 0;
            fwLastNanos = System.nanoTime();
            fwAvgVel = fwError = fwPTerm = fwITerm = fwDTerm = fwOutput = 0;
            return;
        }
        long now = System.nanoTime();
        double dt = (now - fwLastNanos) / 1e9;
        fwLastNanos = now;
        if (dt <= 0) return;

        fwAvgVel = (fwVel1 + fwVel2) / 2.0;
        fwError  = TARGET_FLYWHEEL_VELOCITY - fwAvgVel;

        fwIntegral += fwError * dt;
        double derivative = (fwError - fwLastError) / dt;
        fwLastError = fwError;

        fwIntegral = Math.max(-FW_MAX_INTEGRAL / FW_KI, Math.min(FW_MAX_INTEGRAL / FW_KI, fwIntegral));
        double feedforward = FW_KF * TARGET_FLYWHEEL_VELOCITY;
        fwPTerm = FW_KP * fwError;
        fwITerm = FW_KI * fwIntegral;
        fwDTerm = FW_KD * derivative;
        fwOutput = Math.max(0, Math.min(1, feedforward + fwPTerm + fwITerm + fwDTerm));

        flywheel1.setPower(fwOutput);
        flywheel2.setPower(fwOutput);
    }

    private boolean flywheelsAtSpeed() {
        return Math.abs(flywheel1.getVelocity()) >= TARGET_FLYWHEEL_VELOCITY * 0.95
                && Math.abs(flywheel2.getVelocity()) >= TARGET_FLYWHEEL_VELOCITY * 0.95;
    }

    private void spinFlywheels() {
        fwLastNanos = System.nanoTime();
        flywheelsEnabled = true;
    }

    private void stopFlywheels() {
        flywheelsEnabled = false;
    }

    // States 2, 3, 4 are shared for both shooting rounds.
    // nextStateAfterShooting controls where the machine goes once NUM_SHOTS are complete.
    public void updatePathState() {
        switch (pathState) {
            case 0: // begin: drive to shoot pose while spinning flywheels
                follower.followPath(startToShoot);
                spinFlywheels();
                intake.setPower(-1);
                pathTimer.resetTimer();
                pathState = 1;
                break;
            case 1: // wait to arrive at shoot pose (first round)
                if (follower.atPose(shootPose, 2, 2) || pathTimer.getElapsedTimeSeconds() > 5) {
                    nextStateAfterShooting = 5;
                    pathTimer.resetTimer();
                    pathState = 2;
                }
                break;
            case 2: // fixed settle wait, then start shot sequence
                if (pathTimer.getElapsedTimeSeconds() >= 1.5) {
                    pathTimer.resetTimer(); // t=0 is now the shot sequence start
                    pathState = 3;
                }
                break;
            case 3: // all shots managed from a single running clock — no per-shot resets
                double shotPeriod = FIRE_DURATION + PAUSE_DURATION;
                double t = pathTimer.getElapsedTimeSeconds();
                int currentShot = (int) (t / shotPeriod);
                double timeInShot = t - currentShot * shotPeriod;
                if (currentShot < NUM_SHOTS) {
                    launcher.setPower(timeInShot < FIRE_DURATION ? LAUNCHER_POWER : 0);
                } else {
                    launcher.setPower(0);
                    stopFlywheels();
                    pathState = nextStateAfterShooting;
                }
                break;
            case 5: // first round done — drive to pickup
                intake.setPower(-1);
                follower.followPath(shootToPickup);
                pathTimer.resetTimer();
                pathState = 6;
                break;
            case 6: // wait at pickup, then drive to gate
                if (follower.atPose(ballPickup1, 2, 2) || pathTimer.getElapsedTimeSeconds() > 5) {
                    follower.followPath(pickupToGate);
                    pathTimer.resetTimer();
                    pathState = 7;
                }
                break;
            case 7: // wait at gate, then drive back to shoot pose
                if (follower.atPose(openGate, 2, 2) || pathTimer.getElapsedTimeSeconds() > 5) {
                    follower.followPath(gateToShoot);
                    spinFlywheels();
                    pathTimer.resetTimer();
                    pathState = 8;
                }
                break;
            case 8: // wait to arrive at shoot pose (second round) — 5s settle, abort if off-position
                if (follower.atPose(shootPose2, 2, 2) && pathTimer.getElapsedTimeSeconds() >= 5) {
                    nextStateAfterShooting = 10;
                    pathTimer.resetTimer();
                    pathState = 2;
                } else if (pathTimer.getElapsedTimeSeconds() > 11) {
                    stopFlywheels();
                    launcher.setPower(0);
                    intake.setPower(0);
                    pathState = 11;
                }
                break;
            case 9: // nudge to finalShootPose for the last shot
                if (follower.atPose(finalShootPose, 2, 2) || pathTimer.getElapsedTimeSeconds() > 3) {
                    launcher.setPower(LAUNCHER_POWER);
                    pathTimer.resetTimer();
                    pathState = 10;
                }
                break;
            case 10: // fire the very last shot from finalShootPose
                if (pathTimer.getElapsedTimeSeconds() >= FIRE_DURATION) {
                    launcher.setPower(0);
                    stopFlywheels();
                    intake.setPower(0);
                    pathState = 11;
                }
                break;
            case 11: // terminal — all done
                intake.setPower(0.0);
                requestOpModeStop();
                break;
        }
    }

    @Override
    public void init() {
        pathTimer   = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        pathBuilder();
        follower.setPose(startPose);
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        launcher  = hardwareMap.get(DcMotorEx.class, "Launcher");
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake = hardwareMap.get(DcMotorEx.class, "Intake");
        intake.setPower(0.0);
        stopFlywheels();
    }

    public void start() {
        pathTimer.resetTimer();
        opModeTimer.resetTimer();
        pathState = 0;
        spinFlywheels();
        launcher.setPower(0.0);
        intake.setPower(0.0);
    }

    @Override
    public void stop() {
        stopFlywheels();
        launcher.setPower(0.0);
        intake.setPower(0.0);
    }

    @Override
    public void loop() {
        follower.update();
        updatePathState();
        updateFlywheelPID();
        telemetry.addLine("--- Path ---");
        telemetry.addData("State", pathState);
        telemetry.addData("Path Timer", "%.2f s", pathTimer.getElapsedTimeSeconds());
        telemetry.addLine("--- Flywheel PID ---");
        telemetry.addData("Target vel", TARGET_FLYWHEEL_VELOCITY);
        telemetry.addData("FW1 vel",   "%.1f", fwVel1);
        telemetry.addData("FW2 vel",   "%.1f", fwVel2);
        telemetry.addData("Avg vel",   "%.1f", fwAvgVel);
        telemetry.addData("Error",     "%.1f", fwError);
        telemetry.addData("P term",    "%.4f", fwPTerm);
        telemetry.addData("I term",    "%.4f", fwITerm);
        telemetry.addData("D term",    "%.4f", fwDTerm);
        telemetry.addData("Output",    "%.3f", fwOutput);
        telemetry.addData("At speed",  flywheelsAtSpeed());
        telemetry.update();
    }
}
