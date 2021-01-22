package org.firstinspires.ftc.teamcode.drive.opmode.BERSERK;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

/**
 * This opmode demonstrates how one would implement "align to point behavior" in teleop. You specify
 * a desired vector (x/y coordinate) via`targetPosition`. In the `ALIGN_TO_POINT` mode, the bot will
 * switch into field centric control and independently control its heading to align itself with the
 * specified `targetPosition`.
 * <p>
 * Press `a` to switch into alignment mode and `b` to switch back into standard teleop driving mode.
 * <p>
 * Note: We don't call drive.update() here because it has its own field drawing functions. We don't
 * want that to interfere with our graph so we just directly update localizer instead
 */
@Config
@TeleOp(group = "advanced")
public class TeleopBERSERK_v2 extends LinearOpMode {

    //public static double DRAWING_TARGET_RADIUS = 2;
    public static double DRAWING_TARGET_RADIUS = 1;

    enum Mode {
        NORMAL_CONTROL,
        ALIGN_TO_POINT
    }

    private Mode currentMode = Mode.NORMAL_CONTROL;
    private PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);

    //Target Position of TowerGoal
    private Vector2d targetPosition = new Vector2d(75, 40);

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        HardwareBERSERK robot       = new HardwareBERSERK();
        robot.init(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FtcDashboard dashboard = FtcDashboard.getInstance();

        //Start Pose
        drive.getLocalizer().setPoseEstimate(PoseStorage.currentPose);
        headingController.setInputBounds(-Math.PI, Math.PI);

        waitForStart();

        //STATE VARIABLES
        double shooter_target_velo = 1800;
        double launch_angle = 0.174;
        double max_launch_angle = 0.2;
        double min_launch_angle = 0.113;
        double kicker_out = 0.7;
        double kicker_in = 0.2;
        double wobble_close = 0.45;
        double wobble_open = 1;
        double wobble_up = 0.6;
        double wobble_down = 0.16;
        long shootWait = 150;

        //SET SERVOS
        robot.wobble_lift.setPosition(wobble_up);
        robot.wobble_claw.setPosition(wobble_close);
        robot.kicker.setPosition(kicker_out);

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            Pose2d poseEstimate = drive.getLocalizer().getPoseEstimate();
            Pose2d driveDirection = new Pose2d();
            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();

            //INTAKE+INDEXER
            if (gamepad1.right_bumper){
                robot.intake.setPower(1);
                robot.feeder_turn.setPower(1);
            }
            else if (gamepad1.left_bumper){
                robot.intake.setPower(0);
                robot.feeder_turn.setPower(0);
            }

            //SHOOTER
            if (gamepad1.a || gamepad2.a) {
                ((DcMotorEx) robot.shooter1).setVelocity(shooter_target_velo);
                //((DcMotorEx) robot.shooter2).setVelocity(((DcMotorEx) robot.shooter1).getVelocity());

                //B Turns off Intake, Indexer, and Shooter
            } else if (gamepad1.b || gamepad2.b) {
                //  robot.intake.setPower(0);
                //  robot.feeder_turn.setPower(0);
                ((DcMotorEx) robot.shooter1).setVelocity(0);
                ((DcMotorEx) robot.shooter2).setVelocity(0);
            }

            // FLAP
            robot.flap.setPosition(Math.min(Math.max(launch_angle, min_launch_angle), max_launch_angle));

            if (gamepad1.dpad_up && launch_angle >= min_launch_angle) {
                launch_angle += -0.0002;
            }
            else if (gamepad1.dpad_down && launch_angle <= max_launch_angle) {
                launch_angle += 0.0002;
            }
            // Y prepares for endgame by dropping flap and powering up shooter
            else if (gamepad1.y || gamepad2.y) {
                launch_angle= 0.2;
                ((DcMotorEx) robot.shooter1).setVelocity(shooter_target_velo);
                //((DcMotorEx) robot.shooter2).setVelocity(((DcMotorEx) robot.shooter1).getVelocity());
            }

            // WOBBLE ARM
            if (gamepad2.dpad_up) {
                robot.wobble_lift.setPosition(wobble_up);
            }
            else if (gamepad2.dpad_down) {
                robot.wobble_lift.setPosition(wobble_down);
            }
            else if (gamepad2.dpad_left) {
                robot.wobble_claw.setPosition(wobble_open);
            }
            else if (gamepad2.dpad_right) {
                robot.wobble_claw.setPosition(wobble_close);
            }

            //FEEDER SERVO
            if (gamepad1.x) {
                robot.kicker.setPosition(kicker_in);
                sleep(shootWait);
                robot.kicker.setPosition(kicker_out);
            }

            switch (currentMode) {
                case NORMAL_CONTROL:
                    // Switch to normal control if gamepad1 right stick is moved
                    if (Math.abs(gamepad1.right_stick_x) >= 0) {
                        currentMode = Mode.ALIGN_TO_POINT;
                    }

                    //(gamepad1.right_stick_x <= 0 || gamepad1.right_stick_x >= 0)

                    driveDirection = new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x

                        //    -Math.log(gamepad1.left_stick_y),
                        //    -Math.log(gamepad1.left_stick_x),
                        //    -Math.log(gamepad1.right_stick_x)
                    );
                    break;
                case ALIGN_TO_POINT:
                    // Switch to align to point if gamepad1 right trigger is activated
                    if (Math.abs(gamepad1.right_trigger) >= 0) {
                        currentMode = Mode.NORMAL_CONTROL;
                    }

                    Vector2d fieldFrameInput = new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    );
                    Vector2d robotFrameInput = fieldFrameInput.rotated(-poseEstimate.getHeading());
                    Vector2d difference = targetPosition.minus(poseEstimate.vec());
                    double theta = difference.angle();
                    double thetaFF = -fieldFrameInput.rotated(-Math.PI / 2).dot(difference) / (difference.norm() * difference.norm());
                    headingController.setTargetPosition(theta);

                    double headingInput = (headingController.update(poseEstimate.getHeading())
                            * DriveConstants.kV + thetaFF)
                            * DriveConstants.TRACK_WIDTH;

                    driveDirection = new Pose2d(
                            robotFrameInput,
                            headingInput
                    );

                    fieldOverlay.setStroke("#dd2c00");
                    fieldOverlay.strokeCircle(targetPosition.getX(), targetPosition.getY(), DRAWING_TARGET_RADIUS);
                    fieldOverlay.setStroke("#b89eff");
                    fieldOverlay.strokeLine(targetPosition.getX(), targetPosition.getY(), poseEstimate.getX(), poseEstimate.getY());
                    fieldOverlay.setStroke("#ffce7a");
                    fieldOverlay.strokeLine(targetPosition.getX(), targetPosition.getY(), targetPosition.getX(), poseEstimate.getY());
                    fieldOverlay.strokeLine(targetPosition.getX(), poseEstimate.getY(), poseEstimate.getX(), poseEstimate.getY());
                    break;
            }

            fieldOverlay.setStroke("#3F51B5");
            DashboardUtil.drawRobot(fieldOverlay, poseEstimate);
            drive.setWeightedDrivePower(driveDirection);
            headingController.update(poseEstimate.getHeading());
            drive.getLocalizer().update();
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("launch angle", launch_angle);
            telemetry.addData("shooter rpm", ((DcMotorEx) robot.shooter1).getVelocity());
            telemetry.addData("mode", currentMode);
            telemetry.update();

            packet.put("FlyWheel Velocity", Math.abs(((DcMotorEx) robot.shooter1).getVelocity()));
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
