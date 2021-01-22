package org.firstinspires.ftc.teamcode.drive.opmode.BERSERK;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
/**
 * This opmode demonstrates how to create a teleop using just the SampleMecanumDrive class without
 * the need for an external robot class. This will allow you to do some cool things like
 * incorporating live trajectory following in your teleop. Check out TeleOpAgumentedDriving.java for
 * an example of such behavior.
 * <p>
 * This opmode is essentially just LocalizationTest.java with a few additions and comments.
 */
@TeleOp(group = "BERSERK")
public class TeleopBERSERK extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        HardwareBERSERK robot       = new HardwareBERSERK();
        robot.init(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FtcDashboard dashboard = FtcDashboard.getInstance();

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

            //DRIVING CONTROL
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

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

            // READ POSE
            Pose2d poseEstimate = drive.getPoseEstimate();

            // TELEMETRY
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("launch angle", launch_angle);
            telemetry.addData("shooter rpm", ((DcMotorEx) robot.shooter1).getVelocity());
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("FlyWheel Velocity", Math.abs(((DcMotorEx) robot.shooter1).getVelocity()));
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
