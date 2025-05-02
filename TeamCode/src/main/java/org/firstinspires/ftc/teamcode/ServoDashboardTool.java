package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name = "Driver Controlled with Dashboard", group = "Robot")
public class ServoDashboardTool extends LinearOpMode {

    HardwareRobot robot = new HardwareRobot();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        // Initialize FTC Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry.addData("Status", "Waiting for Start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Read live values from FTC Dashboard
            robot.bucketgrab.setPosition(RobotConfig.bucketGrabPosition);
            robot.bucketart.setPosition(RobotConfig.bucketArtPosition);
            robot.bucketrotate.setPosition(RobotConfig.bucketRotatePosition);

            robot.servointake.setPosition(RobotConfig.servoIntakePosition);
            robot.servoswing.setPosition(RobotConfig.servoSwingPosition);
            robot.servorotate.setPosition(RobotConfig.servoRotatePosition);
            robot.servoart.setPosition(RobotConfig.servoArtPosition);
            robot.servosweep.setPosition(RobotConfig.servoSweepPosition);
            // Send telemetry data to FTC Dashboard
            telemetry.addData("Servo Intake Pose", RobotConfig.servoIntakePosition);
            telemetry.addData("Servo Rotate", RobotConfig.servoRotatePosition);
            telemetry.addData("Servo Artic", RobotConfig.servoArtPosition);
            telemetry.addData("Servo Swing", RobotConfig.servoSwingPosition);


            telemetry.addData("Bucket Grab", RobotConfig.bucketGrabPosition);
            telemetry.addData("Bucket Articulation", RobotConfig.bucketArtPosition);
            telemetry.addData("Bucket Rotate", RobotConfig.bucketRotatePosition);
            telemetry.addData("servo_sweep", RobotConfig.servoSweepPosition);
            telemetry.update();
        }
    }
}
