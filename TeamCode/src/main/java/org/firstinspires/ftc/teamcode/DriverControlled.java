package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp (name = "Driver Controlled", group = "Robot")

public class DriverControlled extends LinearOpMode {

    HardwareRobot robot = new HardwareRobot();

    
@Override
public void runOpMode() {
        robot.init(hardwareMap);

        int newLiftTargetLeft;
        int newLiftTargetRight;

        RevBlinkinLedDriver.BlinkinPattern pattern;
        pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        RevBlinkinLedDriver.BlinkinPattern patternPrime;
        patternPrime = RevBlinkinLedDriver.BlinkinPattern.GREEN;

        BNO055IMU imu;

        Orientation angles;

       // robot.init(hardwareMap);


        //IMU Initialization
                BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
                parameters.mode = BNO055IMU.SensorMode.IMU;
                parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
                parameters.loggingEnabled = false;

                imu = hardwareMap.get(BNO055IMU.class, "imu");
                imu.initialize(parameters);

                telemetry.addData("Mode", "calibrating...");
                telemetry.update();

                // make sure the imu gyro is calibrated before continuing.
                while (!isStopRequested() && !imu.isGyroCalibrated()) {
                    sleep(50);
                    idle();
                }


        telemetry.addData("Say", "Waiting for Start");
        telemetry.update();

        robot.liftup.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftout.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    waitForStart();

        newLiftTargetLeft = robot.liftup.getCurrentPosition();
        newLiftTargetRight = robot.liftout.getCurrentPosition();
        robot.liftup.setTargetPosition(newLiftTargetLeft);
        robot.liftout.setTargetPosition(newLiftTargetRight);

        double dropperPosition = .53;//.5

        while (opModeIsActive()){


        double Turn = gamepad1.left_stick_x;
        double Speed = -gamepad1.left_stick_y;
        double Strafe = -gamepad1.right_stick_x;

        double front_left = Speed + Turn - Strafe;
        double front_right = Speed - Turn + Strafe;
        double rear_left = Speed + Turn + Strafe;
        double rear_right = Speed - Turn - Strafe;
    
        front_left = Range.clip(front_left, -1, 1);
        front_right = Range.clip(front_right, -1, 1);
        rear_left = Range.clip(rear_left, -1, 1);
        rear_right = Range.clip(rear_right, -1, 1);

        front_left = (float)scaleInput(front_left);
        front_right = (float)scaleInput(front_right);
        rear_left = (float)scaleInput(rear_left);
        rear_right = (float)scaleInput(rear_right);

        front_left /=2;
        front_right /=2;
        rear_left /=2;
        rear_right /=2;

        if (gamepad1.right_bumper){
        front_left *=2;
        front_right *=2;
        rear_left *=2;
        rear_right *=2;
        }

        if (gamepad1.left_bumper){
        front_left /=2.5;
        front_right /=2.5;
        rear_left /=2.5;
        rear_right /=2.5;
        }

        double intakePower = 0;

        if (gamepad2.a){
        intakePower = -.8;
        }

        if (gamepad2.left_trigger != 0){
        intakePower = .8;
        }


        double liftupPower = gamepad2.left_stick_y;
        double liftoutPower = gamepad2.left_stick_y;

//*******************************************************************
        //Robot Coloration Conditions and Controls
        //***********************************************************
        if (gamepad2.dpad_right){
            patternPrime = RevBlinkinLedDriver.BlinkinPattern.DARK_RED;
        }

        if (gamepad2.dpad_left){
            patternPrime = RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE;
        }
/*
        if (robot.pole.getDistance(DistanceUnit.INCH) > 9 && robot.pole.getDistance(DistanceUnit.INCH) <14) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN;
        }
        else {
            pattern = patternPrime;
        }
*/
//************************************************************************

    robot.blinkinLedDriver.setPattern(patternPrime);

    robot.leftFrontDrive.setPower(front_left);
    robot.rightFrontDrive.setPower(front_right);
    robot.leftRearDrive.setPower(rear_left);
    robot.rightRearDrive.setPower(rear_right);

    robot.liftup.setPower(liftupPower);
    robot.liftout.setPower(liftoutPower);

    robot.Pickupspesaman.setPower(intakePower);

    robot.servoDropper.setPosition(dropperPosition);

    }

}

double scaleInput(double dVal)  {
      double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
            0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };
      
      // get the corresponding index for the scaleInput array.
      int index = (int) (dVal * 16.0);
      if (index < 0) {
         index = -index;
      } else if (index > 16) {
         index = 16;
      }
      
      double dScale = 0.0;
      if (dVal < 0) {
         dScale = -scaleArray[index];
      } else {
         dScale = scaleArray[index];
      }
      
      return dScale;
   }
    
}


