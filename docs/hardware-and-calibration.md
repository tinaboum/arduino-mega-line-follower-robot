# Hardware and calibration guide

This guide separates the verified project architecture from values that must be tuned on the physical robot.

## Documented hardware

| Component | Engineering role |
| --- | --- |
| Arduino Mega 2560 | Sensor acquisition, behavior logic and motor commands |
| Five-channel digital IR array | Detects the track and estimates lateral error |
| HC-SR04 ultrasonic sensor | Measures forward and side clearance |
| Scanning servo | Rotates the ultrasonic sensor for left/right comparison |
| TCS34725 color sensor | Detects colored checkpoints through I2C |
| L298N dual H-bridge | Controls direction and PWM drive for two DC motors |
| Two geared DC motors | Differential-drive actuation |
| Wheeled chassis | Carries the embedded and electromechanical system |

The available documentation does not establish the battery chemistry, nominal motor voltage, servo model, wheel diameter, chassis mass, or electrical current measurements. Those details are intentionally not inferred here.

## Electrical checks before operation

1. Confirm that the motor supply voltage is compatible with the motors and L298N module.
2. Do not power the motors from the Arduino 5 V rail.
3. Connect the Arduino, sensor and motor-driver grounds together.
4. Confirm that the L298N enable jumpers are removed when pins 6 and 11 provide PWM.
5. Check the polarity of both motor channels with the chassis lifted from the surface.
6. Confirm that the HC-SR04 echo signal, servo supply and TCS34725 module are electrically compatible with the controller and power arrangement.
7. Verify SDA on pin 20 and SCL on pin 21 for the Arduino Mega 2560.

## Commissioning sequence

### 1. Motor direction

Command both motors at low positive PWM with the wheels clear of the surface. If one wheel rotates backwards, correct its motor wiring or direction mapping before continuing.

### 2. IR sensor polarity and order

Observe each digital output over the track and background. The firmware assumes `LOW` means that a sensor sees the line. Change `LINE_DETECTED_STATE` if the hardware is inverted. Confirm that the array is ordered from the robot's left side to its right side.

### 3. Base motion

Begin with a conservative `BASE_LINE_SPEED`. Verify straight motion before tuning steering. Motor mismatch may require separate left and right base offsets in a future revision.

### 4. PD steering

Tune on the real track:

1. Set `LINE_KD` to zero and increase `LINE_KP` until the robot follows the line but begins to oscillate.
2. Reduce `LINE_KP` slightly.
3. Increase `LINE_KD` until oscillation and overshoot improve without making the response noisy.
4. Repeat at the intended operating speed and battery condition.

The controller uses a discrete error change rather than a time-normalized derivative. Keep the loop behavior consistent during tuning.

### 5. Ultrasonic and servo geometry

Measure the true center, left and right viewing directions. Update the three servo angles if the mechanical mounting is reversed. Test the distance threshold against obstacles with representative surface materials and angles.

### 6. Timed bypass

Tune the quarter-turn, side-step, pass-obstacle and line-search durations at low speed. Timed motion is open-loop and must be retested whenever the battery, surface, wheels, motor loading or PWM values change.

### 7. Color checkpoints

Record raw red, green, blue and clear-channel readings for each marker under the intended lighting. Adjust `MINIMUM_CLEAR_READING` and the dominance test only from measured samples. Avoid claiming a checkpoint class until it has been verified across the expected lighting range.

## Known limitations

- No wheel encoder or heading feedback is available.
- Obstacle bypass uses elapsed time rather than measured displacement or angle.
- A single ultrasonic range measurement does not provide a complete obstacle map.
- The digital IR array provides coarse line-position information.
- Raw RGB dominance is sensitive to illumination, mounting height and surface reflectance.
- Blocking delays pause other sensing while the robot scans, turns or waits at a checkpoint.
- No quantitative repeatability dataset was supplied with the recovered project files.

## Recommended test record

For a future hardware revision, record:

- track layout and surface;
- battery voltage at the start and end of each run;
- configured PWM, PD gains and obstacle threshold;
- successful laps and failures;
- line-loss and recovery events;
- measured obstacle clearance;
- checkpoint classification results; and
- firmware commit identifier.

This converts prototype demonstrations into reproducible engineering validation.
