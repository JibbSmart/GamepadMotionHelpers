# GamepadMotionHelpers
GamepadMotionHelpers is a lightweight header-only library for sensor fusion, gyro calibration, etc. BYO input library (eg [SDL2](https://github.com/libsdl-org/SDL)).

## Units
Convert your gyro units into **degrees per second** and accelerometer units to **g-force** (1 g = 9.8 m/s^2). You don't have to use these units in your application, but convert to these units when writing to GamepadMotionHelpers and convert back when reading from it. Your input reader might prefer radians per second and metres per second squared, but the datasheets for every IMU I've seen talk about degrees per second and g-force.

## Coordinate Space
This library uses a Y-up coordinate system. While Z-up is (only slightly) preferable for many games, PlayStation controllers use Y-up, and have set the standard for input libraries like [SDL2](https://github.com/libsdl-org/SDL) and [JSL](https://github.com/JibbSmart/JoyShockLibrary). These libraries convert inputs from other controller types to the same space used by PlayStation's DualShock 4 and DualSense, so that's what's used here.

## Basic Use
Include the GamepadMotion.hpp file in your C++ project. That's it! Everything you need is in that file, and its only dependency is ```<math.h>```.

For each controller with gyro (and optionally accelerometer), create a ```GamepadMotion``` object. At regular intervals, whether when a new report comes in from the controller or when polling the controller's state, you should call ```ProcessMotion(...)```. This is when you tell your GamepadMotion object the latest gyro (in degrees per second) and accelerometer (in g-force) inputs. You'll also give it the time since the last update for this controller (in seconds).

ProcessMotion takes these inputs, updates some internal values, and then you can use any of the following to read its current state:
- ```GetCalibratedGyro(float& x, float& y, float& z)``` - Get the controller's angular velocity in degrees per second. This is just the raw gyro you gave it minus the gyro's bias as determined by your calibration settings (more on that below).
- ```GetGravity(float& x, float& y, float& z)``` - Get the gravity direction in the controller's local space. When the controller is still on a flat surface it'll be approximately (0, -1, 0). The controller can't detect the gravity direction when it's in freefall or being shaken around, but it can make a pretty good guess if its gyro is correctly calibrated and then make further corrections when the controller is still again.
- ```GetProcessedAcceleration(float& x, float& y, float& z)``` - Get the controller's current acceleration in g-force with gravity removed. Raw accelerometer input includes gravity -- it is only (0, 0, 0) when the controller is in freefall. However, using the gravity direction as calculated for GetGravity, it can remove that component and detect how you're shaking the controller about. This function gives you that acceleration vector with the gravity removed.
- ```GetOrientation(float& w, float& x, float& y, float& z)``` - Get the controller's orientation. Gyro and accelerometer input are combined to give a good estimate of the controller's orientation.

## Sensor Fusion
Combining multiple types of sensor like this to get a better picture of the controller's state is called "sensor fusion". Moment-to-moment changes in orientation are detected using the gyro, but that only gives local angular velocity and needs to be correctly calibrated. Errors can accumulate over time. The gravity vector as detected by the accelerometer is used to make corrections to the relevant components of the controller's orientation.

But this cannot be used to correct the controller's orientation around the gravity vector (the **yaw** axis). If you're using the controller's absolute orientation for some reason, this "yaw drift" may need to be accounted for somehow. Some devices also have a magnetometer (compass) to counter yaw drift, but since popular game controllers don't have a magnetometer, I haven't tried it myself. In future, if I get such a device, I'd like to add the option for GamepadMotionHelpers to accept magnetometer input and account for it when calculating values for the above functions.

## Gyro Calibration
Modern gyroscopes often need calibration. This is like how a [weighing scale](https://en.wikipedia.org/wiki/Weighing_scale) can need calibration to tell it what 'zero' is. Like a weighing scale, a correctly calibrated gyroscope will give an accurate reading. If you're using the gyro input as a mouse, which is the simplest application of a controller's gyro, you can find essential reading on [GyroWiki here](http://gyrowiki.jibbsmart.com/blog:good-gyro-controls-part-1:the-gyro-is-a-mouse).

Calibration just means having the controller sit still and remembering the average reported angular velocity in each axis. This is the gyro's "bias". In GamepadMotionHelpers, I call our best guess at the controller's bias the "calibration offset". GamepadMotionHelpers has some options to help with calibrating:

At any time, you can begin manually calibrating a controller by calling ```StartContinuousCalibration()```. This will start recording the average angular velocity and apply it immediately to any subsequent **GetGalibratedGyro(...)** call. At any time you can ```PauseContinuousCalibration()``` to no longer add current values to the average angular velocity being recorded. You can ```ResetContinousCalibration()``` to remove the recorded average before starting over with **StartContinuousCalibration** again.

You can read the stored calibration values using ```GetCalibrationOffset(float& xOffset, float& yOffset, float& zOffset)```. You can manually set the calibration offset yourself with ```SetCalibrationOffset(float xOffset, float yOffset, float zOffset, int weight)```. This will override all stored values. The **weight** argument at the end determines how strongly these values should be considered over time if Continuous Calibration is still active (new values are still being added to the average). Each new sample has a weight of 1, so if you **SetCalibrationOffset** with a weight of 10, it'll have the weight of 10 samples when calculating the average. If you're not continuing to add samples (Continuous Calibration is not active), the weight will be meaningless. Setting this manually is unusual, so don't worry about it too much if that sounds complicated.

Most games don't ask the user to calibrate the gyro themselves. They have built-in automatic calibration, which I like to call "auto-calibration". There's no such thing as a "good enough" auto-calibration solution -- at least not with only gyro and accelerometer. Every game that has an auto-calibration solution would be made better for more serious players with the option to manually calibrate their gyro, so I urge you to provide players the option to do the same in your game. Having said that, auto-calibration is a useful option for casual players, and you may choose to have it enabled in your game by default.

So GamepadMotionHelpers provides some auto-calibration options. You can call ```SetCalibrationMode(CalibrationMode)``` on each GamepadMotion instance with the following options:
- ```CalibrationMode::Manual``` - No auto-calibration. This is the default.
- ```CalibrationMode::Stillness``` - Automatically try to detect when the controller is being held still and update the calibration offset accordingly.
- ```CalibrationMode::SensorFusion``` - Calculate an angular velocity from changes in the gravity direction as detected by the accelerometer. If these are steady enough, use them to make corrections to the calibration offset. This will only apply to relevant axes.

The last two can be combined by passing ```CalibrationMode::Stillness | CalibrationMode::SensorFusion``` to **SetCalibrationMode**. In this case, **SensorFusion** will be applied first, and only the axis or axes unaffected by it will be affected by the **Stillness** calculation.

Many players are already aware of the shortcomings of trying to automatically detect stillness to automatically calibrate the gyro. Whether on Switch, PlayStation, or using PlayStation controllers on PC, players have tried to track a slow or distant target only to have the aimer suddenly stop moving! The game or the platform has misinterpreted their slow and steady input as the controller being held still, and they've incorrectly recalibrated accordingly. Players *hate it* when this happens.

**This is why it's important to let players manually calibrate their gyro** if they want to.

Auto-calibration is used so widely in console games, that it's speculated that game developers may not have the option to disable it on these platforms. If this is the case, GamepadMotionHelpers offers a big advantage over those platforms: you can disable it and enable it at any time.

You, the game developer, can have your game tell if the player is tracking a distant or slow-moving target. You can tell if the player's aimer is moving towards a visible target or roughly following the movement of one. When it is, maybe disabling the auto-calibration (```SetCalibrationMode(CalibrationMode::Manual)```) could be the difference between good and bad auto-calibration. I don't know if the GamepadMotionHelpers auto-calibration functions are better or worse than their Switch and PlayStation counterparts generally, but by letting you take the game's context into account, you may be able to offer players a way better experience without them having to manually calibrate.

But still give them the option to, please :)

The **SensorFusion** calibration mode has shortcomings of its own. It's much harder to accidentally trick the game into incorrectly calibrating, but the angular velocity calculated from the accelerometer is generally much less precise. Leaving the controller still, you'll notice the calibrated gyro moving slightly up and down over time. So while the **Stillness** mode is characterised by good behaviour occasionally punctuated by frustrating errors, the **SensorFusion** mode will tend to be more consistently not-quite-right without being terrible.

Secondly, this library currently only combines accelerometer and gyro, so the **SensorFusion** auto-calibration cannot correct the gyro in all axes at the same time.

## In the Wild
GamepadMotionHelpers is currently used in:
- [JoyShockMapper](https://github.com/Electronicks/JoyShockMapper)
- JoyShockOverlay

If you know of any other games or applications using GamepadMotionHelpers, please let me know!