#### Implemented body rate control in C++

`BodyRateControl` is a P controller with angular accelerations as outputs, which is actuated by torques using Euler's equation of rotation.

#### Implement roll pitch control in C++

`RollPitchControl` is a P controller loosely based on [full quaternion control](http://www.diva-portal.org/smash/get/diva2:1010947/FULLTEXT01.pdf). The only constraint being used is the maximum tilting angle.

A thrust direction objective vector is calculated based on the lateral acceleration objective `accelCmd` and current vertical acceleration induced by motor `accThrust_z`, this vector is converted into drone's body frame. Finally the control output is set to be proportional to `getRotationQuaternion`, which yields the smallest quaternion to rotate from current thrust direction vector to its objective.

(a faster alternative implementation with equivalent result is to use `getRotationQuaternion` directly in world frame and multiply it with current attitude, current implementation is chosen merely for convenience in debugging)

#### Implement altitude controller in C++

`AltitudeControl` is a PID controller with vertical thrust as output, actuated by collective thrust and it's vertical projection (indicated by `R33`)

#### Implement lateral position control in C++

`LateralPositionControl` is a PD controller with horizontal linear acceleration as output.

#### Implement yaw control in C++

`YawControl` is a P controller with yaw angular speed as output.

#### Implement calculating the motor commands given commanded thrust and moments in C++

The ouputs of all 4 motors $\vec F$ is the solution of the following linear equation:

$$
(F_{collective} | \vec \tau)^T = M \vec F^T
$$

where M is the mixing matrix denoting the effect of each motor to all 4 actuation commands (collective thrust, roll/pitch/yaw accelerations).

The solver is made faster by calculating $M^{-1}$ beforehand