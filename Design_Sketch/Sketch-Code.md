# 7 motor design
My modification of my friends 8 motor design.
If we let i,j, k be the unit vectors and two motors are attached to each of these arms,
And for each i,j,k pairs, they are rotated clockwise and counterclockwise along the j,k,i axeses respectively.
If the pair of motors are 90 degrees about ( 45 degrees rotated around the axis of choice)
```
Force_i = 1/sqrt(2) * ( rotor_i_1 + rotor_i_2 ) ; 
Torque_j = 1/sqrt(2) * rotor_arm_length * ( rotor_i_1 - rotor_i_2 ) ;
```
with a 7th motor sticking out in the -i -j -k direction.
Note: rotor 1 vs rotor 2 is hard to specify right now.
We can easily invert the matrix implied by this set of 3 2 by 2 hadmard matricies.
(It is its own inverse for those unfamiliar with the terms)
The reason we need 7 motors is because in the standard drone controller setting, motors can not be reversed.
Thus, after solving for the 6 rotor values in terms of the 6 target control elements (force,torque *rotor_arm_length),
we need to add a correction factor C = min( all 6 rotors, 0) being added to all main rotors. Then sqrt(6) * C added to the 7th rotor.
Lets put this into code:
```c++
void control(float T_i, float T_j, float T_k ,
             float F_i, float F_j, float F_k,
             float &r_i_1 , float &r_i_2,
             float &r_j_1, float &r_j_2,
             float &r_k_1, float &r_k_2,
             float &r_7) {
const float rotor_arm_length = //need value based on design size
const float one_over_sqrt_two = //need value prestored
const float sqrt_six =//need value prestored
r_i_1 = one_over_sqrt_two* (F_i + rotor_arm_length * T_j );
r_i_2 = one_over_sqrt_two* (F_i - rotor_arm_length * T_j );
r_j_1 = one_over_sqrt_two* (F_j + rotor_arm_length * T_k );
r_j_2 = one_over_sqrt_two* (F_j - rotor_arm_length * T_k );
r_k_1 = one_over_sqrt_two* (F_k + rotor_arm_length * T_i );
r_k_2 = one_over_sqrt_two* (F_k - rotor_arm_length * T_i );
float correction_factor = -1* min( 0, r_i_1, r_i_2, r_j_1, r_j_2, r_k_1, r_k_2) ;
r_7 = sqrt_six * correction_factor;
r_i_1 += correction_factor;
r_i_2 += correction_factor;
r_j_1 += correction_factor;
r_j_2 += correction_factor;
r_k_1 += correction_factor;
r_k_2 += correction_factor;

}
```
# 6 motor design
we have three arms along the i, j, k axis. Each axis has two reversible rotors pointed along the next axis (j, k, i).
Thus letting each axis positionally produces torque along the previous axis and force along the next axis.
The code is very similar to above, but is even simpler (no correction step is needed if all motors can be reversed)
```c++
void control(float T_i, float T_j, float T_k ,
             float F_i, float F_j, float F_k,
             float &r_i_1 , float &r_i_2,
             float &r_j_1, float &r_j_2,
             float &r_k_1, float &r_k_2){
const float rotor_arm_length = //need value based on design size
r_i_1 = F_j + rotor_arm_length * T_k;
r_i_2 = F_j - rotor_arm_length * T_k;
r_j_1 = F_k + rotor_arm_length * T_i;
r_j_2 = F_k - rotor_arm_length * T_i;
r_k_1 = F_i + rotor_arm_length * T_j;
r_k_2 = F_i - rotor_arm_length * T_j;

}
```
# Force at distance
We also have to do a unit conversion from point of contact to center of drone.
This involves lever laws.
Say for the sake of simplicity that the point of contact is some distance(lever length) away on the ith axis of the drone.
Force and torque along this axis are converted directly.
Along the j and k axises, we need to add correction forces and torques in order to only experience torques and forces desired.
Take for example if we just pushed a negative j force to experience a negative j force (resist a positive j force)
The net torque on said object would result in the drone spinning.
```
void convert(float &T_i, float &T_j, float &T_k ,
             float &F_i, float &F_j, float &F_k,
             float desired_T_i, float desired_T_j, float desired_T_k ,
             float desired_F_i, float desired_F_j, float desired_F_k){
float lever_arm_length = // needs number in appropriate unit.
T_i = desired_T_i;
F_i = desired_F_i;
T_j = (desired_T_j + desired_F_k / lever_arm_length ) / 2.0;
T_k = (desired_T_k - desired_F_j / lever_arm_length ) / 2.0 ;
F_j = (desired_F_j + desired_T_k * lever_arm_length ) / 2.0 ;
F_k = (desired_F_k - desired_T_j * lever_arm_length ) / 2.0 ;
}
```
## Force FBD for lever
In cartesian coordinates:
1. Human body exerts downward force F on (0,0) 
2. Drone exerts upward force alpha* F on (r,0)
3. Drone exerts counterclockwise torque beta*F on (r,0)
4. At (0,0) Net forces = -F + alpha * F + Beta * F / r 
5. At (0,0) Net torques = beta * F - r * alpha * F 
#### In conclusion:
Thus beta = r/2 and alpha = 1/2 makes the net forces and torques 0.
## Torque FBD for lever
Again in cartesian coordinates:
1. Human Body exerts clockwise torque T on (0,0)
2. Drone exerts counterclockwise torque alpha * T on (r,0)
3. Drone exerts upwards force beta * T on (r,0)
4. At (0,0) Net forces = beta * T - alpha * T / r
5. At (0,0) Net Torques = -T + alpha * T + r * beta * T
#### In conclusion:
Thus, alpha = 1/2 and beta = 1/(2r) makes the net forces and torques 0.

# Coordinate change:
If it isn't obvious you can change where i,j,k are pointing by appying the appropriate matrix multiplication to torque and force separately.
Note if the singular values aren't 1 then you have also changed the units in some dimension.
