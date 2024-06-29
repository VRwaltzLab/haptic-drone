# 7 motor design
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
