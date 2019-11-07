#ifndef MB_STRUCTS_H
#define MB_STRUCTS_H

typedef struct mb_state mb_state_t;
struct mb_state{
    // raw sensor inputs
    float   theta;             // body angle (rad)
    float   phi;               // average wheel angle (rad)
    float   phi_prev;
    float   phi_dot;                
    int     left_encoder;      // left encoder counts since last reading
    int     right_encoder;     // right encoder counts since last reading

    //outputs
    float   left_cmd;  //left wheel command [-1..1]
    float   right_cmd; //right wheel command [-1..1]

    //TODO: Add more variables to this state as needed
    float opti_x;
    float opti_y;
    float opti_roll;
    float opti_pitch;
    float opti_yaw;
    
    float   theta_ref;
    float   theta_dot; 
    

    float   phi_left;          // left wheel angle (rad)
    float   phi_right;         // right wheel angle (rad)
    float   phi_ref;
    float   gamma_prev;
    float   gamma;
    float   gamma_ref;

    float   dist;              //total distance
    float   dist_ref;  
    float   dist_prev;
    /////////////////////////////////////////////////
    float theta_dot_ref_LQR;   // LQR
    float phi_dot_ref_LQR; 
    float theta_ref_LQR;
    float phi_ref_LQR; 
    /////////////////////////////////////////////////
    float theta_LQR;
    float theta_dot_LQR;
    float phi_LQR;
    float phi_dot_LQR;
    /////////////////////////////////////////////////

};

typedef struct mb_setpoints mb_setpoints_t;
struct mb_setpoints{

    float fwd_velocity; // fwd velocity in m/s
    float turn_velocity; // turn velocity in rad/s
    int manual_ctl;
};

typedef struct mb_odometry mb_odometry_t;
struct mb_odometry{

    float x;        //x position from initialization in m
    float y;        //y position from initialization in m
    float psi;      //orientation from initialization in rad

    float psi_ref;
    float psi_prev; 
};

#endif