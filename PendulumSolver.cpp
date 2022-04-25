#include <stdio.h>
#include <vector>
#include <iostream>
#include <cmath>
#include "GridCell.hpp"

#define DT 0.01

void inverseStateTransition(double angle, double angular_velocity, double &torque, double new_angle, double new_angular_velocity);
void stateTransition(double angle, double angular_velocity, double torque, double &new_angle, double &new_angular_velocity);
int bellman_update(std::vector<std::vector<GridCell>> &state_mat);
void constrain(int &value, int min, int max);

int main(){

// Magic numbering it up
// But really, I'm deciding to break this state space into 2000 increments.
// We'll see if it's fine enough to reach an optimal solution. 
std::vector <std::vector<GridCell>> state_mat (2000, std::vector<GridCell>(2000));

// The first index is the angle, the second is the angular velocity.   
// So state_mat[0][0] is the state at angle = +pi, angular velocity = -8 rad/s. 
state_mat[0][1000].value = 50;
state_mat[1999][1000].value = 50;

int x = bellman_update(state_mat);


std::cout << state_mat[5][950].value << std::endl;

}


int bellman_update(std::vector<std::vector<GridCell>> &state_mat){

    bool notConverged = true;

    while(notConverged){

        notConverged = false;
        // Assume that the next iteration will result in convergence. If there is a 
        // significant difference between the previous and current values in any cell,
        // then we know we have not reached convergence and set it back to true.

        for (int i = 0; i < 2000; i++){
            for (int j = 0; j < 2000; j++){
                
                // Calculate the optimal torque for the current state.
                double new_angular_velocity;
                double new_angle;
                double final_torque;

                double torque = 1;
                // Doing conversions from index to angle and angular velocity within the function call
                // because I AM LAZY
                stateTransition((i * (M_PI/999.5)) - M_PI, (j * (8/999.5)) - 8, torque, new_angle, new_angular_velocity);

                // Find the grid cell for the new state
                int angle_index_pos_t = (new_angle + M_PI) * (999.5/M_PI);

                int angular_velocity_index_pos_t = (new_angular_velocity + 8) * (999.5/8);
                
                if(angle_index_pos_t == 0 && angular_velocity_index_pos_t == 1000){
                    int sdfsdf = 0;
                }

                constrain(angle_index_pos_t, 0, 1999);
                constrain(angular_velocity_index_pos_t, 0, 1999);

                torque = -1;
                stateTransition((i * (M_PI/999.5)) - M_PI, (j * (8/999.5)) - 8, torque, new_angle, new_angular_velocity);        
                
                // Find the grid cell for the new state
                int angle_index_neg_t = (new_angle + M_PI) * (999.5/M_PI);

                int angular_velocity_index_neg_t = (new_angular_velocity + 8) * (999.5/8);

                if(angle_index_neg_t == 0 && angular_velocity_index_neg_t == 1000){
                    int sdfsdf = 0;
                }

                constrain(angle_index_neg_t, 0, 1999);
                constrain(angular_velocity_index_neg_t, 0, 1999);

                double max_value = 0;
                int max_value_angle_index = 0;
                int max_value_angular_velocity_index = 0;

                for(int k = angle_index_neg_t; k <= angle_index_pos_t; k++){
                    for(int l = angular_velocity_index_neg_t; l <= angular_velocity_index_pos_t; l++){
                        
                        // We know which states we can "reach" from the current state with the minimum and 
                        // maximum torque. So, we make a bounding box that is determined by the states we can reach
                        // and figure out which of those states is the state with the highest value that we are 
                        // also capable of reaching from our current state.
                        
                        if (state_mat[k][l].value > max_value){

                            double final_angular_velocity = (l * (8/999.5)) - 8;
                            double final_angle = (k * (M_PI/999.5)) - M_PI;

                            double angular_velocity = (j * (8/999.5)) - 8;
                            double angle = (i * (M_PI/999.5)) - M_PI;
                            
                            // The angle after the state update is not affected by the torque applied at the 
                            // current timestep, so it is simple to see if the candidate angle state* after 
                            // the state update is a feasible next state.
                            // * there has to be a better way to say that
                            
                            if(final_angle - ((angular_velocity * DT) + angle) < M_PI/2000)
                            {
                                //                      angle                     angular_velocity
                                inverseStateTransition((i * (M_PI/999.5)) - M_PI, (j * (8/999.5)) - 8, torque, final_angle, final_angular_velocity);

                                if(torque > -1 && torque < 1){
                                    final_torque = torque;
                                    max_value = state_mat[k][l].value;
                                    max_value_angle_index = k;
                                    max_value_angular_velocity_index = l;
                                }  

                            }
                        }
                    }
                }

                // We found the maximum value that we are capable of reaching from our current state, so now we 
                // set the value of the current state with a discount factor multiplied by that maximum state.
                double old_value = state_mat[i][j].value;
                
                if(max_value_angle_index != i){
                   
                    state_mat[i][j].value = 0.9 * max_value;
                    state_mat[i][j].optimal_torque = final_torque;
    
                    if(old_value - state_mat[i][j].value > 0.01){
                        notConverged = true;
                    }

                }


            }
        }
    }
    return 1;
}


// Given an input and output angle and angular velocity, calculate the torque it would take to reach that output state.
void inverseStateTransition(double angle, double angular_velocity, double &torque, double new_angle, double new_angular_velocity){
    double dt = DT;
    double friction = 0.3;
    double length = 0.5;
                              
    torque = (new_angular_velocity - (angular_velocity + 9.81 * sin(angle) * dt)/0.5 + dt * friction * angular_velocity)
     * length * length/dt;

    return;
}

void stateTransition(double angle, double angular_velocity, double torque, double &new_angle, double &new_angular_velocity){
    
    double friction = 0.3;
    double dt = DT;
    double length = 0.5;                                
    
    new_angular_velocity = angular_velocity - (9.81 * std::sin(angle) * dt)/0.5 - dt * friction * angular_velocity
     + dt * torque/length/length;

    new_angle = angle + dt * angular_velocity;

    // Angles go from +pi to -pi.
    if(new_angle > M_PI){
        new_angle = new_angle - 2*M_PI;
    }

    if(new_angle < -M_PI){
        new_angle = new_angle + 2*M_PI;
    }

    return;
}

/*
if (n = this.p.angle, o = this.p.angle_dt, this.p.currentTime, 
this.p.angle = this.p.angle + e * o, this.p.angle >= 2 * Math.PI && (this.p.angle -= 2 * Math.PI),
this.p.angle < 0 && (this.p.angle += 2 * Math.PI), 
this.p.angle_dt = this.p.angle_dt - 9.81 * e * Math.sin(n) / this.length - e * this.friction * o + e * this.torque / this.length / this.length, 
this.p.currentTime += e, this.p.simulation_step += 1, !(this.p.currentTime / this.yr > this.p.control_step + 1)) {

friction = 0.3
e = 0.0001
PI = 3.141592653589793
length = 0.5
this.yr = 0.001 < what is that?
*/


void constrain(int &value, int min, int max){
    if(value < min){
        value = min;
    }
    if(value > max){
        value = max;
    }
}
