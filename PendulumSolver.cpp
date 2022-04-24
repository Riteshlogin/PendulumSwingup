#include <stdio.h>
#include <vector>
#include <iostream>
#include <cmath>

#include "GridCell.hpp"

int main(){

// Magic numbering it up
// But really, I'm deciding to break this state space into 2000 increments.
// We'll see if it's fine enough to reach an optimal solution. 
std::vector <std::vector<GridCell>> state_mat (2000, std::vector<GridCell>(2000));

// The first index is the angle, the second is the angular velocity.   
// So state_mat[0][0] is the state at angle = +pi, angular velocity = -8 rad/s. 
state_mat[0][1000].value = 50;
state_mat[1999][1000].value = 50;



std::cout << "Hello World" << std::endl;

}


int bellman_update(std::vector<std::vector<GridCell>> state_mat){

    for (int i = 0; i < 2000; i++){
        for (int j = 0; j < 2000; j++){
            
            // Calculate the optimal torque for the current state.
            double new_angular_velocity;
            double new_angle;
            
            while(posSearchContinues)
            {
                double torque = 1;
                // Doing conversions from index to angle and angular velocity within the function call
                // because I AM LAZY
                stateTransition((i * (M_PI/999.5)) - M_PI, (j * (8/999.5)) - 8, torque, new_angle, new_angular_velocity);

                // Find the grid cell for the new state
                int angle_index_pos_t = (new_angle + M_PI) * (999.5/M_PI);

                int angular_velocity_index_pos_t = (new_angular_velocity + 8) * (999.5/8);
                
                constrain(angle_index_pos_t, 0, 1999);
                constrain(angular_velocity_index_pos_t, 0, 1999);

                torque = -1;
                stateTransition((i * (M_PI/999.5)) - M_PI, (j * (8/999.5)) - 8, torque, new_angle, new_angular_velocity);        
                
                // Find the grid cell for the new state
                int angle_index_neg_t = (new_angle + M_PI) * (999.5/M_PI);

                int angular_velocity_index_neg_t = (new_angular_velocity + 8) * (999.5/8);

                constrain(angle_index_neg_t, 0, 1999);
                constrain(angular_velocity_index_neg_t, 0, 1999);

                double max_value = 0;

                for(int k = angle_index_neg_t; k < angle_index_pos_t; k++){
                    for(int l = angular_velocity_index_neg_t; l < angular_velocity_index_pos_t; l++){
                        
                        // We know which states we can "reach" from the current state with the minimum and 
                        // maximum torque. So, we make a bounding box that is determined by the states we can reach
                        // and figure out which of those states is the state with the highest value that we are 
                        // also capable of reaching from our current state.
                        


                    }
                }

                if(angle_index_pos_t == i && new_angular_velocity == j)
                {
                    if(torque + 0.01 <= 1.0)
                    {
                        // Increment the torque gets us nowhere.
                        break;
                    }
                    torque += 0.01;
                    continue;                    
                }

                // Calculate the value of the new state
            }
        }
    }

    return 1;
}

void inverseStateTransition(double angle, double angular_velocity, double &torque, double new_angle, double new_angular_velocity)
{
    double dt = 0.0001;
    double friction = 0.3;
    double length = 0.5;
                              
    torque = (new_angular_velocity - (angular_velocity + 9.81 * sin(angle) * dt)/0.5 + dt * friction * angular_velocity)
     * length * length/dt;

    return;
}

void stateTransition(double angle, double angular_velocity, double torque, double &new_angle, double &new_angular_velocity){
    
    double friction = 0.3;
    double dt = 0.0001;
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