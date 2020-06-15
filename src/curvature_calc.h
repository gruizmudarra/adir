/*
 Node: curvature_calc
 Author: German Ruiz Mudarra, April 2020

 Description:
    When "line_detection_fu_node" detects a certain number of points on a lane of the road, 
    it adjust a polynomial, usually a parabola (ax²+bx+c).
    This node receives the polynomial information and calculates the radius of curvature.
    If the polynomial seems like a straight line, it will have a very low "a" value and the radius of curvature will tend to infinity. 
    If the polynomial begins to have some curvature, then the radius of curvature will have a fixed value.

 Subscriptions:
    /lane_model/poly_degrees (Float32Multiarray): Degree of the polynomials detected
    /lane_model/coef/Left   (Float32Multiarray): [...,c,b,a] coefficients of the polynomials (Usually a parabola with 3 coefficients)
    /lane_model/coef/Center
    /lane_model/coef/Right

Publications:
    /curvature_calc/left    (Float32): Maximum radius of curvature o the polynomial.
    /curvature_calc/center
    /curvature_calc/right
*/

#ifndef CURVATURE_CALC_H
#define CURVATURE_CALC_H

#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"

struct polynomial_t {
    float a;
    float b;
    float c;
    int degree;
    float curvature;
};

using namespace std;

#endif