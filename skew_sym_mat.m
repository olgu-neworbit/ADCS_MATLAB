function [matrix] = skew_sym_mat(vector)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

matrix = [  0   -vector(3)   vector(2);
            vector(3)    0   -vector(1);
           -vector(2)   vector(1)    0 ];
end