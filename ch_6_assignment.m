clc
clear

%Setting the constants
W1 = 0.109;
W2 = 0.082;
L1 = 0.425;
L2 = 0.392;
H1 = 0.089;
H2 = 0.095;

%Setting the screw axes
B1 = [0 1 0 W1+W2 0 L1+L2]';
B2 = [0 0 1 H2 -L1-L2 0]';
B3 = [0 0 1 H2 -L2 0]';
B4 = [0 0 1 H2 0 0]';
B5 = [0 -1 0 -W2 0 0]';
B6 = [0 0 1 0 0 0]';

bList = [B1 B2 B3 B4 B5 B6];                                    %Creating the blist from screw axes
M = [-1 0 0 0.817; 0 0 1 0.191; 0 1 0 -0.005; 0 0 0 1];         %Creating the home configuration of the robot arm
T = [0 1 0 -0.5; 0 0 -1 0.1; -1 0 0 0.1; 0 0 0 1];              %Desired end-effector configuration
thetaList0 = [-0.1813 2.2036 1.8691 -0.2440 3.1936 2.3016]';    %Creating the initial guess of joint angles
eomg = 0.001;                                                   %Creating the tolerance for angular velocity
ev = 0.0001;                                                    %Creating the tolerance for linear velocity
            
IKinBodyIterates(bList, M, T, thetaList0, eomg, ev)             %Calling the function
