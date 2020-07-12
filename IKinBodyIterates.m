function IKinBodyIterates(Blist, M, T, thetalist0, eomg, ev)

thetalist = thetalist0;                                                     %Assigning the initial guess to thetalist                         
i = 0;
maxiterations = 20;
Vb = se3ToVec(MatrixLog6(TransInv(FKinBody(M, Blist, thetalist)) * T));     %Calculating the Body-Twist
err = norm(Vb(1: 3)) > eomg || norm(Vb(4: 6)) > ev;                         %Booelan (true or false) error condition
errorAng = norm(Vb(1: 3));                                                  %Angular error
errorLin = norm(Vb(4: 6));                                                  %Linear error
configuration = FKinBody(M, Blist, thetalist);                              %Configuration when the joint angles are equal to initial guess
jointVector(1,:) = thetalist;                                               %First row of the joint angles vector

%Printing starts for the first (0th) iteration:
fprintf('Iteration %d: \n', i);         

fprintf('\n');                                                              %Blank Line for readability
fprintf('Joint Vector: ');
fprintf('%d   ', thetalist(1:length(thetalist)));                           %Printing the values one-by-one
fprintf('\n');

fprintf('\n');                                                              %Blank Line for readability
fprintf('SE(3) end−effector configuration: \n');                        
disp(configuration)                                                         %Since it is a matrix, 'disp' function is used since it is easier to modify

fprintf('error twist V_b: ')    
fprintf('%d   ', Vb(1:length(Vb)));                                         %Printing the values one-by-one
fprintf('\n');                                                              %Blank Line for readability

fprintf('\n');                                                              %Blank Line for readability
fprintf('angular error magnitude ∣∣omega_b∣∣: ');
fprintf('%d   ', errorAng(1:length(errorAng)));                             %Printing the values one-by-one
fprintf('\n');                                                              %Blank Line for readability  

fprintf('\n');                                                              %Blank Line for readability
fprintf('linear error magnitude ∣∣v_b∣∣: ');
fprintf('%d   ', errorLin(1:length(errorLin)));                             %Printing the values one-by-one
fprintf('\n');                                                              %Blank Line for readability
%Printing ends for the first (0th) iteration:

while err && i < maxiterations
    i = i + 1;                                                              %Increasing i by one to move on to the next iteration
    thetalist = thetalist + pinv(JacobianBody(Blist, thetalist)) * Vb;      %Updating thetalist with Newton-Raphson algorithm
    Vb = se3ToVec(MatrixLog6(TransInv(FKinBody(M, Blist, thetalist)) * T)); %Calculating the new Body Twist
    configuration = FKinBody(M, Blist, thetalist);                          %Calculating the new configuration of the End-Effector
    errorAng = norm(Vb(1: 3));                                              %Calculating the new angular error
    errorLin = norm(Vb(4: 6));                                              %Calculating the new linear error
    err = norm(Vb(1: 3)) > eomg || norm(Vb(4: 6)) > ev;                     %calculating the new error boolean expression
    jointVector(i+1,:) = thetalist;                                         %Creating the next row of the jointVector

%Printing the results of the next/ith iteration
fprintf('\n');
fprintf('Iteration %d: \n', i);

fprintf('\n');
fprintf('Joint Vector: ');
fprintf('%d   ', thetalist(1:length(thetalist)));
fprintf('\n');

fprintf('\n');
fprintf('SE(3) end−effector configuration: \n');  
disp(configuration)

fprintf('error twist V_b: ')
fprintf('%d   ', Vb(1:length(Vb)));
fprintf('\n');

fprintf('\n');
fprintf('angular error magnitude ∣∣omega_b∣∣: ');
fprintf('%d   ', errorAng(1:length(errorAng)));
fprintf('\n');

fprintf('\n');
fprintf('linear error magnitude ∣∣v_b∣∣: ');
fprintf('%d   ', errorLin(1:length(errorLin)));
fprintf('\n');
    
end

fprintf('\n');
disp(jointVector)
writematrix(jointVector,'iterates.csv')                                     %Creating a new csv file from the jointVector
end