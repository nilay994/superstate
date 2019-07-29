clear;

BEGIN_ACADO;                                % Always start with "BEGIN_ACADO". 
    
    acadoSet('problemname', 'simplequad');% Set your problemname. If you 
                                            % skip this, all files will
                                            % be named "myAcadoProblem"
    DifferentialState px;
    DifferentialState py;
    DifferentialState vx;
    DifferentialState vy;    
%     DifferentialState L_theta;                    % Dummy state
%     DifferentialState L_phi;                    % Dummy state
    Control theta;
    Control phi;    % Control input
    Parameter T;
    % Disturbance test;
    
    %% Diferential Equation
    f = acado.DifferentialEquation(0, T);       % Set the differential equation object
   
    f.add(dot(px) == vx);                     % Write down your ODE. 
    f.add(dot(py) == vy);      
    f.add(dot(vx) == tan(theta) * 9.81 - 0.5 * vx);       
    f.add(dot(vy) == -tan(phi) * 9.81 / cos(theta) - 0.5 * vy);                   % Dummy equation to integrate used power
%     f.add(dot(L_theta) == theta*theta);
%     f.add(dot(L_phi) == phi*phi);   
    %f.differentialList{1}.toString         % Print an equation to the screen 
    
    
    %% Optimal Control Problem
    ocp = acado.OCP(0.0, T);         % Set up the Optimal Control Problem (OCP)
                                            % Start at 0s, control in 20
                                            % intervals upto 10s
                                            
%     ocp.minimizeMayerTerm(L_phi);               % Minimize the consumed energy
%     ocp.minimizeMayerTerm(L_theta);               % Minimize the consumed energy
    ocp.minimizeMayerTerm(T);
    ocp.subjectTo( f                   );    
    ocp.subjectTo( 'AT_START', px ==  0.0 );   
    ocp.subjectTo( 'AT_START', py ==  0.0 );   
    ocp.subjectTo( 'AT_START', vx ==  0.0 );  
    ocp.subjectTo( 'AT_START', vy ==  0.0 );  

    ocp.subjectTo( 'AT_END', px ==  10.0 );
    ocp.subjectTo( 'AT_END', py ==  10.0 );  
    ocp.subjectTo( 'AT_END', vy ==  0.0 );  

    ocp.subjectTo( -1 <= theta <=  1   );    
    ocp.subjectTo( -1 <= phi <=  1   );     
    ocp.subjectTo(  0.5 <= T <= 10.0   );   
    
    %% Optimization Algorithm
    algo = acado.OptimizationAlgorithm(ocp); % Set up the optimization algorithm, link it to your OCP
    algo.set( 'KKT_TOLERANCE', 1e-6 );      % Set a custom KKT tolerance
    
    
END_ACADO;           % Always end with "END_ACADO".
                     % This will generate a file problemname_ACADO.m. 
                     % Run this file to get your results. You can
                     % run the file problemname_ACADO.m as many
                     % times as you want without having to compile again.

% Run the test
out = simplequad_RUN();

draw;