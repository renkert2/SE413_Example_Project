function P = calcBodyDims(P)    
    % Proportions: http://www.webcomicalliance.com/wp-content/uploads/2013/10/prop_male.gif
    P.l_1 = ((2+1/3)/8)*P.h; % Upper Leg
    P.l_2 = (2/8)*P.h; % Lower Leg
    P.t = ((2+1/3)/8)*P.h; % Torso
    P.a_1 = ((1+1/2)/8)*P.h; % Upper Arm
    P.a_2 = ((1+1/2)/8)*P.h; % Lower Arm
    P.t_to_ladder = P.l_1; % Distance from torso to ladder, taking as distance of upper leg
    P.COM = ((2/3)/8)*P.t; % Distance from bottom of torso to center of mass;
    
    % Geometry
    theta_l_1_min = asin(P.t_to_ladder / (P.l_1 + P.l_2)); % Leg Extended
    theta_l_1_max = pi/2; % Leg contracted
    
    theta_a_1_max = pi - asin(P.t_to_ladder / (P.a_1 + P.a_2)); % Arm extended
    theta_a_1_min = pi/4; % Arm contracted
    
    theta_l_1 = mean([theta_l_1_min, theta_l_1_max]);
    theta_a_1 = mean([theta_a_1_min, theta_a_1_max]);
    
    theta_l_2 = theta_l_1 - asin((P.t_to_ladder - P.l_1*sin(theta_l_1))/P.l_2);
    theta_a_2 = pi - theta_a_1 - asin((P.t_to_ladder - P.a_1*sin(theta_a_1))/P.a_2);
    
    r_l_1 = P.l_1*[-cos(theta_l_1); -sin(theta_l_1)]; % Hip to Knee
    P.r_l_1 = r_l_1;
    
    r_l_2 = P.l_2 * [-cos(theta_l_1 - theta_l_2); -sin(theta_l_1 - theta_l_2)]; % Knee to Foot
    P.r_l_2 = r_l_2;
    
    r_l = r_l_1 + r_l_2; % Hip to Foot
    P.r_l = r_l;
    
    r_a_1 = P.a_1 * [-cos(theta_a_1); -sin(theta_a_1)]; % Shoulder to Elbow
    P.r_a_1 = r_a_1;
    
    r_a_2 = P.a_2 * [-cos(theta_a_1 + theta_a_2); -sin(theta_a_1 + theta_a_2)]; % Elbow to Hand
    P.r_a_2 = r_a_2;
    
    r_a = r_a_1 + r_a_2; % Shoulder to Hand
    P.r_a = r_a;

    P.r_COM_f = [-P.COM; 0] + r_l; % Vector from COM to Foot
    P.r_COM_h = [P.t-P.COM; 0] + r_a; % Vector from COM to Hand  
    P.r_f_h = -P.r_COM_f + P.r_COM_h; % Vector from Foot to Hand
end
