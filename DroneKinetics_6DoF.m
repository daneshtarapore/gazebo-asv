
close all;
clc;
clear

%% Mass matrix

u = 0; v = 0; w = 0; % current velocity m/s
p = 0; q = 0; r = 0; % current angular velocity rad./s    

pos_x = 0; pos_y = 0; pos_z = 10; 
rot_euler_x = 0; rot_euler_y = 0; rot_euler_z = 0;

velocity = [u v w p q r]';
position = [pos_x pos_y pos_z rot_euler_x rot_euler_y rot_euler_z]';

m  =  3; % mass is 1 kg
Ix = 0.0245;
Iy = 0.0788;
Iz = 0.1005;

MRB = [m 0 0 0 0 0; 
    0 m 0 0 0 0; 
    0 0 m 0 0 0;
    0 0 0 Ix 0 0;
    0 0 0 0 Iy 0;
    0 0 0 0 0 Iz];


center_mass_x = -0.04; center_mass_y = 0.0; center_mass_z = 0.067;
center_of_volume_boat_x = -0.04; center_of_volume_boat_y =  0.00; center_of_volume_boat_z =  0.07;
volume_of_boat = 0.006994; %m^3

asv_length = 0.653; %meters
asv_breath = 0.386; %meters
asv_height = 0.100; %meters

density = 1000.0; %kg/m^3

half_hull_cylinder_radius = asv_breath/2;


%% Added mass matrix
% m_ij mass associated with a force on the body in the ith direction due
% to a unit acceleration in the jth direction
Xu1 = -0.05 * m; % Kg -- OR DO ADDED MASS COEFFICIENTS HAVE NO UNITS
Yv1 = -.5 * density * 3.142 * half_hull_cylinder_radius * half_hull_cylinder_radius * asv_length; % Kg
Nr1 = -1/24*(0.1*m*asv_breath*asv_breath + density*3.142*half_hull_cylinder_radius*half_hull_cylinder_radius*asv_length*asv_length*asv_length); % Kg m^2

MA = -[Xu1 0 0 0 0 0; 
    0 Yv1 0 0 0 0; 
    0 0 0 0 0 0;
    0 0 0 0 0 0;
    0 0 0 0 0 0;
    0 0 0 0 0 Nr1]; %% Se Thor's pg37 & 39, look for surface ships


%% Coriolis force for rigid body
CRB = ...
[0	-m*r	m*q	0	0	0;
m*r	0	-m*p	0	0	0;
-m*q	m*p	0	0	0	0;
0	0	0	0	Iz*r	-Iy*q;
0	0	0	-Iz*r	0	Ix*p;
0	0	0	Iy*q	-Ix*p	0];


%% Coriolis force for added mass
CA = ...
[0 0 0 0 0  (Yv1*v); 
0 0 0 0 0 -(Xu1*u);
0 0 0 -(Yv1*v) (Xu1*u) 0;
0 0 (Yv1*v) 0 -(Nr1*r) 0;
0 0 -(Xu1*u) (Nr1*r) 0 0;
-(Yv1*v) (Xu1*u) 0 0 0 0];

%% Linear drag forces
dlc = 10; % Kg / s 
dac = 10; % Kg m^2 / s
DL = ...
[dlc 0 0 0 0 0;
0 dlc 0 0 0 0;
0 0 dlc 0 0 0;
0 0 0 dac 0 0;
0 0 0 0 dac 0;
0 0 0 0 0 dac];


%% Quadractic drag forces -- can be ignored if the boat is moving at low
%% speeds < abs(+/-2m/s)




%% Restoring forces
%% Elaborated in time loop below
     

%% Thrust force -- two propellers with no rotation possible

FL =  12; % Newtons
FR =  0;
DeltaY = 0.0975; % distance of propeller to y coordinate of CoM. Unit: m 
DeltaZ = -0.031; % distance of propeller to z coordinate of CoM. Unit: m; -ve sign added to conform with the direction of torque in Gazebo  

% two propellers with no rotation possible
T = ...
    [(FL + FR); 
    0; 
    0;
    (DeltaZ*FL + DeltaZ*FR);
    0;
    (-DeltaY*FL + DeltaY*FR);]; % left and right propellers are situated on either side of CoM y-aix, and each generates torque along z-axis in opposite direction 

DeltaT = 0.0001; % in seconds
 
fg=figure;
for time = 1:84141
    
    M = MA + MRB;
    
    %% Coriolis and centripetal forces
    CRB = ...
    [0	-m*r	m*q	0	0	0;
    m*r	0	-m*p	0	0	0;
    -m*q	m*p	0	0	0	0;
    0	0	0	0	Iz*r	-Iy*q;
    0	0	0	-Iz*r	0	Ix*p;
    0	0	0	Iy*q	-Ix*p	0];


    %% Coriolis force for added mass
    CA = ...
    [0 0 0 0 0  (Yv1*v); 
    0 0 0 0 0 -(Xu1*u);
    0 0 0 -(Yv1*v) (Xu1*u) 0;
    0 0 (Yv1*v) 0 -(Nr1*r) 0;
    0 0 -(Xu1*u) (Nr1*r) 0 0;
    -(Yv1*v) (Xu1*u) 0 0 0 0];
    
    
    %% Compute restoring forces as function of position of boat
    water_level = 10.0; %m 
    volume_displaced_fluid = 0.0;
    if pos_z > water_level %pos_z is the coordinate of the lower plane of the bounding box
        'Body is completely above water surface'
        volume_displaced_fluid = 0.0;
    elseif ((pos_z + asv_height) < water_level)
        'Body is completely submerged under water'
        boundingbox_depth_underwater = asv_height;
        volume_displaced_fluid = (asv_length * asv_breath * asv_height);
        center_volume_displaced_fluid_x = center_of_volume_boat_x;
        center_volume_displaced_fluid_y = center_of_volume_boat_y;
        center_volume_displaced_fluid_z = center_of_volume_boat_z;
    else
        'Body is partially submerged under water'
        boundingbox_depth_underwater = water_level - pos_z;
        volume_displaced_fluid = asv_length * asv_breath * boundingbox_depth_underwater;
        center_volume_displaced_fluid_x = center_of_volume_boat_x;
        center_volume_displaced_fluid_y = center_of_volume_boat_y;
        center_volume_displaced_fluid_z = boundingbox_depth_underwater/2.0;
    end

    volume_displaced_fluid = volume_displaced_fluid * volume_of_boat / (asv_length * asv_breath * asv_height);
    buoyancy_force_z = -volume_displaced_fluid * density * -9.8;
    bf = ...
     [-buoyancy_force_z * sin(rot_euler_y);
       buoyancy_force_z * cos(rot_euler_y) * sin(rot_euler_x);
       buoyancy_force_z * cos(rot_euler_y) * cos(rot_euler_x); 
       (density * -9.8 * volume_displaced_fluid * ((1.0/12.0)*(asv_breath*asv_breath/boundingbox_depth_underwater) - (center_mass_z - center_volume_displaced_fluid_z))*sin(rot_euler_x));
       (density * -9.8 * volume_displaced_fluid * ((1.0/12.0)*(asv_length*asv_length/boundingbox_depth_underwater) - (center_mass_z - center_volume_displaced_fluid_z))*sin(rot_euler_y))
       0];
     
    gravity_force_z = m * -9.8;
    gf = ...
     [ gravity_force_z * sin(rot_euler_y);
      -gravity_force_z * cos(rot_euler_y) * sin(rot_euler_x);
      -gravity_force_z * cos(rot_euler_y) * cos(rot_euler_x); 
       0;
       0;
       0];
   
     
    if volume_displaced_fluid > 0 
        acceleration = inv(M)*(T - CRB*velocity - (-CA)*velocity - DL*velocity + bf - gf);
        %velocity'      2.4000    0.0000   -0.0000   -0.0002   -0.0000   -0.0000
        %position'      19.4378    0.0006    9.9571   -0.1037    0.0000    0.0000
        
        
        
        %acceleration = inv(M)*(T - CRB*velocity - CA*velocity - DL*velocity + bf - gf);
        %velocity'     2.3952    0.0163   -0.0069   -0.0005    0.0087   -0.1262
        %position'     19.4310   -0.1952    9.9573   -0.1036   -0.0037   -0.1049
        
        
        
        %acceleration = inv(M)*(T - CRB*velocity - DL*velocity + bf - gf);
        %velocity'      2.4000    0.0000   -0.0000   -0.0002         0         0 
        %position'      19.4378   -0.0000    9.9571   -0.1037         0         0
                
        %acceleration = inv(M)*(T - DL*velocity + bf - gf);
        %velocity'      2.4000   -0.0000   -0.0000   -0.0002         0         0
        %position'      19.4378   -0.0008    9.9571   -0.1037         0         0
    else
        acceleration = inv(M)*(-gf);
    end
    
    
    
    velocity = velocity + DeltaT * acceleration;

    sx = sin(rot_euler_x); sy = sin(rot_euler_y); sz = sin(rot_euler_z);
    cx = cos(rot_euler_x); cy = cos(rot_euler_y); cz = cos(rot_euler_z);
    tx = tan(rot_euler_x); ty = tan(rot_euler_y); tz = tan(rot_euler_z);
    
    
    R_Jacobian = ...    
    [
    cy*cz sx*sy*cz-cx*sz cx*sy*cz+sx*sz; 
    cy*sz sx*sy*sz+cx*cz cx*sy*sz-sx*cz; 
    -sy   sx*cy          cx*cy];

    Jr_Jacobian = ...    
    [
    1 sx*ty cx*ty;
    0 cx    -sx;
    0 sx/cy cx/cy];

    
    jacobian = [R_Jacobian  zeros(3,3);
                zeros(3,3)  Jr_Jacobian];    

    position = position + DeltaT * (jacobian*velocity); % make sure to keep the roll angle in range [-pi  to pi] or [0 to 2pi]?
    
    pos_x = position(1); pos_y = position(2); pos_z = position(3); rot_euler_x = position(4); rot_euler_y = position(5); rot_euler_z = position(6);
    u = velocity(1); v = velocity(2); w = velocity(3); p = velocity(4); q = velocity(5); r = velocity(6);
      
    vel_history(time, 1:6) = velocity';
    pos_history(time, 1:6) = position';
    
    velocity'
    
    figure(fg); hold on; 
    quiver(pos_x, pos_y, 0.1*cos(rot_euler_z), 0.1*sin(rot_euler_z))
    %quiver3(pos_x, pos_y, pos_z, rot_euler_x, rot_euler_y, rot_euler_z)
end
