function cell_width=coverage_width_computation(wall2manipulatorbase_distance,wall_height)
% clc,clear all, close all
%% the setting parameters are:
% wall2mobilebase_distance=1;
% manipulatorbase2mobilebase=0.2;
% wall2manipulatorbase_distance=wall2mobilebase_distance-manipulatorbase2mobilebase;

figure;
%% manipulator parameters are shown as follows:
manipulator_3dworkspace_radius=1.50;
manipulator_minheight=1.2;
manipulator_maxheight=1.6;
%% visualization of manipulator base positions in the global coordinate frame
manipulator_baseposition(1,:)=[0,0,manipulator_minheight];
manipulator_baseposition(2,:)=[0,0,manipulator_maxheight];
scatter3(manipulator_baseposition(:,1),manipulator_baseposition(:,2),manipulator_baseposition(:,3),'k');
hold on;

%% wall parameters are shown as follows:
%% the intersection region between manipulator 3d workspace and wall surface are shown
%% as follows:
%% the circle centriod in the minimum height and maximum height, the circular radius
circle_centroids=zeros(2,3);
circle_centroids(1,1)=wall2manipulatorbase_distance;
circle_centroids(1,2)=0;
circle_centroids(1,3)=manipulator_minheight;
circle_centroids(1,1)=wall2manipulatorbase_distance;
circle_centroids(1,2)=0;
circle_centroids(1,3)=manipulator_maxheight;
circle_radius=sqrt(manipulator_3dworkspace_radius^2-wall2manipulatorbase_distance^2);

%% the intersection region visualization as follows:
t=0:0.1:(2*pi);
n=[t,0];
plot3(wall2manipulatorbase_distance*ones(size(n)), 0+circle_radius*sin(n),manipulator_minheight+circle_radius*cos(n));
hold on;
plot3(wall2manipulatorbase_distance*ones(size(n)), 0+circle_radius*sin(n),manipulator_maxheight+circle_radius*cos(n));
hold on;

%% the intersection points between higher-height circle and wall-ceiling
%% line are: point_a, point_b
costheta=(wall_height-manipulator_maxheight)/circle_radius;
point_a_y=-circle_radius*sqrt(1-costheta^2);
point_b_y=circle_radius*sqrt(1-costheta^2);
%% the intersection points betwen lower-height circle and wall-ground line
costheta=(-manipulator_minheight)/circle_radius;
point_c_y=-circle_radius*sqrt(1-costheta^2);
point_d_y=circle_radius*sqrt(1-costheta^2);
%% the intersection points are recorded as follows:
intersection_points=ones(4,3);
%% point a
intersection_points(1,1)=wall2manipulatorbase_distance;
intersection_points(1,2)=point_a_y;
intersection_points(1,3)=wall_height;
%% point b
intersection_points(2,1)=wall2manipulatorbase_distance;
intersection_points(2,2)=point_b_y;
intersection_points(2,3)=wall_height;
%% point c
intersection_points(3,1)=wall2manipulatorbase_distance;
intersection_points(3,2)=point_c_y;
intersection_points(3,3)=0;
%% point d
intersection_points(4,1)=wall2manipulatorbase_distance;
intersection_points(4,2)=point_d_y;
intersection_points(4,3)=0;
%% plot point a,b,c,d
scatter3(intersection_points(:,1),intersection_points(:,2),intersection_points(:,3),'k');
hold on;
%% the polishing region is defined as follows:
sidelength1=abs(point_b_y-point_a_y);
sidelength2=abs(point_c_y-point_d_y);
region_points=intersection_points;
if sidelength1<sidelength2
    region_points(3,2)=point_a_y;
    region_points(4,2)=point_b_y;
else
    region_points(1,2)=point_c_y;
    region_points(2,2)=point_d_y;
end
region_points(:,2)=region_points(:,2);
%% plot the polishing region as follows:
line1(1,:)=region_points(1,:);
line1(2,:)=region_points(2,:);
plot3(line1(:,1),line1(:,2),line1(:,3),'b','LineWidth',2);
hold on;
line2(1,:)=region_points(2,:);
line2(2,:)=region_points(4,:);
plot3(line2(:,1),line2(:,2),line2(:,3),'b','LineWidth',2);
hold on;
line3(1,:)=region_points(4,:);
line3(2,:)=region_points(3,:);
plot3(line3(:,1),line3(:,2),line3(:,3),'b','LineWidth',2);
hold on;
line4(1,:)=region_points(3,:);
line4(2,:)=region_points(1,:);
plot3(line4(:,1),line4(:,2),line4(:,3),'b','LineWidth',2);
hold off;
axis equal;

%% computate the width of each cell
if sidelength1<sidelength2
    cell_width=sidelength1;
else
    cell_width=sidelength2;
end

end



