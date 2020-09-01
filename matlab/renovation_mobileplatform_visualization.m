function renovation_mobileplatform_visualization(renovation_cells_mobileplatform_base_positions,room_plane_edge_cell,renovation_cells_waypioints_onwaypath)

figure;
for i=1:1:size(renovation_cells_mobileplatform_base_positions,2)
    for j=1:1:size(renovation_cells_mobileplatform_base_positions{i},2)
        x0=renovation_cells_mobileplatform_base_positions{i}{j}(:,1);
        y0=renovation_cells_mobileplatform_base_positions{i}{j}(:,2);
        z0=renovation_cells_mobileplatform_base_positions{i}{j}(:,3);
        scatter3(x0,y0,z0,'*');
        hold on;
        axis equal;
    end
end
hold on;
for i=1:1:size(room_plane_edge_cell,2)
    for j=1:1:size(room_plane_edge_cell{i},1)
        xlabel("x axis");
        ylabel("y axis");
        zlabel("z axis");
        title('3D model of interior surfaces framework','FontSize',24);
        x1=[room_plane_edge_cell{i}(j,1),room_plane_edge_cell{i}(j,4)];
        y1=[room_plane_edge_cell{i}(j,2),room_plane_edge_cell{i}(j,5)];
        z1=[room_plane_edge_cell{i}(j,3),room_plane_edge_cell{i}(j,6)];
        plot3(x1 ,y1,z1,'b','LineWidth',1);
        axis equal;
        view(-114,24);
        hold on;
    end
end
hold on;
for i=1:1:size(renovation_cells_waypioints_onwaypath,2)
    for j=1:1:size(renovation_cells_waypioints_onwaypath{i},2)
        for k=1:1:size(renovation_cells_waypioints_onwaypath{i}{j},2)
            for m=1:1:size(renovation_cells_waypioints_onwaypath{i}{j}{k},1)-1
                x1=[renovation_cells_waypioints_onwaypath{i}{j}{k}(m,1),renovation_cells_waypioints_onwaypath{i}{j}{k}(m+1,1)];
                y1=[renovation_cells_waypioints_onwaypath{i}{j}{k}(m,2),renovation_cells_waypioints_onwaypath{i}{j}{k}(m+1,2)];
                z1=[renovation_cells_waypioints_onwaypath{i}{j}{k}(m,3),renovation_cells_waypioints_onwaypath{i}{j}{k}(m+1,3)];
                plot3(x1,y1,z1,'b','LineWidth',1);
                hold on;
            end
            axis equal;
        end
    end
end
% hold on;
% for i=1:1:size(renovation_cells_mobileplatform_base_positions,2)
%     for j=1:1:size(renovation_cells_mobileplatform_base_positions{i},2)
%         x0=renovation_cells_mobileplatform_base_positions1{i}{j}(:,1);
%         y0=renovation_cells_mobileplatform_base_positions1{i}{j}(:,2);
%         z0=renovation_cells_mobileplatform_base_positions1{i}{j}(:,3);
%         scatter3(x0,y0,z0,'*');
%         hold on;
%         axis equal;
%     end
% end


hold off;

end


