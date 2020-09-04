[stl_facet,stl_vertices,stl_norm_vector]=stl_read("20200716_scan_planning_1.stl");
z_min=min(stl_vertices(:,3));
for i=1:1:size(stl_vertices,1)
    stl_vertices1(i,3)=stl_vertices(i,3)-z_min;
end
stl_facet1=stl_facet;
stl_write("/Users/zhouyang/Desktop/github_packages/polishingrobot_planner/mesh/20200716_scan_planning_2.stl",stl_facet1,stl_vertices1)

[stl_facet2,stl_vertices2,stl_norm_vecto2]=stl_read("20200716_scan_planning_2.stl");
