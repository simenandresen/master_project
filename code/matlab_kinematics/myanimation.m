%-----------------------------------
%
%  animation file
%  all animation called from here
%
%-----------------------------------

close all;
%% initialize

% vehicle
rov_vertices_init = [0 -1 0; 0 1 0; -2.3 1 0; -2.3 -1 0;       0 -1 1; 0 1 1; -2.3 1 1; -2.3 -1 1];
my_faces = [1 2 3 4; 2 6 7 3; 4 3 7 8; 1 5 8 4; 1 2 6 5; 5 6 7 8];
% obstacle
v1=[4,-2,0.6; 5, -2,0.1;5,2,0.1; 4,2,0.6 ];
v2=[5,-2,-0.1; 5,-2,0.1; 5,2,0.1; 5,2,-0.1];
v3=[4,-2,-0.6; 5, -2,-0.1;5,2,-0.1; 4,2,-0.6 ];
obs_verts=[v1;v2;v3];


ee_traj=[];
ee_traj_c=[];
f=1;
h(f)= figure(f);
set(h(f),'Units','normalized');
set(h(f),'Position',[0.1,0.3,0.5,0.4]);
xlabel('x');
ylabel('y');
zlabel('z');
m_i=1;
%obspatch=patch('Vertices', obs_verts, 'Faces', [1,2,3,4;5,6,7,8 ; 9,10,11,12], 'FaceColor', 'y');
%set(obspatch,'facealpha',0.4);
hold on;

for i=1:20:length(eta1(:,1))
    hom_mat=homogenous_mat(eta1(i,:),eta2(i,:));
    new_mat=homtrans(hom_mat,rov_vertices_init');
    six_link.plot(q(i,:),'nobase','noshadow','noname','nojaxes','nojoints');
    %hold on;
    six_link.base=hom_mat;
    if i>1
        pause(0.1);
        delete(rov_figure);
    end
    rov_figure=patch('Vertices', new_mat', 'Faces', my_faces, 'FaceColor', 'y');
    %set(rov_figure,'facealpha',0.5);
    %axis([-7, 7, -7, 7 , -7, 7]);    
    
    ee_traj(end+1,:)=ee_eta1(i,:);
    %plot3(ee_traj(:,1),ee_traj(:,2),ee_traj(:,3), '-m');
    
    %ee_traj_c(end+1,:)=ee_eta1_c(i,:);
    plot3(ee_eta1_c(1:20:end,1),ee_eta1_c(1:20:end,2),ee_eta1_c(1:20:end,3), '-r', 'LineWidth',3);
    
    my_movie(m_i)=getframe;
    m_i=m_i+1;
    axis([-3,3.6, -2,2,-0.9,1.2]);
    view([0,0]);
    print(h(1),'-depsc2',sprintf('/home/simena/Dropbox/master_project/report/figures/matlab/kin-robot%i.eps',i));
    if i>3700
        break;
    end
end

%print(h(1),'-depsc2','/home/simena/Dropbox/master_project/report/figures/matlab/untitled.eps');
%print(h(1),'-djpeg','/home/simena/Dropbox/master_project/report/figures/matlab/untitled3.jpeg');

%%
% close all;
% f=2;
% h(f)= figure(f);
% set(h(f),'Units','normalized');
% set(h(f),'Position',[0.1,0.3,0.4,0.3]);
% 
% movie(my_movie,1,1);
% hold off;
% 
% 
% %%
% close all;
% rov_vertices_init = [0 -1 0; 0 1 0; -2.3 1 0; -2.3 -1 0;       0 -1 1; 0 1 1; -2.3 1 1; -2.3 -1 1];
% my_faces = [1 2 3 4; 2 6 7 3; 4 3 7 8; 1 5 8 4; 1 2 6 5; 5 6 7 8];
%     hom_mat=homogenous_mat([0,0,0],[0,0,0]);
%     new_mat=homtrans(hom_mat,rov_vertices_init');
%     six_link.plot([0,0,0,0,0,0]);
%     hold on;
%     six_link.base=hom_mat;
%     
%     h1=patch('Vertices', new_mat', 'Faces', my_faces, 'FaceColor', 'g');
%     axis([-4, 4, -4, 4 , -4, 4]);    
%     
%           
%     
%     hold off;
%     
%     
% %%
% 
% v1=[4,-2,0.6; 5, -2,0.1;5,2,0.1; 4,2,0.6 ];
% v2=[5,-2,-0.1; 5,-2,0.1; 5,2,0.1; 5,2,-0.1];
% v3=[4,-2,-0.6; 5, -2,-0.1;5,2,-0.1; 4,2,-0.6 ];
% 
% obs_verts=[v1;v2;v3];
% 
% obspatch=patch('Vertices', obs_verts, 'Faces', [1,2,3,4;5,6,7,8 ; 9,10,11,12], 'FaceColor', 'y');
% xlabel('x'); ylabel('y'); zlabel('z')
% ;
% grid on;
% set(obspatch,'facealpha',0.4);
%    


















