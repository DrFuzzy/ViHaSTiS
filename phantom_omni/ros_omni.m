function ros_omni(coord, face, nodes_per_elem, numb_nodes, elements, young_modulus, poisson_ratio, handles_axes1, handles_axes2, handles_text, fig, window)
%ros_omni - ros_omni is function that fem model interact with haptic device
%Omni Phantom
%device Omni Phantom
%
% Syntax:  ros_omni(coord, face, nodes_per_elem, numb_nodes, elements, young_modulus, poisson_ratio, handles_axes1, handles_axes2, handles_text, fig, window)
% 
%
%
% Inputs:
%    coord - matrix with coordinates of all nodes
%    face - matrix with the nodes with built the surface
%    nodes_per_elem - matrix with the nodes connecting each element
%    numb_nodes - number of nodes
%    elements - number of elements
%    young_modulus - Young Modulus
%    poisson_ratio - Poisson ratio
%    handles_axes1 - the handles of first axis
%    handles_axes2 - the handles of second axis
%    handles_text - the handles static text
%    fig - the handles of gui figure
%    window - the handles of log 
%
%
% Outputs: none
%   
%
%
% Other m-files required: none
% Subfunctions: pose_callback(msg), button_callback(msg)
% MAT-files required: none
%
% 
% Author: Dimitris Dounas
% Work address: none
% email: jdounas1992@gmail.com 
% Website: none
% May 2015; Last revision: none

%------------- BEGIN CODE --------------

% export ROS_IP=10.190.12.40
% export ROS_MASTER_URI=http://10.190.12.40:11311
setenv('ROS_MASTER_URI','http://10.190.12.40:11311')
setenv('ROS_IP','10.190.12.40')
%rosmatlab_AddClassPath

roscore = rosmatlab.roscore(11311);

node = rosmatlab.node('phantom_omni_rosmatlab_node','http://localhost:11311');
% Subscribers
pose_sub  = node.addSubscriber('/tip_pose','geometry_msgs/PoseStamped',1);
button_sub  = node.addSubscriber('/button','phantom_omni_pkg/PhantomButtonEvent',1);
% Publishers
force_pub = node.addPublisher('/set_forces','geometry_msgs/Vector3');

% Set linear velocity components of command
msg_lin = rosmatlab.message('geometry_msgs/Vector3', node);


data = struct('buttonState',[0 0],'tipPos',[0 0 0]);
global coord_find;
coord_find=coord;
assignin('base','coord_find',coord_find);
% display liver model to figure 1 in Gui
axes(handles_axes1);
trimesh(face,coord(:,1),coord(:,2),coord(:,3));
view(-60,60)
camlight;
set(get(handles_axes1, 'XLabel'), 'String', 'X AXIS')
set(get(handles_axes1, 'YLabel'), 'String', 'Y AXIS')
set(get(handles_axes1, 'ZLabel'), 'String', 'Z AXIS')
% fig_h = gcf;

hold on

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Define global vars
global H1;
global H2;
global R;
global rigid;
global dipl_node;
global inv_Kg;
set(gcf, 'Renderer','OpenGL');
H1 = plot3(0,0,0,'m+:');
H3 = plot3(0,0,0,'k*');
% scatter3(pos.getX(),pos.getY(),pos.getZ(),'filled')
axis([-0.25 0.25 -0.25 0.25 -0.25 0.25])
grid on
H2 = title('Tip position (x,y,z): ');
H4 = plot3(0,0,0,'r');

view(-60,60)
hold off

% Assign vars to base workspace
assignin('base','H1',H1);
assignin('base','H2',H2);
assignin('base','H3',H3);
assignin('base','H4',H4);

% rigid point index
j=1;
index1=1;
cnt=1;
% finite element method.stiffness matrix of each element calculations
[Ke, C, B]= finite_elements(numb_nodes, elements, coord, nodes_per_elem, young_modulus, poisson_ratio);

% Refer to Callbacks
pose_sub.setOnNewMessageListeners({@pose_callback});
button_sub.setOnNewMessageListeners({@button_callback});

% Wait Until key press to end program
while (1)
    str = input('Press q followed by enter to quit [q]: ','s');
    if str=='q'
        break
    end
end

sprintf('\nDo some tidying up:\n')
sprintf('Reset force')
msg_lin.setX(0);
msg_lin.setY(0);
msg_lin.setZ(0);
force_pub.publish(msg_lin);
node.delete();
clear('roscore');
%evalin('base',['clear ',H1])

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Pose Callback Function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function pose_callback(msg)
    pos = msg.getPose().getPosition();
    ori = msg.getPose().getOrientation();
    
    data.tipPos = [pos.getX(),pos.getY(),pos.getZ()];
    last_button_2_state = 0;
    last_button_1_state = 0;
    
    
  
  % Find nearest node
    
    x =[face(:,1)
        face(:,2)
        face(:,3)];
    y = unique(x);
    
    node_face = coord(y(:,1),:);
    p = size(node_face,1);
    x2 = ones(p,3);
    
    x2(:,1) = x2(:,1) .*data.tipPos(:,1);
    x2(:,2) = x2(:,2) .*data.tipPos(:,2);
    x2(:,3) = x2(:,3) .*data.tipPos(:,3);
    
    xi = x2(:,1)-node_face(:,1);
    yi = x2(:,2)-node_face(:,2);
    zi = x2(:,3)-node_face(:,3);
   
    squar_dis = ((xi.^2)+(yi.^2)+(zi.^2));
    l = min(squar_dis);
    k = squar_dis==l;
    m = node_face(k,:);
    
 
    
    % with Grey button select rigid points
    % Debounce key press (very simplistic)
    if (data.buttonState(2) ~= last_button_2_state)
        pause(0.3)
       last_button_2_state = 1;
        if (last_button_2_state == 1)
            sprintf('\nTip position (x,y,z): %d %d %d\n',pos.getX(),pos.getY(),pos.getZ())
            sprintf('\nNearest node position (x,y,z): %d %d %d',m(:,1),m(:,2),m(:,3))
            
            %matrix with rigid nodes coordiantes
            
            rigid(j,:)=[m(:,1) m(:,2) m(:,3)];
            j = j+1;
            assignin('base','rigid',rigid);
            fid = fopen('data', 'a+');
            fprintf(fid, '%d %d %d %d %d %d %d\n',pos.getX(),pos.getY(),pos.getZ(),ori.getX(),ori.getY(),ori.getZ(),ori.getW());
            fclose(fid);
        end
    end
    
    
    
    % with White button select node where the force applies
    % Debounce key press (very simplistic)
    if (data.buttonState(1) ~= last_button_1_state)
        pause(0.17)
        last_button_1_state = 1;
        if (last_button_1_state == 1)
            sprintf('\nTip position (x,y,z): %d %d %d\n',pos.getX(),pos.getY(),pos.getZ())
            sprintf('\nNearest node position (x,y,z): %d %d %d',m(:,1),m(:,2),m(:,3))
            
            % assembly global stiffness matrix. 
            [inv_Kg, R, num_rig] = assemb_global_stiffnessm(numb_nodes, coord, nodes_per_elem, elements, Ke, rigid, coord_find);
            % set to gui static text the numer of fixed nodes
            set(handles_text,'string',num_rig);
            
            assignin('base','inv_Kg',inv_Kg);
            assignin('base','R',R);
            
            % the node which will deform
            dipl_node= [m(:,1) m(:,2) m(:,3)]
            assignin('base','dipl_node',dipl_node);
            index1=index1+1;
        end
    end
    
    
  
    if (index1==2)
    
        % the distance between the node which will deform and current point
         d_cont_legend=[pos.getX()-dipl_node(:,1) pos.getY()-dipl_node(:,2) pos.getZ()-dipl_node(:,3)];
         mess1 = sprintf('dx= %f dy=%f dz=%f',d_cont_legend(:,1),d_cont_legend(:,2),d_cont_legend(:,3));        
         set(window, 'String', mess1);
%        log_buf1 = ['Displacement (dx,dy,dz): ' num2str(d_cont_legend(:,1),'%.5f') ', ' num2str(d_cont_legend(:,2),'%.5f') ', ' num2str(d_cont_legend(:,3),'%.5f')];
        
        %%%this link have buttons code http://www.expandinghead.net/keycode.html
        
        %%%char(103)==me to G
        
%           currChar = get(fig,'CurrentCharacter')
%           if isequal(currChar,char(103)) |(cnt==1)  
          %the position that user select to move the deformed node
          
%           cnt=cnt+1;
          
            pos_dipl_node=[pos.getX() pos.getY() pos.getZ()];
            
            
            %plot line between the point that user want to move the
            %selected node and the node selected node
            plot_line=[pos.getX() pos.getY() pos.getZ();
                       dipl_node(:,1) dipl_node(:,2) dipl_node(:,3)];
                                     
            set(H4,'XData',plot_line(:,1),'YData',plot_line(:,2),'ZData',plot_line(:,3));
            
%             fid = fopen('data', 'a+');
%             fprintf(fid, '%d %d %d %d %d %d %d\n',pos.getX(),pos.getY(),pos.getZ(),ori.getX(),ori.getY(),ori.getZ(),ori.getW());
%             fclose(fid);
           
            % calculate force vector
            [f_contact, k, d_contact] = force(dipl_node,coord_find,pos_dipl_node,inv_Kg);
%             log_buf2 = ['F contact (x,y,z): ' num2str(f_contact(1),'%.5f') ', ' num2str(f_contact(2),'%.5f') ', ' num2str(f_contact(3),'%.5f')];
            %log_buf3 = ['d contact (x,y,z): ' num2str(d_contact(1),'%.5f') ', ' num2str(d_contact(2),'%.5f') ', ' num2str(d_contact(3),'%.5f')];
            
%             set(window, 'String', char(log_buf1,log_buf2));
            
            % publish the force vector to Omni Phantom
            
%             msg_lin.setX(f_contact(1));
%             msg_lin.setY(f_contact(2));
%             msg_lin.setZ(f_contact(3));
%             force_pub.publish(msg_lin);
            
            % calculate the new vertices of deformed liver model         

            [new_vertices, e, stress] = displacem_strain_stress(inv_Kg,d_contact,f_contact,k, C, R, elements, B, numb_nodes, nodes_per_elem, coord) ;
            
            % display deformed liver model to figure 2 in Gui
            
            axes(handles_axes2);
            trimesh(face,new_vertices(:,1),new_vertices(:,2),new_vertices(:,3));
            axis([-0.25 0.25 -0.25 0.25 -0.25 0.25])
            view(-60,60);
            camlight;
            set(get(handles_axes2, 'XLabel'), 'String', 'X AXIS')
            set(get(handles_axes2, 'YLabel'), 'String', 'Y AXIS')
            set(get(handles_axes2, 'ZLabel'), 'String', 'Z AXIS')
           
%           end 
    end
    % Detect keypress on figure
    % set(fig_h,'KeyPressFcn',@(fig_obj,eventDat) disp(['You just pressed: ' eventDat.Key])); 

    % Set handles to plot
    set(H1,'XData',pos.getX(),'YData',pos.getY(),'ZData',pos.getZ());
    set(H2,'String', ['Tip position (x,y,z): ' num2str(pos.getX(),'%.5f') ', ' num2str(pos.getY(),'%.5f') ', ' num2str(pos.getZ(),'%.5f')]);
    set(H3,'XData',m(:,1),'YData',m(:,2),'ZData',m(:,3));
    
    % Display info
    %sprintf('\nTip position (x,y,z): %d %d %d \nTip orientation (x,y,z,w): %d %d %d %d', pos.getX(), pos.getY(), pos.getZ(), ori.getX(), ori.getY(), ori.getZ(), ori.getW())
    %sprintf('\nWhite button: %d \nGrey button: %d', data.buttonState(1), data.buttonState(2))
    end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Button Callback Function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function button_callback(msg)
    button(1) = msg.getWhiteButton();
    button(2) = msg.getGreyButton();
    data.buttonState = button;
    
    % Display info
    %sprintf('\nWhite button: %d \nGrey button: %d', button(1), button(2))
    end

end %function

%------------- END OF CODE --------------
