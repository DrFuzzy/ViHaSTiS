function ROS_omni(coord, face, nodes_per_elem, numb_nodes, elements, ...
    young_modulus, poisson_ratio, handles_axes1, handles_axes2, ...
    window, fig, handles_disp, handles_f, handles_p)
% ROS_omni - provides the interface with Phantom Omni haptic device
%
% Syntax:  ros_omni(coord, face, nodes_per_elem, numb_nodes, elements, ...
%      young_modulus, poisson_ratio, handles_axes1, handles_axes2, ...
%      handles_text, fig, window)
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
% rosmatlab_AddClassPath

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
coord_find = coord;
assignin('base','coord_find',coord_find);

% display liver model to figure 1 in Gui
axes(handles_axes1);
trimesh(face,coord(:,1),coord(:,2),coord(:,3));
% tetramesh(nodes_per_elem,coord);
view(-60,60)
camlight;
set(get(handles_axes1, 'XLabel'), 'String', 'X AXIS')
set(get(handles_axes1, 'YLabel'), 'String', 'Y AXIS')
set(get(handles_axes1, 'ZLabel'), 'String', 'Z AXIS')


hold on

% Define global vars
global stop1;
stop1=0;
global R;
global rigid;
global displ_node;
global inv_Kg;
set(gcf, 'Renderer','OpenGL');
H1 = plot3(0,0,0,'m+:');
H3 = plot3(0,0,0,'k*');
axis([-0.25 0.25 -0.25 0.25 -0.25 0.25])
grid on
H2 = title('Tip position (x,y,z): ');
H4 = plot3(0,0,0,'g');
H5 = scatter3(0,0,0,'b*');
H6 = scatter3(0,0,0,'r');


hold off



% rigid point index
j = 1;
index1 = 1;
% finite element method.stiffness matrix of each element calculations
[Ke, C, B]= finite_elements(numb_nodes, elements, coord, ...
    nodes_per_elem, young_modulus, poisson_ratio);

% Refer to Callbacks
pose_sub.setOnNewMessageListeners({@pose_callback});
button_sub.setOnNewMessageListeners({@button_callback});



while (stop1~=1)
    str = input('Press any to release workspace ','s');
    
    if isempty(str)
        break
    end
end

disp('See you!')

%--------------------------------------------------------------------------
% Pose Callback Function
%--------------------------------------------------------------------------
    function pose_callback(msg)
    
    axes(handles_axes2);
    
    currChar = get(fig,'CurrentCharacter');
    if isequal(currChar,char(17))
        
        sprintf('\nDo some tidying up:\n')
        sprintf('Reset force')
        msg_lin.setX(0);
        msg_lin.setY(0);
        msg_lin.setZ(0);
        force_pub.publish(msg_lin);
        node.delete();
        clear('roscore');
        stop1=1;
        assignin('base','stop1',stop1);
    end
    
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
            sprintf('\nTip position (x,y,z): %d %d %d\n',pos.getX(),...
                pos.getY(),pos.getZ())
            sprintf('\nNearest node position (x,y,z): %d %d %d',m(:,1),...
                m(:,2),m(:,3))
            
            % matrix with rigid nodes coordinates
            rigid(j,:)=[m(:,1) m(:,2) m(:,3)];
            j = j+1;
            assignin('base','rigid',rigid);
            
            fid = fopen('data', 'a+');
            fprintf(fid, '%d %d %d %d %d %d %d\n',pos.getX(),pos.getY(),...
                pos.getZ(),ori.getX(),ori.getY(),ori.getZ(),ori.getW());
            fclose(fid);
        end
    end
    
    % with White button select node where the force applies
    % Debounce key press (very simplistic)
    if (data.buttonState(1) ~= last_button_1_state)
        pause(0.17)
        last_button_1_state = 1;
        if (last_button_1_state == 1)
            sprintf('\nTip position (x,y,z): %d %d %d\n',pos.getX(),...
                pos.getY(),pos.getZ())
            sprintf('\nNearest node position (x,y,z): %d %d %d',m(:,1),...
                m(:,2),m(:,3))
            
            % mark rigid point on graph
            set(H5,'XData',rigid(:,1),'YData',rigid(:,2),'ZData',rigid(:,3));
            
            % assembly global stiffness matrix.
            [inv_Kg, R, num_rig] = global_stiffness(numb_nodes,...
                coord, nodes_per_elem, elements, Ke, rigid, coord_find);
            % set to gui static text the numer of fixed nodes
            
            assignin('base','inv_Kg',inv_Kg);
            assignin('base','R',R);
            set(window, 'String', char('No. of rigid points: ',...
                num2str(num_rig), 'Rigid points (x,y,z): ',num2str(rigid)));
            
            % the node which will deform
            displ_node = [m(:,1) m(:,2) m(:,3)]; % Log window!
            assignin('base','displ_node',displ_node);
            set(H6,'XData',displ_node(:,1),'YData',displ_node(:,2),...
                'ZData',displ_node(:,3));
            log_buf3 = [num2str(displ_node(:,1),...
                '%.5f') ', ' num2str(displ_node(:,2),'%.5f') ', '...
                num2str(displ_node(:,3),'%.5f')];
            set(handles_p, 'String', log_buf3);
            
            index1=index1+1;
        end
    end
    
    if (index1==2)
        % the distance between the node which will deform and current point
        d_cont_legend=[pos.getX()-displ_node(:,1)...
            pos.getY()-displ_node(:,2) pos.getZ()-displ_node(:,3)];
        
        log_buf1 = [num2str(d_cont_legend(:,1),'%.5f') ', '...
            num2str(d_cont_legend(:,2),'%.5f') ', '...
            num2str(d_cont_legend(:,3),'%.5f')];
        set(handles_disp, 'String', log_buf1);
        
        pos_dipl_node=[pos.getX() pos.getY() pos.getZ()]; % Log window!
        
        %plot line between the point that user want to move the
        %selected node and the node selected node
        plot_line=[pos.getX() pos.getY() pos.getZ();
            displ_node(:,1) displ_node(:,2) displ_node(:,3)];
        
        set(H4,'XData',plot_line(:,1),'YData',plot_line(:,2),...
            'ZData',plot_line(:,3));
        
        
        % calculate force vector
        [f_contact, k, d_contact] = force(displ_node,coord_find,...
            pos_dipl_node,inv_Kg);
        log_buf2 = [ num2str(f_contact(1),'%.5f') ', ' ...
            num2str(f_contact(2),'%.5f') ', ' num2str(f_contact(3),'%.5f')];
        set(handles_f,'string',log_buf2);
        
        
        % publish the force vector to Omni Phantom
        if max(f_contact) <= 1
            msg_lin.setX(-f_contact(1));
            msg_lin.setY(-f_contact(2));
            msg_lin.setZ(-f_contact(3));
            force_pub.publish(msg_lin);
        end
        
        % calculate the new vertices of deformed liver model
        [new_vertice, e, stress] = new_vertices(inv_Kg, ...
            d_contact,f_contact,k, C, R, elements, B, numb_nodes, ...
            nodes_per_elem, coord) ;
        
        % display deformed liver model to figure 2 in GUI
        trimesh(face,new_vertice(:,1),new_vertice(:,2),new_vertice(:,3));
        axis([-0.25 0.25 -0.25 0.25 -0.25 0.25])
        view(-60,60);
        camlight;
        set(get(handles_axes2, 'XLabel'), 'String', 'X AXIS')
        set(get(handles_axes2, 'YLabel'), 'String', 'Y AXIS')
        set(get(handles_axes2, 'ZLabel'), 'String', 'Z AXIS')
    end
    
    % Set handles to plot
    set(H1,'XData',pos.getX(),'YData',pos.getY(),'ZData',pos.getZ());
    set(H2,'String', ['Tip position (x,y,z): '...
        num2str(pos.getX(),'%.5f') ', ' num2str(pos.getY(),'%.5f') ', '...
        num2str(pos.getZ(),'%.5f')]);
    set(H3,'XData',m(:,1),'YData',m(:,2),'ZData',m(:,3));
    end

%--------------------------------------------------------------------------
% Button Callback Function
%--------------------------------------------------------------------------
    function button_callback(msg)
    button(1) = msg.getWhiteButton();
    button(2) = msg.getGreyButton();
    data.buttonState = button;
    
    % Display info
    %sprintf('\nWhite button: %d \nGrey button: %d', button(1), button(2))
    end

end %function

%------------- END OF CODE --------------
