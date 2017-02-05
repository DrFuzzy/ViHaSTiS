function [Inv_Kg, R, num_rig] = global_stiffness(numb_nodes, coord,...
    nodes_per_elem, elements, Ke, rigid, coord_find)
%assemb_global_stiffnessm - calculate global stiffness matrix of soft
%tissue
%
% Syntax: [Inv_Kg, R, num_rig] = assemb_global_stiffnessm(numb_nodes,...
%                   coord, nodes_per_elem, elements, Ke, rigid, coord_find)
% Inputs:
%    numb_nodes - number of nodes
%    coord - matrix with coordinates of all nodes
%    nodes_per_elem - matrix with the nodes connecting each element
%    elements - number of elements
%    R - connectivity matrix
%    Ke - metrix with the stiifness matrix for each element
%    rigid  - coordinates of nodes which are fixed
%    coord_find - matrix with nodes coordinates we used into find function
%
%
% Outputs:
%    Inv_Kg - inverse global stiffness matrix
%    R - connectivity matrix
%    num_rig - number of fixed nodes
%
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
%
%
% Author: Dimitris Dounas
% Work address: none
% email: jdounas1992@gmail.com
% Website: none
% May 2015; Last revision: none

%------------- BEGIN CODE --------------

% Re has nodes coordinates
Re(:,:)= coord(:,:);

% find which nodes is rigid
num_rig=size(rigid,1);
P=zeros(num_rig,1);
M=zeros(num_rig,1);
N=zeros(num_rig,1);

for ym=1:num_rig;
    P(ym)=find(coord_find(:,1)==rigid(ym,1));
    M(ym)=find(coord_find(:,2)==rigid(ym,2));
    N(ym)=find(coord_find(:,3)==rigid(ym,3));
    
    % check if rigid node that user select exist
    if (P(ym)~=M(ym))&(P(ym)~=N(ym))
        disp('error with the point')
    end
end

% replace the rigid nodes with zeros in Re matrix
for zm=1:num_rig;
    Re(P(zm)',:)=[0 0 0];
end

nom = size(Re,2);
% numbers the Degrees of Freedom
item = 0;

for i = 1:numb_nodes;
    for j = 1:nom;
        if Re(i,j) ~= 0;
            item = item +1;
            Re(i,j) = item;
        end
    end
end


% create the connectivity array
for i = 1:elements
    R(i,:) = [Re(nodes_per_elem(i,1),:) Re(nodes_per_elem(i,2),:) ...
        Re(nodes_per_elem(i,3),:) Re(nodes_per_elem(i,4),:)];
end

% calculation of Global stiffness matrix
Kg(item,item)=0;
for x = 1:elements;
    for sat = 1:12;
        for sut = 1:12;
            if (R(x,sat) ~= 0)
                if (R(x,sut) ~= 0);
                    Kg(R(x,sut),R(x,sat)) = Kg(R(x,sut),R(x,sat))...
                        + Ke(sat,sut,x);
                end
            end
        end
    end
end

Inv_Kg=inv(Kg);

end %function

%------------- END OF CODE --------------
