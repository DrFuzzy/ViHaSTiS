function [coord, nodes_per_elem, numb_nodes, elements,...
    face] = read_file(filename)
%triang_to_tetrah - read the obj file and converts the triange elements to
%tetrahedral elements
%
% Syntax:  [coord, nodes_per_elem, numb_nodes,...
%                             elements, face] = triang_to_tetrah(filename)
%
% Inputs:
%    filename - obj file name
%
%
% Outputs:
%    coord - matrix with coordinates of all nodes
%    nodes_per_elem - matrix with the nodes connecting each element
%    numb_nodes - number of nodes
%    elements - number of elements
%    face - matrix with the nodes with built the surface
%
%
% Other m-files required: none
% Subfunctions: read_wobj(), surf2mesh()
% MAT-files required: none
%
%
% Author: Dimitris Dounas
% Work address: none
% email: jdounas1992@gmail.com
% Website: none
% May 2015; Last revision: none

%------------- BEGIN CODE --------------

% read file
OBJ=read_wobj(filename);

% node coordinates
x = OBJ.vertices(:,1);
y = OBJ.vertices(:,2);
z = OBJ.vertices(:,3);
v=[x y z];

% triangles elements
f=OBJ.objects(3).data.vertices;

% convert elements from triangles to tetrahedra
[node,elem,faces]=surf2mesh(v,f,[1 1 1],[100 100 100],0.1,25);

% surface elements
face=[faces(:,1) faces(:,2) faces(:,3)];

% tetrahedra elements
nodes_per_elem=[elem(:,1) elem(:,2) elem(:,3) elem(:,4)] ;

% convert coordinates from cm to m
coord=0.05* node;

% number of nodes
numb_nodes=size(coord,1);

% number of elements
elements=size(nodes_per_elem,1);

end %function

%------------- END OF CODE --------------