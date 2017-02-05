function [new_vertice, e, stress] =new_vertices(Inv_Kg, ...
    d_contact, f_contact, k, C, R,...
    elements, B, numb_nodes, nodes_per_elem, coord)
%displacem_strain_stress - calculate the new coordinates of deformed
%nodes,also calculates strain and stress vector for each element
%modeling
%
% Syntax:  [new_vertices, e, stress] = displacem_strain_stress(Inv_Kg,...
%                             d_contact, f_contact, k, C, R, elements,...
%                             B, numb_nodes, nodes_per_elem, coord)
%
% Inputs:
%    Inv_Kg - inverse global stiffness matrix
%    elements - number of elements
%    d_contact - the diplacement vector
%    f_contact - force vector
%    k - the node which deformed
%    C - elasticity matrix
%    R - connectivity matrix
%    elements - number of elements
%    B  - deformable matrix
%    numb_nodes - number of nodes
%    nodes_per_elem - matrix with the nodes connecting each element
%    coord - matrix with coordinates of all nodes
%
%
% Outputs:
%    new_vertices - the new coordinates for deformed nodes
%    e  - strain matrix for each element
%    stress  - stress matrix for each element
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

%global stiffness matrix size
glob_stiff_size=size(Inv_Kg,1);

% the position of degrees of freedom in global stiffness matrix
c=k*3;
b=c-1;
a=b-1;
ind=1;

%calculate the Kiiab matrix which need to calculate the diplacemt in all
%node

Kiib=[Inv_Kg(:,a) Inv_Kg(:,b) Inv_Kg(:,c)];

Kiiab=[Kiib(1:(a-1),:);
    Kiib((c+1):glob_stiff_size,:)];


%calculate the diplacement in all nodes
d_no_contact= Kiiab *f_contact;

%calculate the diplacements


D=zeros(glob_stiff_size,1);

for i=1:(a-1);
    D(i)=d_no_contact(i);
end

D(a)= d_contact(1);
D(b)= d_contact(2);
D(c)= d_contact(3);

for j=(c+1):glob_stiff_size;
    D(j)=d_no_contact(j-3);
end

% calculate the displacements for each node

for  i = 1:elements;
    for m = 1:12;
        s = R(i, m);
        if s ~=0; Hu(i,m) = D(s) ;
        else      Hu(i,m)  = 0   ;
        end
    end
end

% calculate strain for each element strain=B*diplacements(ux,uy,uz)
e=zeros(6,1,elements);
for i=1:elements;
    e(:,:,i)= B(:,:,i)* Hu(i,:)';
    
end

% calculate stress stress=C*strain
stress=zeros(6,1,elements);

for p=1:elements;
    stress(:,:,p)=C*e(:,:,p);
end

dis=zeros(numb_nodes,3);

% create matric with diplacements for each node with same shape...
% with coordinates matrix

for  i = 1:elements;
    for j=1:4;
        if j==1;
            b=1;
            dis(nodes_per_elem(i,j),:)= [Hu(i,b) Hu(i,b+1) Hu(i,b+2)];
        else if j==2;
                b=4;
                dis(nodes_per_elem(i,j),:)= [Hu(i,b) Hu(i,b+1) Hu(i,b+2)];
            else if j==3;
                 b=7;
                 dis(nodes_per_elem(i,j),:)= [Hu(i,b) Hu(i,b+1) Hu(i,b+2)];
                else
                 b=10;
                 dis(nodes_per_elem(i,j),:)= [Hu(i,b) Hu(i,b+1) Hu(i,b+2)];
                end
            end
        end
    end
end

% calculate the new coordinates for deformed tissue
new_vertice=zeros(numb_nodes,3);
new_vertice(:,:)= coord(:,:) + dis(:,:);


end %function

%------------- END OF CODE --------------
