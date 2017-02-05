function [Ke, C, B] = finite_elements(numb_nodes, elements, coord, ...
    nodes_per_elem, young_modulus, poisson_ratio)
%finite_elements - calculate elements stiffness matrix with linear elastic
%modeling
%
% Syntax:  [Ke, C, B] = finite_elements(numb_nodes, elements, coord,...
%                          nodes_per_elem, young_modulus, poisson_ratio)
%
% Inputs:
%    numb_nodes - number of nodes
%    elements - number of elements
%    coord - matrix with coordinates of all nodes
%    nodes_per_elem - matrix with the nodes connecting each element
%    young_modulus - Young Modulus
%    poisson_ratio - Poisson ratio
%
% Outputs:
%    Ke - metrix with the stiifness matrix for each element
%    C  - elasticity matrix
%    B  - deformable matrix
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

% shell material elasticity constant (YOUNG MODULUS)
E = young_modulus;

% shell material poisson ratio
v = poisson_ratio;

% calculate the elastic matrix
C = E / ((1+v)*(1-2*v))*          [1-v ,  v , v ,  0      ,    0     ,   0      ;
                                   v  , 1-v, v ,  0      ,    0     ,   0      ;
                                   v  ,  v ,1-v,  0      ,    0     ,   0      ;
                                   0  ,  0 , 0 ,(1-2*v)/2,    0     ,   0      ;
                                   0  ,  0 , 0 ,   0     , (1-2*v)/2,   0      ;
                                    0  ,  0 , 0 ,  0      ,    0     ,(1-2*v)/2];

% initialize matrix Ke and B
Ke = zeros(12,12,elements);
B = zeros(6,12,elements);

% calculate the stiffness matrix for each element
for n = 1:elements
    
    V1=zeros(4);
    P=zeros(4);
    Q=zeros(4);
    
    Vol=0;
    
    gNode = coord(nodes_per_elem(n,:),:);
    
    V1=[ 1 1 1 1;
        gNode(1,1) gNode(2,1) gNode(3,1) gNode(4,1);
        gNode(1,2) gNode(2,2) gNode(3,2) gNode(4,2);
        gNode(1,3) gNode(2,3) gNode(3,3) gNode(4,3)];
    
    % volume of each tetrahedron
    Vol=abs((1/6)*det(V1));
    
    P=inv(V1);
    
    Q=(6*Vol)*P;
    
    
    
    B(:,:,n)= (1/(6*Vol))* ...
        [Q(1,2), 0, 0, Q(2,2), 0, 0, Q(3,2), 0, 0, Q(4,2), 0, 0;
        0,  Q(1,3), 0, 0, Q(2,3), 0, 0, Q(3,3), 0, 0, Q(4,3), 0;
        0, 0, Q(1,4), 0, 0, Q(2,4), 0, 0, Q(3,4), 0, 0, Q(4,4);
        Q(1,3),Q(1,2),0,Q(2,3), Q(2,2),0,Q(3,3),Q(3,2),0,Q(4,3),Q(4,2),0;
        0,Q(1,4),Q(1,3),0,Q(2,4),Q(2,3),0,Q(3,4),Q(3,3),0,Q(4,4),Q(4,3);
        Q(1,4),0,Q(1,2),Q(2,4),0,Q(2,2),Q(3,4),0,Q(3,2),Q(4,4),0,Q(4,2)];
    
    
    
    
    
    % calculate the stiffness matrix
    Ke(:,:,n)= Vol *B(:,:,n)' * C* B(:,:,n);
    
end %function



%------------- END OF CODE --------------
