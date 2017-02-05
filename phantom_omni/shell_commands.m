%
[status,result] = system('ls')
system('bash')
[status,result] = system('export ROS_IP=10.190.12.40')
[status,result] = system('export ROS_MASTER_URI=http://10.190.12.40:11311')
[status,result] = system('source /opt/ros/indigo/setup.bash')
[status,result] = system('source ~/catkin_ws/devel/setup.bash')
[status,result] = system('rosrun phantom_omni_pkg phantom_omni_pkg_node')

%
cmd=sprintf('/bin/bash --login -c ''echo "$profilevar"; source /opt/ros/indigo/setup.bash; source ~/catkin_ws/devel/setup.bash; rosrun phantom_omni_pkg phantom_omni_pkg_node''');
[r,s]=system(cmd)
disp(s);

%
path_to_script = fullfile(pwd,'rosrun_phantom_omni_pkg.sh');
[status,result] = unix(path_to_script)
