% export ROS_IP=10.190.12.40
% export ROS_MASTER_URI=http://10.190.12.40:11311
setenv('ROS_MASTER_URI','http://10.190.12.40:11311')
setenv('ROS_IP','10.190.12.40')
Node = rosmatlab.node('Test','http://10.190.12.40:11311');
Sub  = Node.addSubscriber('/chatter','std_msgs/String',2);
Sub.setOnNewMessageListeners({@test_fn});