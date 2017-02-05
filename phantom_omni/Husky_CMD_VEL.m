% Ilia Baranov for Clearpath Robotics

function Husky_CMD_VEL
    % Connect to the IP address of the Husky
    node = rosmatlab.node('NODE', '10.25.0.228', 11311);
    
    % Create Subscriber to ROS TWIST Message, called husky/cmd_vel, with
    % buffer size = 1
    subscriber = rosmatlab.subscriber('husky/cmd_vel', 'geometry_msgs/Twist', 1, node);
    
    % When message is received, call function to show value 
    subscriber.setOnNewMessageListeners({@display_vel});

    % Create a publisher for the ROS TWIST message, called husky/cmd_vel
    publisher = rosmatlab.publisher('husky/cmd_vel', 'geometry_msgs/Twist', node);

    % Set Linear velocity components of command
    msgLin = rosmatlab.message('geometry_msgs/Vector3', node);
    msgLin.setX(0);
    msgLin.setY(0);
    msgLin.setZ(0);

    % Set Angular velocity components of command
    msgAng = rosmatlab.message('geometry_msgs/Vector3', node);
    msgAng.setX(0);
    msgAng.setY(0);
    msgAng.setZ(0);

    % Create the message itself from the Linear and Angular Components
    msg = rosmatlab.message('geometry_msgs/Twist', node);
    msg.setLinear (msgLin);
    msg.setAngular (msgAng);

    %Publish to the Husky
    publisher.publish(msg);
    pause(1);

    % If we want the robot to move forwards at 1 meter/second, we use
    msgLin.setX(1)
    publisher.publish(msg);
    pause(1);
    
    % If we want the robot to turn at 0.5 rad/second, we use:
    msgLin.setX(0)
    msgAng.setZ(0.5)
    publisher.publish(msg);
    pause(1);
    
    % Stop the Robot
    msgAng.setZ(0)
    publisher.publish(msg);
    
    % Print out velocity messages
    function display_vel(message)
        Linear_Vel = [message.getLinear.getX() message.getLinear.getY() message.getLinear.getZ()]
        Angular_Vel = [message.getAngular.getX() message.getAngular.getY() message.getAngular.getZ()]
    end


    %Wait Until key press to end program
    str = input('Enter any key to terminate','s');
    
end