clc
clear all
close all
setenv("ROS_DOMAIN_ID","33") % 33 is the assigned domain ID 
node1 = ros2node('matlab_node1');  
ros2 node list %view the list of ROS nodes that are visible. 
ros2 msg list %view the list of available message types (templates)
firstMsg = ros2message("example_interfaces/Int32")
firstMsg.data=int32(3.1)
ros2 topic list -t %to find the available topics and their associated message type.
%%

openExample('ros/WorkWithBasicROS2MessagesExample')

scanData = ros2message("sensor_msgs/LaserScan")

clear scanData % To delete the created message.
ros2 msg show geometry_msgs/Twist %To view the definition of the message type.

%%
runTime = 60; % How long do you want the loop to run for in seconds

tic
while(toc <= runTime)
    frontRangeMsg = receive(frontRangeSub);
    leftRangeMsg =  receive(leftRangeSub);
    rightRangeMsg =  receive(rightRangeSub);

    frontDist =  frontRangeMsg.data; % Complete this statement
    leftDist = leftRangeMsg.data; % Complete this statement
    rightDist = rightRangeMsg.data; % Complete this statement

    if frontDist < 0.6 && leftDist < 1
        LeftMotor = 100;
        RightMotor = -100;
    elseif frontDist < 0.6 && rightDist < 1
        LeftMotor = -100;
        RightMotor = 100;
    else
        LeftMotor = 100;
        RightMotor = 100;
    end

    lMotorMsg.data = LeftMotor; % Complete this statement
    rMotorMsg.data = RightMotor; % Complete this statement
    send(lMotorPub, lMotorMsg)
    send(rMotorPub, rMotorMsg)

    pause(0.5)

end