#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/Quaternion.h>
#include<geometry_msgs/Point.h>
#include<tf/tf.h>

ros::Publisher pub; 

double normalizeAngle(double angle){
    if (angle > M_PI){
        return(angle - 2*M_PI);
    }
    if (angle < -M_PI){
        return(angle + 2*M_PI);
    }
    return (angle);    
}
    
void Callback(const nav_msgs::OdometryConstPtr &msg){
    double xc, yc, xo = 4, yo = 1.5, x, y, xf = -3, yf = 1.5, d, d2, aux, d_stop;   
    double delta = 1;                  
    double R, Ru; 
    double theta, beta, theta_u, yaw, teta, deltaTeta;            //falta theta e beta
    double v, w;
    geometry_msgs::Quaternion qt;
    geometry_msgs::Point position;
    geometry_msgs::Twist twist_msg; 
    
    qt = msg->pose.pose.orientation;  // pegando posição
    yaw = tf::getYaw(qt);  
    position = msg->pose.pose.position;  //pegando posição x e y
    x = position.x;
    y = position.y;
        
    theta = atan2(yf-yo, xf-xo);
    theta_u = atan2(y-yo, x-xo);
    
    beta = theta - theta_u;
            
    Ru = sqrt(pow(x-xo,2)+pow(y-yo,2));
    
    d = sin(beta)*Ru;    
    R = sqrt(pow(Ru, 2) - pow(Ru*sin(beta), 2));
    d_stop = sqrt(pow(xf-x,2) + pow(yf-y,2));
    xc = (R+delta)*cos(theta) + xo;
    yc = (R+delta)*sin(theta) + yo;   
    ROS_INFO("Xc: %f", xc);
    ROS_INFO("Yc: %f", yc);
    
    //encontrei xc e yc
    
    d2 = sqrt(pow(xc - x, 2) + pow(yc - y, 2));
    
    teta = atan2(yc - y, xc - x);
    deltaTeta = normalizeAngle(teta - yaw);  
    aux = deltaTeta / abs(deltaTeta);
    
     if(d2 > 0.8){     // ainda não chegou
         if(abs(0.9*deltaTeta) > 2.75){
             v = 0.45;
             w = 2.8 * aux;
         }
         else{
        v = 0.75;   
        w = 0.9 * deltaTeta;                        //alinhando o carro com o angulo        
        }
     }
    if(d_stop < 0.2){
        v = 0;
        w = 0;
    }   
    
    twist_msg.linear.x = v;
    twist_msg.angular.z = w; 
    
    pub.publish(twist_msg);  
}

int main(int argc, char **argv){
    
    ros::init(argc, argv, "node_carrot_chasing");
    ros::NodeHandle node;
    
    ros::Subscriber sub = node.subscribe("/vrep/vehicle/odometry", 1, Callback);
    pub = node.advertise<geometry_msgs::Twist>("topic_carrot_chasing", 1);
    
    ros::spin();
    
    return 0;
}