#include "ros/ros.h"
#include "beggar_bot/SensorData.h"

#include "../include/constants.h"

#include "../submodules/mcp3008/mcp3008Spi.h"

// analog read from MCP3008 ADC chip
// adc: mcp3008Spi object
// channel: 0..7
unsigned int analogRead(mcp3008Spi &adc, unsigned char channel)
{
  unsigned char spi_data[3];
  unsigned int val = 0;
  
  if (channel > 8) return -1;
  
  // write sequence
  spi_data[0] = 1;  // start bit
  spi_data[1] = 0b10000000 | ( channel << 4); // mode and channel
  spi_data[2] = 0; // anything
  adc.spiWriteRead(spi_data, sizeof(spi_data) );
  
  // read value, combine last two bits of second byte with whole third byte
  val = (spi_data[1]<< 8) & 0b1100000000; 
  val |= (spi_data[2] & 0xff);
  
  return val;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sensor_node");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<beggar_bot::SensorData>("sensor_state", 1);
  ros::Rate loop_rate(100);
  
  float left=0, right=0, left_new=0, right_new=0;
  mcp3008Spi ADC;
  
  beggar_bot::SensorData msg;
  
  ROS_INFO("Sensor node and MCP3008 initialized");
  
  // read initial values to init EWMA
  left = analogRead(ADC, BB_IR_LEFT) / BB_IR_SCALER_LEFT;
  right = analogRead(ADC, BB_IR_RIGHT) / BB_IR_SCALER_RIGHT;
  loop_rate.sleep();

  while (ros::ok())
  {
    // read sensor data and scale each sensor
    left_new = analogRead(ADC, BB_IR_LEFT) / BB_IR_SCALER_LEFT;
    right_new = analogRead(ADC, BB_IR_RIGHT) / BB_IR_SCALER_RIGHT;
    
    // apply exponential moving window
    left = left_new*BB_EWMA_GAMMA + left*(1-BB_EWMA_GAMMA);
    right = right_new*BB_EWMA_GAMMA + right*(1-BB_EWMA_GAMMA);
    
    // make message
    msg.left = left;
    msg.right = right;
    
    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
