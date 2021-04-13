#define I2C

#include <ros.h>

#include <std_msgs/Int16MultiArray.h>

#include <Pixy2I2C.h>
Pixy2I2C pixy;

ros::NodeHandle  nh;
std_msgs::Int16MultiArray Pixy_msg;
ros::Publisher pub_Pixy("Pixy_Data", &Pixy_msg);

int Pixy_data[4];

void setup()
{
 //Serial.begin(115200);
  //Serial.print("Starting...\n");
  
//pixy.init(0x1);

nh.initNode();

nh.getHardware()->setBaud(115200);

nh.advertise(pub_Pixy);

//Pixy_msg.data = malloc(sizeof(int)*4);

Pixy_msg.data = Pixy_data;

Pixy_msg.data_length = 4;

}

void loop()
{ 
  int i; 
  int j = 1;
  // grab blocks!
for (j=1; j<6; j++) {

  pixy.init(j);
  pixy.ccc.getBlocks();
  // If there are detect blocks, print them!
  if (pixy.ccc.numBlocks)
  {
    //Serial.print("Detected ");
    //Serial.println(pixy.ccc.numBlocks);
    for (i=0; i<pixy.ccc.numBlocks; i++)
    {
      if (pixy.ccc.blocks[i].m_signature < 38) {
        
        
      
      //Serial.print("  block ");
      //Serial.print(i);
      //Serial.print(", ");
      Pixy_data[0] = (pixy.ccc.blocks[i].m_x);
      //Serial.print(", ");
      Pixy_data[1] = (pixy.ccc.blocks[i].m_y);
      //Serial.print(", ");
      Pixy_data[2] = (pixy.ccc.blocks[i].m_signature);

      if (Pixy_data[2] == 28) {
        Pixy_data[2] = 0; }

      if (Pixy_data[2] == 11) {
        Pixy_data[2] = 1; }
`
      if (Pixy_data[2] == 13) {
        Pixy_data[2] = 2; }

      if (Pixy_data[2] == 37) {
        Pixy_data[2] = 3; }

      if (Pixy_data[2] == 29) {
        Pixy_data[2] = 4; }
   
     Pixy_data[3] = j-1;

    

      pub_Pixy.publish(&Pixy_msg);

      nh.spinOnce();
    }
    }
  } 
}

}
  
  


 
