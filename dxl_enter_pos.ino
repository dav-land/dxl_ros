
#include <DynamixelSDK.h>
#include <ros.h>
#include <std_msgs/Int16.h>


// Control table address MX-64
#define ADDR_PRO_TORQUE_ENABLE          24                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          30
#define ADDR_PRO_PRESENT_POSITION       36

// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel


#define BAUDRATE                        57600
#define DEVICENAME                      "/dev/cu.usbmodem14301"   // Check which port is being used on your controller
// ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE1      100                 // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE1      4000                // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD1     20                  // Dynamixel moving status threshold
#define DXL_MINIMUM_POSITION_VALUE2      775                 // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE2      3250                // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD2     20                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b

void onPanPos(const std_msgs::Int16 &pos);
void onTiltPos(const std_msgs::Int16 &pos);

//ROS integration

ros::NodeHandle nh;
std_msgs::Int16 dyna1_pos;
std_msgs::Int16 dyna2_pos;
ros::Publisher panPos("pan_cur_position", &dyna1_pos);
ros::Publisher tiltPos("tilt_cur_position", &dyna2_pos);
ros::Subscriber<std_msgs::Int16> panGoal("pan_goal_position", &onPanPos);
ros::Subscriber<std_msgs::Int16> tiltGoal("tilt_goal_position", &onTiltPos);

char logBuffer[128];
int pos1 = 2000, pos2 = 2000;

void onPanPos(const std_msgs::Int16 &pos){
  pos1 = pos.data;
}

void onTiltPos(const std_msgs::Int16 &pos){
  pos2 = pos.data;
}

void setup() {

  //ROS Setup
  nh.initNode();
  nh.advertise(panPos);
  nh.advertise(tiltPos);
  nh.subscribe(panGoal);
  nh.subscribe(tiltGoal);

  // put your setup code here, to run once:
  Serial.begin(BAUDRATE);
  while (!Serial);


  nh.loginfo("Start..");

  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  uint8_t dxl_error = 0;                          // Dynamixel error
  int32_t dxl_present_position1 = 0;               // Present position
  int32_t dxl_present_position2 = 0;

  // Open port
  if (portHandler->openPort())
  {
    nh.loginfo("Succeeded to open the port!\n");
  }
  else
  {
    nh.loginfo("Failed to open the port!\n");
    nh.loginfo("Press any key to terminate...\n");
    return;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
  {
    nh.loginfo("Succeeded to change the baudrate!\n");
  }
  else
  {
    nh.loginfo("Failed to change the baudrate!\n");
    nh.loginfo("Press any key to terminate...\n");
    return;
  }

  // Enable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 1, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->getTxRxResult(dxl_comm_result);
  }
  else if (dxl_error != 0)
  {
    packetHandler->getRxPacketError(dxl_error);
  }
  else
  {
    nh.loginfo("Dynamixel 1 has been successfully connected \n");
  }
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 2, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->getTxRxResult(dxl_comm_result);
  }
  else if (dxl_error != 0)
  {
    packetHandler->getRxPacketError(dxl_error);
  }
  else
  {
    nh.loginfo("Dynamixel 2 has been successfully connected \n");
  }


  while (1)
  {
    // Write goal position
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 1, ADDR_PRO_GOAL_POSITION, pos1, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      packetHandler->getTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      packetHandler->getRxPacketError(dxl_error);
    }
    // Write goal position
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 2, ADDR_PRO_GOAL_POSITION, pos2, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      packetHandler->getTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      packetHandler->getRxPacketError(dxl_error);
    }

    do
    {
      // Read present position
      dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, 1, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position1, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        packetHandler->getTxRxResult(dxl_comm_result);
      }
      else if (dxl_error != 0)
      {
        packetHandler->getRxPacketError(dxl_error);
      }

      //          nh.loginfo("[ID:1");
      //          sprintf(logBuffer,"Goal Position: %d",pos1);
      //          nh.loginfo(logBuffer);
      //          sprintf(logBuffer,"Present Position: %d",dxl_present_position1);
      //          nh.loginfo(logBuffer);



    } while ((abs(pos1 - dxl_present_position1) > DXL_MOVING_STATUS_THRESHOLD1));
    do
    {
      // Read present position
      dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, 2, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position2, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        packetHandler->getTxRxResult(dxl_comm_result);
      }
      else if (dxl_error != 0)
      {
        packetHandler->getRxPacketError(dxl_error);
      }

      //         nh.loginfo("[ID:2");
      //          sprintf(logBuffer,"Goal Position: %d",pos2);
      //          nh.loginfo(logBuffer);
      //          sprintf(logBuffer,"Present Position: %d",dxl_present_position2);
      //          nh.loginfo(logBuffer);

    } while ((abs(pos2 - dxl_present_position2) > DXL_MOVING_STATUS_THRESHOLD2));

    //ROS
    dyna1_pos.data = dxl_present_position1;
    dyna2_pos.data = dxl_present_position2;
    panPos.publish(&dyna1_pos);
    tiltPos.publish(&dyna2_pos);
    nh.spinOnce();
  }

  // Disable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 1, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->getTxRxResult(dxl_comm_result);
  }
  else if (dxl_error != 0)
  {
    packetHandler->getRxPacketError(dxl_error);
  }
  // Disable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 2, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->getTxRxResult(dxl_comm_result);
  }
  else if (dxl_error != 0)
  {
    packetHandler->getRxPacketError(dxl_error);
  }

  // Close port
  portHandler->closePort();

}

void loop() {
  // put your main code here, to run repeatedly:

}
