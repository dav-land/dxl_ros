#include <DynamixelSDK.h>
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <dynamixel_msg/PanTiltInternalControl.h>
#include <dynamixel_msg/PanTiltInternalStatus.h>


// Control table address MX-64
#define ADDR_PRO_TORQUE_ENABLE          24                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          30
#define ADDR_PRO_PRESENT_POSITION       36
#define ADDR_PRO_MOVING_SPEED           32
#define ADDR_PRO_PRESENT_SPEED          38

// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel


#define BAUDRATE                        1000000
#define DEVICENAME                      "1"   // Check which port is being used on your controller
// ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE1      100                 // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE1      4000                // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD1     20                  // Dynamixel moving status threshold
#define DXL_MINIMUM_POSITION_VALUE2      775                 // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE2      3250                // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD2     5                  // Dynamixel moving status threshold
#define RESET_PIN                        A0

#define ESC_ASCII_VALUE                 0x1b

void onPanTiltInternalControl(const dynamixel_msg::PanTiltInternalControl &curCtl);
void onPanTiltReset(const std_msgs::Bool &rst);

//ROS integration

ros::NodeHandle nh;

dynamixel_msg::PanTiltInternalStatus curStat;

ros::Publisher PanTiltInternalStatus("pan_tilt_internal_status", &curStat);
ros::Subscriber<dynamixel_msg::PanTiltInternalControl> PanTiltInternalControl("pan_tilt_internal_control", &onPanTiltInternalControl, 1);
ros::Subscriber<std_msgs::Bool> PanTiltReset("pan_tilt_reset", &onPanTiltReset, 1);


char logBuffer[128];
int pos1 = 2000, pos2 = 2000, speed1 = 0, speed2 = 0, prevPos1 = -1, prevPos2 = -1, prevSpeed1 = -1, prevSpeed2 = -1;
bool torque1 = true, torque2 = true, prevTorque1 = true, prevTorque2 = true, publish = false, reset = false;


void onPanTiltInternalControl(const dynamixel_msg::PanTiltInternalControl &curCtl) {
  pos1 = curCtl.pan_position;
  if (pos1 > 4095)
    pos1 = 4095;
  if (pos1 < 0)
    pos1 = 0;

  pos2 = curCtl.tilt_position;
  if (pos2 > 3250)
    pos2 = 3250;
  if (pos2 < 750)
    pos2 = 750;

  speed1 = curCtl.pan_max_speed;
  if (speed1 < 0)
    speed1 = 0;
  if (speed1 > 1023)
    speed1 = 1023;

  speed2 = curCtl.tilt_max_speed;
  if (speed2 < 0)
    speed2 = 0;
  if (speed2 > 1023)
    speed2 = 1023;

  torque1 = curCtl.pan_torque;
  torque2 = curCtl.tilt_torque;

}

void onPanTiltReset(const std_msgs::Bool &rst) {
  reset = rst.data;
}

void setup() {

  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, HIGH);

  //ROS Setup
  nh.initNode();

  nh.advertise(PanTiltInternalStatus);
  nh.subscribe(PanTiltInternalControl);
  nh.subscribe(PanTiltReset);

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
  int16_t dxl_present_position1 = 0;               // Present position
  int16_t dxl_present_position2 = 0;
  int16_t dxl_present_speed1 = 0;
  int16_t dxl_present_speed2 = 0;

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
  packetHandler->write1ByteTxRx(portHandler, 1, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  packetHandler->write1ByteTxRx(portHandler, 2, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);

  while (1)
  {

    if (reset) {
      nh.loginfo("Setting pin to low");
      digitalWrite(RESET_PIN, LOW);
    }

    if (prevTorque1 != torque1 && torque1) {
      // Enable pan Torque
      packetHandler->write1ByteTxRx(portHandler, 1, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
      prevTorque1 = torque1;
    }

    if (prevTorque1 != torque1 && !torque1) {
      // Disable pan Torque
      packetHandler->write1ByteTxRx(portHandler, 1, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
      prevTorque1 = torque1;
    }

    if (prevTorque2 != torque2 && torque2) {
      //Enable tilt torque
      packetHandler->write1ByteTxRx(portHandler, 2, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
      prevTorque2 = torque2;
    }

    if (prevTorque2 != torque2 && !torque2) {
      // Disable tilt Torque
      packetHandler->write1ByteTxRx(portHandler, 2, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
      prevTorque2 = torque2;
    }

    //Writes new goal pan position if a change is detected
    if (prevPos1 != pos1) {
      // Write goal pan position
      packetHandler->write2ByteTxRx(portHandler, 1, ADDR_PRO_GOAL_POSITION, pos1, &dxl_error);
      prevPos1 = pos1;
    }

    //Write new goal tilt position if a change is detected
    if (prevPos2 != pos2) {
      // Write goal tilt position
      packetHandler->write2ByteTxRx(portHandler, 2, ADDR_PRO_GOAL_POSITION, pos2, &dxl_error);
      prevPos2 = pos2;
    }

    if (prevSpeed1 != speed1) {
      // Write goal speed
      packetHandler->write2ByteTxRx(portHandler, 1, ADDR_PRO_MOVING_SPEED, speed1, &dxl_error);

      prevSpeed1 = speed1;
    }

    if (prevSpeed2 != speed2) {
      // Write goal speed
      packetHandler->write2ByteTxRx(portHandler, 2, ADDR_PRO_MOVING_SPEED, speed2, &dxl_error);
      prevSpeed2 = speed2;
    }

    //Read present Position
    packetHandler->read2ByteTxRx(portHandler, 1, ADDR_PRO_PRESENT_POSITION, (uint16_t*)&dxl_present_position1, &dxl_error);
    packetHandler->read2ByteTxRx(portHandler, 2, ADDR_PRO_PRESENT_POSITION, (uint16_t*)&dxl_present_position2, &dxl_error);
    // Read present speed 1
    packetHandler->read2ByteTxRx(portHandler, 1, ADDR_PRO_PRESENT_SPEED, (uint16_t*)&dxl_present_speed1, &dxl_error);
    // Read present speed 2
    packetHandler->read2ByteTxRx(portHandler, 2, ADDR_PRO_PRESENT_SPEED, (uint16_t*)&dxl_present_speed2, &dxl_error);

    //ROS
    curStat.header.stamp = nh.now();
    curStat.pan_position  = dxl_present_position1;
    curStat.pan_max_speed     = dxl_present_speed1;
    curStat.pan_torque    = torque1;
    curStat.tilt_position = dxl_present_position2;
    curStat.tilt_max_speed    = dxl_present_speed2;
    curStat.tilt_torque   = torque2;

    PanTiltInternalStatus.publish(&curStat);
    nh.spinOnce();
    delay(15);
    nh.spinOnce();
  }

  // Disable Dynamixel Torque
  packetHandler->write1ByteTxRx(portHandler, 1, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);

  // Disable Dynamixel Torque
  packetHandler->write1ByteTxRx(portHandler, 2, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);

  // Close port
  portHandler->closePort();

}

void loop() {
  // put your main code here, to run repeatedly:

}
