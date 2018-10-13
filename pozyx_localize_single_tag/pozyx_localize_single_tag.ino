// Please read the ready-to-localize tuturial together with this example.
// https://www.pozyx.io/Documentation/Tutorials/ready_to_localize
/**
  The Pozyx ready to localize tutorial (c) Pozyx Labs

  Please read the tutorial that accompanies this sketch: https://www.pozyx.io/Documentation/Tutorials/ready_to_localize/Arduino

  This tutorial requires at least the contents of the Pozyx Ready to Localize kit. It demonstrates the positioning capabilities
  of the Pozyx device both locally and remotely. Follow the steps to correctly set up your environment in the link, change the
  parameters and upload this sketch. Watch the coordinates change as you move your device around!
*/
#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>

////////////////////////////////////////////////
////////////////// PARAMETERS //////////////////
////////////////////////////////////////////////

uint16_t remote_id = 0x6000;                            // set this to the ID of the remote device
bool remote = false;                                    // set this to true to use the remote ID

boolean use_processing = true;                         // set this to true to output data for the processing sketch

const uint8_t num_anchors = 4;                                    // the number of anchors
uint16_t anchors[num_anchors] = {0x6841, 0x6E11, 0x6129, 0x6059};     // the network id of the anchors: change these to the network ids of your anchors.
//uint16_t anchors[num_anchors] = {0x6E11, 0x6841, 0x6129, 0x6059};     // for automatic
int32_t anchors_x[num_anchors] = {6300, 0, -300, 5400};               // anchor x-coorindates in mm
int32_t anchors_y[num_anchors] = {0, 0, 7670, 7770};                  // anchor y-coordinates in mm
int32_t heights[num_anchors] = {2380, 2380, 2320, 2130};              // anchor z-coordinates in mm

uint8_t algorithm = POZYX_POS_ALG_TRACKING;             // positioning algorithm to use. try POZYX_POS_ALG_TRACKING for fast moving objects.
uint8_t dimension = 1;                           // positioning dimension
int32_t height = 1000;                                  // height of device, required in 2.5D positioning

uint8_t filter_type = FILTER_TYPE_MOVINGMEDIAN; // types = FILTER_TYPE_NONE, FILTER_TYPE_FIR, FILTER_TYPE_MOVINGAVERAGE, FILTER_TYPE_MOVINGMEDIAN
uint8_t filter_strength = 5; // 0 - 15 range

uint16_t interval_ms = 10;

////////////////////////////////////////////////

void setup(){
  Serial.begin(115200);

  if(Pozyx.begin() == POZYX_FAILURE){
    Serial.println(F("ERROR: Unable to connect to POZYX shield"));
    Serial.println(F("Reset required"));
    delay(100);
    abort();
  }

  if(!remote){
    remote_id = NULL;
  }

//  Serial.println(F("----------POZYX POSITIONING V1.1----------"));
//  Serial.println(F("NOTES:"));
//  Serial.println(F("- No parameters required."));
//  Serial.println();
//  Serial.println(F("- System will auto start anchor configuration"));
//  Serial.println();
//  Serial.println(F("- System will auto start positioning"));
//  Serial.println(F("----------POZYX POSITIONING V1.1----------"));
//  Serial.println();
//  Serial.println(F("Performing manual anchor configuration:"));

  // clear all previous devices in the device list
  Pozyx.clearDevices(remote_id);
  // sets the anchor manually
//  Pozyx.doAnchorCalibration(POZYX_2_5D, 5, 4, anchors, heights); // set anchors automatically
  setAnchorsManual();
  
  // sets the positioning algorithm
  Pozyx.setPositionAlgorithm(algorithm, dimension, remote_id);

  // set update rate of the pozyx device
  Pozyx.regWrite(POZYX_POS_INTERVAL, (uint8_t*)&interval_ms , 2);
  
  int s = Pozyx.setPositionFilter(filter_type, filter_strength, remote_id);
  
  printCalibrationResult();
  delay(2000);

  Serial.println(F("Starting positioning: "));
}

void loop(){
  coordinates_t position;
  sensor_raw_t sensor_raw;
  int status;
  if(remote){
    status = Pozyx.doRemotePositioning(remote_id, &position, dimension, height, algorithm);
  }else{
    status = Pozyx.doPositioning(&position, dimension, height, algorithm);
    Pozyx.getRawSensorData(&sensor_raw);
  }

  while(!Serial.available()){
  }
  char c = Serial.read();
  if (status == POZYX_SUCCESS && c == 'r'){
    // prints out the result
    printCoordinates(position, sensor_raw);
  }
  else if (status == POZYX_SUCCESS && c != 'r'){
  }
  else{
    // prints out the error code
    printErrorCode("positioning");
  }
}

// prints the coordinates for either humans or for processing
void printCoordinates(coordinates_t coor, sensor_raw_t sensor_raw){
  uint16_t network_id = remote_id;
  if (network_id == NULL){
    network_id = 0;
  }
  if(!use_processing){
    Serial.print("POS ID 0x");
    Serial.print(network_id, HEX);
    Serial.print(", x(mm): ");
    Serial.print(coor.x/10);
    Serial.print(", y(mm): ");
    Serial.print(coor.y/10);
    Serial.print(", z(mm): ");
    Serial.println(coor.z/10);
  }else{
    Serial.print("POS,0x");
    Serial.print(network_id,HEX);
    Serial.print(",");
    Serial.print(coor.x);
    Serial.print(",");
    Serial.print(coor.y);
    Serial.print(",");
    Serial.print(coor.z);
    Serial.print(",");
    Serial.print(sensor_raw.quaternion[0]);
    Serial.print(",");
    Serial.print(sensor_raw.quaternion[1]);
    Serial.print(",");
    Serial.print(sensor_raw.quaternion[2]);
    Serial.print(",");
    Serial.print(sensor_raw.quaternion[3]);
    Serial.print(",");
    Serial.print(sensor_raw.linear_acceleration[0]);
    Serial.print(",");
    Serial.print(sensor_raw.linear_acceleration[1]);
    Serial.print(",");
    Serial.println(sensor_raw.linear_acceleration[2]);
  }
}

// error printing function for debugging
void printErrorCode(String operation){
  uint8_t error_code;
  if (remote_id == NULL){
    Pozyx.getErrorCode(&error_code);
    Serial.print("ERROR ");
    Serial.print(operation);
    Serial.print(", local error code: 0x");
    Serial.println(error_code, HEX);
    return;
  }
  int status = Pozyx.getErrorCode(&error_code, remote_id);
  if(status == POZYX_SUCCESS){
    Serial.print("ERROR ");
    Serial.print(operation);
    Serial.print(" on ID 0x");
    Serial.print(remote_id, HEX);
    Serial.print(", error code: 0x");
    Serial.println(error_code, HEX);
  }else{
    Pozyx.getErrorCode(&error_code);
    Serial.print("ERROR ");
    Serial.print(operation);
    Serial.print(", couldn't retrieve remote error code, local error: 0x");
    Serial.println(error_code, HEX);
  }
}

// print out the anchor coordinates (also required for the processing sketch)
void printCalibrationResult(){
  uint8_t list_size;
  int status;

  status = Pozyx.getDeviceListSize(&list_size, remote_id);
  Serial.print("list size: ");
  Serial.println(status*list_size);

  if(list_size == 0){
    printErrorCode("configuration");
    return;
  }

  uint16_t device_ids[list_size];
  status &= Pozyx.getDeviceIds(device_ids, list_size, remote_id);

  Serial.println(F("Calibration result:"));
  Serial.print(F("Anchors found: "));
  Serial.println(list_size);

  coordinates_t anchor_coor;
  for(int i = 0; i < list_size; i++)
  {
    Serial.print("ANCHOR,");
    Serial.print("0x");
    Serial.print(device_ids[i], HEX);
    Serial.print(",");
    Pozyx.getDeviceCoordinates(device_ids[i], &anchor_coor, remote_id);
    Serial.print(anchor_coor.x);
    Serial.print(",");
    Serial.print(anchor_coor.y);
    Serial.print(",");
    Serial.println(anchor_coor.z);
  }
}

// function to manually set the anchor coordinates
void setAnchorsManual(){
  for(int i = 0; i < num_anchors; i++){
    device_coordinates_t anchor;
    anchor.network_id = anchors[i];
    anchor.flag = 0x1;
    anchor.pos.x = anchors_x[i];
    anchor.pos.y = anchors_y[i];
    anchor.pos.z = heights[i];
    Pozyx.addDevice(anchor, remote_id);
  }
  if (num_anchors > 4){
    Pozyx.setSelectionOfAnchors(POZYX_ANCHOR_SEL_AUTO, num_anchors, remote_id);
  }
}
