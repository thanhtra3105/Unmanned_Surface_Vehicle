#include <MAVLink.h>   // MAVLink lib

#define LED_PIN 9     // giả lập motor bằng LED
#define MAX_WP 20     // số waypoint tối đa

// ================== GLOBAL VARS ==================
int32_t position_lat = 160668138;  
int32_t position_lon = 1081608325;
int32_t position_alt = 0;
struct Waypoint {
  int32_t lat;
  int32_t lon;
  int32_t alt;
};

Waypoint mission[MAX_WP];
uint16_t wp_count = 0;

// ================== SEND ==================
void send_heartbeat() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_heartbeat_pack(
    1, 200, &msg,
    MAV_TYPE_SURFACE_BOAT, MAV_AUTOPILOT_GENERIC,
    MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE
  );

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
}

void send_position() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_global_position_int_pack(
    1, 200, &msg,
    millis(),         // time_boot_ms
    position_lat,       // lat * 1e7
    position_lon,       // lon * 1e7
    position_alt,                // alt (mm)
    0, 0, 0, 0, 0        // velocities
  );

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
}

void send_mission_ack() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_mission_ack_pack(
    1, 200, &msg,
    1, 200,
    MAV_MISSION_ACCEPTED,
    MAV_MISSION_TYPE_MISSION,
    0
  );

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);

  Serial.println("🎉 Sent MISSION_ACK");
}


// ================== RECEIVE ==================
void handle_command_long(const mavlink_message_t& msg) {
  mavlink_command_long_t cmd;
  mavlink_msg_command_long_decode(&msg, &cmd);
 
  if (cmd.command == MAV_CMD_COMPONENT_ARM_DISARM) {
    if (cmd.param1 == 1) {
      digitalWrite(LED_PIN, HIGH);
      Serial.println("ARM: Motor ON");
    } else {
      digitalWrite(LED_PIN, LOW);
      Serial.println("DISARM: Motor OFF");
    }
  }

  if (cmd.command == MAV_CMD_DO_SET_MODE) {
    Serial.print("Mode change request: ");
    Serial.println(cmd.param1);
  }

  if (cmd.command == MAV_CMD_MISSION_START) {
    Serial.println("🚀 Mission START requested");
    // gia lap chay den cac waypoint
    for (int i = 0; i < wp_count; i++) {
      Serial.print("   ▶ Going to WP ");
      Serial.print(i);
      Serial.print(": lat=");
      position_lat = mission[i].lat;
      Serial.print(mission[i].lat / 1e7, 7);
      Serial.print(", lon=");
      position_lon = mission[i].lon;
      Serial.println(mission[i].lon / 1e7, 7);
      send_heartbeat();
      send_position();
      delay(2000); 
    }
  }
}

void handle_mission_count(const mavlink_message_t& msg) {
  mavlink_mission_count_t mc;
  mavlink_msg_mission_count_decode(&msg, &mc);

  wp_count = mc.count;
  if (wp_count > MAX_WP) wp_count = MAX_WP;

  Serial.print("MISSION_COUNT = ");
  Serial.println(wp_count);
}

void handle_mission_item_int(const mavlink_message_t& msg) {
  mavlink_mission_item_int_t item;
  mavlink_msg_mission_item_int_decode(&msg, &item);
  
  if (item.seq < MAX_WP) {
    mission[item.seq].lat = item.x;
    mission[item.seq].lon = item.y;
    mission[item.seq].alt = item.z;
    Serial.print("Got WP ");
    Serial.print(item.seq);
    Serial.print(": lat=");
    Serial.print(item.x);
    Serial.print(", lon=");
    Serial.println(item.y);

    if (item.seq == wp_count - 1) {
      send_mission_ack();
    }
  }
}

void handle_mavlink_receive() {
  mavlink_message_t msg;
  mavlink_status_t status;

  while (Serial.available() > 0) {
    uint8_t c = Serial.read();

    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      Serial.println(msg.msgid);

      switch (msg.msgid) {
        case MAVLINK_MSG_ID_COMMAND_LONG:
          handle_command_long(msg);
          break;
        case MAVLINK_MSG_ID_MISSION_COUNT:
          handle_mission_count(msg);
          break;
        case MAVLINK_MSG_ID_MISSION_ITEM_INT:
          handle_mission_item_int(msg);
          break;
      }
    }
  }
}

// ================== MAIN ==================
uint32_t pre_time;
void setup() {
  Serial.begin(57600);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);


  Serial.println("🚤 USV Arduino ready");
  pre_time = millis();
}

int32_t pos_lat[3] = {-353617613,-353628987, -353641411};
int32_t pos_lon[3] = {1491658807,1491680479,1491670394 };
void loop() {
  // for(int i=0; i<3; i++)
  // {
  //   position_lat = pos_lat[i];
  //   position_lon = pos_lon[i];
  //   send_heartbeat();
  //   send_position();
  //   handle_mavlink_receive();
  //   delay(2000);
  // }
  if(millis() - pre_time > 2000)
  {
    send_heartbeat();
    send_position();
    handle_mavlink_receive();
    pre_time = millis();
  }
  // send_heartbeat();
  
}
