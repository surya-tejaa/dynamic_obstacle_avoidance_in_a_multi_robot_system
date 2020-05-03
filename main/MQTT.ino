#include <PubSubClient.h>
#include <WiFi.h>
#include <WiFiClient.h>

#define CHECK_TICK 50
#define DESTINATION_BYTE 0
#define PRIORITY_BYTE 1
#define POSE_BYTE 2

#define CURRENTX_BYTE 3
#define CURRENTY_BYTE 4
#define FX1_BYTE 5
#define FY1_BYTE 6
#define FX2_BYTE 7
#define FY2_BYTE 8
#define FX3_BYTE 9
#define FY3_BYTE 10

const char* ssid = "Tierra_605";
const char* password = "tierra605viva";
const char* mqtt_server = "broker.mqttdashboard.com";
const char* Clientt = "Vardhan";

//Set Interrupt ; Clear Interrupt
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

//Create Timer
hw_timer_t * timer_tick = NULL;
hw_timer_t * timer = NULL;

int startx = 0; // start x co-ordinates
int starty = 0; // start y co-ordinates
int currentx = 0;
int currenty = 0;
int endx = 0; // end x co-ordinates
int endy = 0; // end y co-ordinates
int pose = 0; // 0 for x-axis, 1 for y-axis
int dest_reached = 0;
int fx1, fy1, fx2, fy2, fx3, fy3; //future positions
int bot1[20];
int init_value[20];
int input_received = 0;
int gridx = 0;
int gridy = 0;
int start_flag = 0;
int static_pointx = 0;
int static_pointy = 0;
int priority = 0;
int obstacle = 0;

volatile byte val_ready = 0;
volatile int renc = 0;
volatile int lenc = 0;
int16_t rightWheel = 0;
int16_t leftWheel = 0;

WiFiClient espClient;
PubSubClient pubsubClient(espClient);

void setup_wifi()
{
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  randomSeed(micros());
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

bool mqttReconnect()
{
  const uint32_t timeout = 30000;
  static uint32_t lastTime;
  bool result = false;
  if (millis() > lastTime + timeout)
  {
    Serial.print("Attempting MQTT connection...");
    result = pubsubClient.connect(Clientt);

    if (result)
    {
      Serial.println(" connected");
      mqtt_publish(pubsubClient, "Robo1", "CONNECTED");
      result = mqtt_subscribe(pubsubClient, "Robo2");
      mqtt_subscribe(pubsubClient, "Input1");
      mqtt_publish(pubsubClient, "Info", "Please Enter Start and End Coordinates");
    }
    else
    {
      Serial.print(" failed, rc=");
      Serial.println(pubsubClient.state());
    }
    lastTime = millis();
  }

  return result;
}


bool mqtt_subscribe(PubSubClient& client, const String& topic) {
  Serial.print("Subscribing to ");
  Serial.println(topic);

  return client.subscribe(topic.c_str());
}

bool mqtt_publish(PubSubClient& client, const String& topic, const String& value) {
  Serial.print("Publishing topic ");
  Serial.print(topic);
  Serial.print(" = ");
  Serial.println(value);

  return client.publish(topic.c_str(), value.c_str());
}

void grid_change_value()
{
  if ((endx - currentx) < 0)
  {
    gridx = -1;
  }
  else
  {
    gridx = 1;
  }
  if ((endy - currenty) < 0)
  {
    gridy = -1;
  }
  else
  {
    gridy = 1;
  }
}

void callback(char* topic, byte* payload, unsigned int length)
{
  String in_data;
  String name;
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
    in_data += (char)payload[i];
  }
  for (int i = 0; topic[i] != '\0'; i++)
  {
    Serial.print((char)topic[i]);
    name += (char)topic[i];
  }
  Serial.print("IN DATA :");
  Serial.println(in_data);
  if (in_data.compareTo("CONNECTED") == 0)
  {
    return;
  }
  if (name.compareTo("Input1") == 0)
  {
    memset(init_value, 0, sizeof(init_value));
    Serial.println("Initializing values");
    input_received = 1;
    for (int i = 0, j = 0; i < length; ++i)
    {
      if (in_data[i] == '~' || in_data[i] == '\0')
      {
        j++;
      }
      else
      {
        init_value[j] = (init_value[j] * 10) + (in_data[i] - '0');
      }
    }
    priority = init_value[0];
    pose = init_value[1];
    startx = init_value[2];
    starty = init_value[3];
    endx = init_value[4];
    endy = init_value[5];
    static_pointx = init_value[6];
    static_pointy = init_value[7];
    currentx = startx;
    currenty = starty;
    grid_change_value();
    return;
  }
  else
  {
    memset(bot1, 0, sizeof(bot1));
    for (int i = 0, j = 0; i < length; ++i)
    {
      if (in_data[i] == '~' || in_data[i] == '\0')
      {
        j++;
      }
      else
      {
        bot1[j] = (bot1[j] * 10) + (in_data[i] - '0');
      }
    }
    position_check();
    return;
  }
}

void f1(int c)
{
  if (c == 1)
  {
    turnMotor_left();
  }
  else
  {
    turnMotor_right();
  }
}

void f2(int c)
{
  if (c == 1)
  {
    turnMotor_right();
  }
  else
  {
    turnMotor_left();
  }
}

void turn_func(int a, int b, int c)
{
  if ((a - b) > 0)
  {
    f1(c);
  }
  else if ((a - b) < 0)
  {
    f2(c);
  }
  else if (a == b)
  {
    if (b >= 5)
    {
      f2(c);
    }
    else if (b < 5)
    {
      f1(c);
    }
  }
  startMotors(true);
}

void obstacle_avoidance(int bot_x, int bot_y)
{
  Serial.print("We have preference");
  if (pose == 0)
  {
    Serial.println("Moving along x-axis");
    if (currenty == bot_y)
    {
      Serial.println("Robot in front of us on y-axis");
      if (currenty == endy)
      {
        Serial.println("All three on the same line on y-axis");
        turn_func(endy, currenty, gridx);
        pose = 1;
        while (!val_ready)
        {
          yaw_loop();
        }
        val_ready = 0;
        currenty += gridy;
        turn_func(currentx, endx, gridy);
        pose = 0;
        grid_change_value();
        return;
      }
      else
      {
        Serial.println("Turning in the direction of the end point");
        pose = 1;
        turn_func(endy, currenty, gridx);
        grid_change_value();
        return;
      }
    }
    else if (currentx == bot_x)
    {
      Serial.println("Robot beside us on X-axis");
      //go straight
      if (currentx == endx)
      {
        Serial.println("All three on the same x-axis");
        Serial.println("Move one grid forward and then turn towards the end point");
        while (!val_ready)
        {
          yaw_loop();
        }
        val_ready = 0;
        currenty += gridy;
        pose = 1;
        //reset
        turn_func(endy, currenty, gridx);
        grid_change_value();
        return;

      }
      else
      {
        Serial.println("Continue original path");
        //continue path
        return;
      }
    }
    else
    {
      if (endx == bot_x)
      {
        Serial.println("Robot in random position and end point and robot on same x-axis");
        turn_func(endy, currenty, gridx);
        pose = 1;
        return;
      }
      else
      {
        Serial.println("Robot in random position and continue path");
        return;
      }
    }
  }
  else if (pose == 1)
  {
    Serial.println("Moving along y-axis");
    if (currentx == bot_x)
    {
      Serial.println("Other robot in front of our robot on y-axis");
      if (currentx == endx)
      {
        Serial.println("all three on y-axis, turn and move by one grid and then turn to original pose");
        turn_func(currentx, endx, gridy);
        pose = 0;
        while (!val_ready)
        {
          yaw_loop();
        }
        val_ready = 0;
        currentx += gridx;
        turn_func(endy, currenty, gridx);
        pose = 1;
        grid_change_value();
        return;
      }
      else
      {
        Serial.println("Turn and reset the robot");
        //turn
        pose = 0;
        turn_func(currentx, endx, gridy);
        grid_change_value();
        return;
      }
    }
    else if (currenty == bot_y)
    {
      Serial.println("Robot and other robot on same y-axis");
      //go straight
      if (currenty == endy)
      {
        Serial.println("All three on the same y-axis");
        while (!val_ready)
        {
          yaw_loop();
        }
        currenty += gridy;
        pose = 0;
        //reset
        turn_func(currentx, endx, gridy);
        grid_change_value();
        return;
      }
      else
      {
        Serial.println("end point not on the same axis as the other robotso continue path");
        //continue path
        return;
      }
    }
    else
    {
      Serial.println("In random position");
      if (endy == bot_y)
      {
        Serial.println("End point and second robot on same axis");
        turn_func(currentx, endx, gridy);
        pose = 0;
        return;
      }
      else
      {
        Serial.println("End point not on the same axis so continue path");
        return;
      }
    }
  }
}

void priority_check(int bot_priority)
{
  if (bot_priority > priority)
  {
    obstacle_avoidance(bot1[CURRENTX_BYTE], bot1[CURRENTY_BYTE]);
  }
  else
  {
    Serial.println("Less priority");
    startMotors(false);
    while (1)
    {
      pubsubClient.loop();
      if (!(((bot1[FX2_BYTE] == fx3) && (bot1[FY2_BYTE] == fy3)) || ((bot1[FX2_BYTE] == fx2) && (bot1[FY2_BYTE] == fy2)) || ((bot1[FX2_BYTE] == fx1) && (bot1[FY2_BYTE] == fy1))))
      {
        break;
      }
      else if (!(((bot1[FX3_BYTE] == fx3) && (bot1[FY3_BYTE] == fy3)) || ((bot1[FX3_BYTE] == fx2) && (bot1[FY3_BYTE] == fy2)) || ((bot1[FX3_BYTE] == fx1) && (bot1[FY3_BYTE] == fy1))))
      {
        break;
      }
      else if (!(((bot1[FX1_BYTE] == fx3) && (bot1[FY1_BYTE] == fy3)) || ((bot1[FX1_BYTE] == fx2) && (bot1[FY1_BYTE] == fy2)) || ((bot1[FX1_BYTE] == fx1) && (bot1[FY1_BYTE] == fy1))))
      {
        break;
      }
    }
    startMotors(true);
  }
}

void position_check()
{
  if (((bot1[FX2_BYTE] == fx3) && (bot1[FY2_BYTE] == fy3)) || ((bot1[FX2_BYTE] == fx2) && (bot1[FY2_BYTE] == fy2)) || ((bot1[FX2_BYTE] == fx1) && (bot1[FY2_BYTE] == fy1)))
  {
    obstacle = 1;
    return;
  }
  else if (((bot1[FX3_BYTE] == fx3) && (bot1[FY3_BYTE] == fy3)) || ((bot1[FX3_BYTE] == fx2) && (bot1[FY3_BYTE] == fy2)) || ((bot1[FX3_BYTE] == fx1) && (bot1[FY3_BYTE] == fy1)))
  {
    obstacle = 1;
    return;
  }
  else if (((bot1[FX1_BYTE] == fx3) && (bot1[FY1_BYTE] == fy3)) || ((bot1[FX1_BYTE] == fx2) && (bot1[FY1_BYTE] == fy2)) || ((bot1[FX1_BYTE] == fx1) && (bot1[FY1_BYTE] == fy1)))
  {
    obstacle = 1;
    return;
  }
}

void mqtt_send()
{
  String fd = "~";
  if (dest_reached == 0)
  {
    switch (pose)
    {
      case 0:
        fx1 = currentx + gridx;
        fx2 = currentx + (2 * gridx);
        fx3 = currentx + (3 * gridx);
        fy1 = fy2 = fy3 = currenty;
        if (fx3 == endx);
        else if (fx2 == endx) {
          fy3 = currenty + gridy;
          fx3 = fx2;
        }
        else if (fx1 == endx) {
          fy2 = currenty + gridy;
          fy3 = currenty + (2 * gridy);
          fx3 = fx2 = fx1;
        }
        else if (currentx == endx) {
          fx3 = fx2 = fx1 = currentx;
          fy1 = currenty + gridy;
          fy2 = currenty + (2 * gridy);
          fy3 = currenty + (3 * gridy);
        }
        if (currenty == endy)
          fy1 = fy2 = fy3 = endy;
        break;
      case 1:
        fy1 = currenty + gridy;
        fy2 = currenty + (2 * gridy);
        fy3 = currenty + (3 * gridy);
        fx1 = fx2 = fx3 = currentx;
        if (fy3 == endy);
        else if (fy2 == endy) {
          fx3 = currentx + gridx;
          fy3 = fy2;
        }
        else if (fy1 == endy) {
          fx2 = currentx + gridx;
          fx3 = currentx + (2 * gridx);
          fy3 = fy2 = fy1;
        }
        else if (currenty == endy) {
          fy3 = fy2 = fy1 = currenty;
          fx1 = currentx + gridx;
          fx2 = currentx + (2 * gridx);
          fx3 = currentx + (3 * gridx);
        }
        if (currentx == endx)
          fx1 = fx2 = fx3 = endx;
        break;
      default: Serial.println("Pose information incorrect");
    }
  }
  else if (dest_reached == 1)
  {
    fx1 = fx2 = fx3 = currentx;
    fy1 = fy2 = fy3 = currenty;
  }
  String data = dest_reached + fd + priority + fd + pose + fd + currentx + fd + currenty + fd + fx1 + fd + fy1 + fd + fx2 + fd + fy2 + fd + fx3 + fd + fy3;
  Serial.println("Mqtt data:");
  Serial.println(data);
  mqtt_publish(pubsubClient, "Robo1", data);
  if ((static_pointx != 11) && (static_pointy != 11))
  {
    if (((static_pointx == fx3) && (static_pointy == fy3)) || ((static_pointx == fx2) && (static_pointy == fy2)) || ((static_pointx == fx1) && (static_pointy == fy1)))
    {
      obstacle = 2;
      return;
    }
  }
  pubsubClient.loop();
}

void destination_reach_check()
{
  if ( (currentx == endx) && (currenty == endy))
  {
    startMotors(false);
    Serial.println("Destination reached");
    dest_reached = 1;
    mqtt_send();
    while (1);
  }
}

void setup()
{
  Serial.begin(115200);
  setup_wifi();
  pubsubClient.setServer(mqtt_server, 1883);
  pubsubClient.setCallback(callback);
  pulse_counter_init();
  straight_line_drive_init();
}

void loop()
{
  if (!pubsubClient.connected()) {
    mqttReconnect();
  }
  if (input_received != 1)
  {
    pubsubClient.loop();
  }
  if (input_received == 1)
  {
    if (start_flag == 0)
    {
      startMotors(true); //motors start
      start_flag = 1;
    }
    yaw_loop();

    if (val_ready == 1)
    {
      if (obstacle != 0)
      {
        startMotors(false);
      }
      val_ready = 0;
      if (pose == 0)
      {
        if (currentx != endx)
        {
          Serial.println("Current X co-ordinates");
          currentx += gridx;
          Serial.println(currentx);
          mqtt_send();
          destination_reach_check();
          if (currentx == endx)
          {
            Serial.println("Turn");
            turn_func(endy, currenty, gridx);
            pose = 1;
          }
        }
      }
      else if (pose == 1)
      {
        if (currenty != endy)
        {
          Serial.println("current Y co-ordinates");
          currenty += gridy;
          Serial.println(currenty);
          mqtt_send();
          destination_reach_check();
          if (currenty == endy)
          {
            Serial.println("Turn");
            turn_func(currentx, endx, gridy);
            pose = 0;
          }
        }
      }
      if (obstacle == 1)
      {
        Serial.println("Dynamic Obstacle");
        priority_check(bot1[PRIORITY_BYTE]);
        obstacle = 0;
      }
      else if (obstacle == 2)
      {
        Serial.println("Static Obstacle");
        obstacle_avoidance(static_pointx, static_pointy);
        obstacle = 0;
      }
    }
  }
}
