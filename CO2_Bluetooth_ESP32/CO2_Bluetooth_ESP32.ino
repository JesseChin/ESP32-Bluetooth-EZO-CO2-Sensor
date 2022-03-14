#include <Ezo_i2c.h>
#include <Ezo_i2c_util.h>
#include <Wire.h>
#include <iot_cmd.h>
#include <math.h>
#include <sequencer1.h>
#include <sequencer2.h>
#include <sequencer3.h>
#include <sequencer4.h>
#include "BluetoothSerial.h"

// Credit to Sahaj Patel for getting CO2 sensors to work over I2C

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#define SENSOR_POLLING_PERIOD 10
#define BLUETOOTH_POLLING_PERIOD 20
#define co2Addr0 0x68
#define co2Addr1 0x6A
#define relay 15

static BluetoothSerial SerialBT;


static Ezo_board CO2 = Ezo_board(105, "CO2");
static int atlasVal = 0;
// static const int co2Addr0 = 0x68;
static int co2Value0 = 0;
// static const int co2Addr1 = 0x6A;
static int co2Value1 = 0;

// static int relay = 15;

// Globals
static int wAvg = 0;

int readCO2(int co2Addr) {
  int co2_value = 0;

  digitalWrite(13, HIGH);

  Wire.beginTransmission(co2Addr);
  Wire.write(0x22);
  Wire.write(0x00);
  Wire.write(0x08);
  Wire.write(0x2A);

  Wire.endTransmission();

  vTaskDelay(SENSOR_POLLING_PERIOD / portTICK_PERIOD_MS);

  Wire.requestFrom(co2Addr, 4);

  vTaskDelay(SENSOR_POLLING_PERIOD / portTICK_PERIOD_MS);

  byte i = 0;
  byte buffer[4] = {0, 0, 0, 0};

  while (Wire.available()) {
    buffer[i] = Wire.read();
    i++;
  }

  co2_value = 0;
  co2_value |= buffer[1] & 0xFF;
  co2_value = co2_value << 8;
  co2_value |= buffer[2] & 0xFF;

  byte sum = 0; // Checksum Byte
  sum = buffer[0] + buffer[1] + buffer[2];

  if (sum == buffer[3]) {
    // Success!
    digitalWrite(13, LOW);
    return co2_value;
  } else {
    digitalWrite(13, LOW);
    return 0;
  }
}

Sequencer2 Seq(&step1, 1000, &step2, 0);

void step1() {
  CO2.send_read_cmd();
  co2Value0 = readCO2(co2Addr0);
  co2Value1 = readCO2(co2Addr1);
}

static bool DataGone = false;
static int iter = 0;

void step2() {
  CO2.receive_read_cmd();
  // select all in bool statement to update datagone value
  bool atG = false;
  bool atK0 = false;
  bool atK1 = false;
  if ((CO2.get_error() == Ezo_board::SUCCESS) &&
      (CO2.get_last_received_reading() > -1000.0)) {
    atlasVal = CO2.get_last_received_reading();
    atG = true;
  } else {
    atlasVal = 0;
    iter++;
    if (!DataGone) {
      DataGone = true;
    } else {
      if (iter >= 100) {
        Serial.println("Data Loss... ERROR ERROR");
      }
    }
  }
  if (co2Value0 > 0) {
    atK0 = true;
  } else {
    iter++;
    if (!DataGone) {
      DataGone = true;
    } else {
      if (iter >= 100) {
        Serial.println("Data Loss... ERROR ERROR");
      }
    }
  }
  if (co2Value1 > 0) {
    atK1 = true;
  } else {
    iter++;
    if (!DataGone) {
      DataGone = true;
    } else {
      if (iter >= 100) {
        Serial.println("Data Loss... ERROR ERROR");
      }
    }
  }

  if (co2Value0 > 0 && co2Value1 > 0 && atlasVal > 0) {
    Serial.println("********************************************");
    Serial.print("A_CO2: ");
    Serial.println(atlasVal);
    Serial.print("K30_0: ");
    Serial.println(co2Value0);
    Serial.print("K30_1: ");
    Serial.println(co2Value1);
    DataGone = false;
    iter = 0;

    wAvg = round((0.6 * atlasVal) + (0.2 * co2Value0) + (0.2 * co2Value1));
    Serial.print("Weighted average: ");
    Serial.println(wAvg);

    if (wAvg < 800) {
      Serial.println("CO2 VALUE BELOW 800... Turning on regulator.");
      digitalWrite(relay, HIGH);
    } else {
      Serial.println("CO2 VALUE ABOVE 800... Shutting down regulator.");
      digitalWrite(relay, LOW);
    }
  }
  //  else
  //  {
  //    Serial.println("********************************************");
  //    if(atG)
  //      Serial.println("A_CO2: Checksum failed / Communication failure");
  //    if(atK0)
  //      Serial.println("K30_0: Checksum failed / Communication failure");
  //    if(atK1)
  //      Serial.println("K30_1: Checksum failed / Communication failure");
  //  }
}
void bluetoothTask(void *parameters) {
  char data;
  for(;;) {
    if (SerialBT.available()) {
      data = SerialBT.read();
      if (data == 'a')
        SerialBT.write(atlasVal);
      else if (data == '0')
        SerialBT.write(co2Value0);
      else if (data == '1')
        SerialBT.write(co2Value1);
      else if (data == 'w')
        SerialBT.write(wAvg);
    }
    vTaskDelay(BLUETOOTH_POLLING_PERIOD / portTICK_PERIOD_MS);
  }
}
void sequenceTask(void *parameters) {
  for(;;) {
    Seq.run();
  }
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  SerialBT.begin("ESP32-CO2-Sensor");
  pinMode(13, OUTPUT);
  pinMode(relay, OUTPUT);

  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println("Setup complete");

  TaskHandle_t xBluetooth = NULL;
  TaskHandle_t xSequence = NULL;
  xTaskCreate(sequenceTask, "Sequence", 8192, NULL, 2, &xSequence);
  xTaskCreate(bluetoothTask, "Bluetooth", 2048, NULL, 2, &xBluetooth);

  vTaskDelete(NULL);
}
void loop() {}
