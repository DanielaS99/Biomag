//Main controller Code
//Temperature, pH, Stirring and Magnetic field 
//Sanchez-Orozco, Landivar, Palacios, Castro

#include <ESP32Servo.h>
#include <analogWrite.h>
#include <ESP32PWM.h>
#include <OneWire.h>                
#include <DallasTemperature.h>

void controlTemp(void *pvParameters);
void controlMotor(void *pvParameters);
void sendData(void *pvParameters);
void controlPH(void *pvParameters);
void leerPH(void *pvParameters);
void iniciarTemp();
void iniciarPH();
void leerTemp(void *pvParameters);
void bomba(void *pvParameters);
void leerUltrasonico (void *pvParameters);
void miniM_fun (void *pvParameters);

//Temperature 
#define rele 0
#define pinTemp 25
 
OneWire ourWire(pinTemp);                //pin 2 as a bus 
DallasTemperature sensors(&ourWire); //Temperature sensor variable
static float P_temp = 3.3; //3.3
static float I_temp = 0.002;
static float D_temp = 82.5;

int ref_temp = 0;
bool pid_off_t = true;
float pidLimit_t;
float pidOffV_t;
int count_e_t;

float error_t;
float integral_t;
float derivate_t;
float dt_t;

float acum_derivate_t;
float acum_error_t;
int nTemp;

float voltage_t = 1;
float T_t = 5000;
TickType_t Ton;
long t0_t;
int stateT = 0;

float temp;
long tdt_t;

//Motor

#define ENCA 27
#define ENCB 15
#define PWM 14
int ref = 0;
#define n_velFilter 40
#define n_voltFilter 1

long pos;
long pos_freeze;
long old_pos;
float vel;
float vels[n_velFilter];
float filtered_vel;
long t0;
bool up;
long dt;
bool firstLoop = true;

float error = ref;
float integral;
float derivative;
static float P = 0.18;//.18
static float I = 0.000015; //0.8
static float D = 0.15; //0.0016
float voltage = 0;
float volts[n_voltFilter];
float filtered_voltage;
int ident=0;

bool enca=false;

//pH
#define phPin 34
#define n_phFilter 1000
float Offset = 24.35; //deviation compensate
float m_ph = -6.2;
float phActual;
float filtered_pH;
float pHs[n_phFilter];
Servo servoAcido;
bool flagAcido= true;
int posAcido=0;
Servo servoBase;
bool flagBase= true;
int posBase=0;
float phMin=6.5;
float phMax=7.5;

//Pump
int pinBomba=23;
int activarBomba=0;
float distanciaStop=10;

//Ultrasonic
#define TRIGGER 12
#define ECHO 26
float distancia;

//Mini Motor
int fCA=36;
int fCB=39;
int miniM=22;
int subirMiniM=0;
int bajarMiniM=0;
int giroSubir=2;
int giroBajar=4;

//Receiving data from the screen
char receivedData[10];
String receivedString;

  

TaskHandle_t ControlMotor_h;
TaskHandle_t ControlTemp_h;
TaskHandle_t LeerTemp_h;
TaskHandle_t ControlPH_h;
TaskHandle_t LeerPH_h;
TaskHandle_t Bomba_h;
TaskHandle_t LeerUltrasonico_h;
TaskHandle_t MiniM_h;

portMUX_TYPE myMutex = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE myMutex2 = portMUX_INITIALIZER_UNLOCKED;


void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 16, 17);
  Serial2.setTimeout(100);
  iniciarTemp();
  iniciarPH();
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(PWM, OUTPUT);

xTaskCreatePinnedToCore(
      controlMotor, /* Function to implement the task */
      "ControlMotor", /* Name of the task */
      10000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      2,  /* Priority of the task */
      &ControlMotor_h,  /* Task handle. */
      0); /* Core where the task should run */

xTaskCreatePinnedToCore(
      controlTemp, /* Function to implement the task */
      "ControlTemp", /* Name of the task */
      10000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      1,  /* Priority of the task */
      &ControlTemp_h,  /* Task handle. */
      1); /* Core where the task should run */

xTaskCreatePinnedToCore(
      leerTemp, /* Function to implement the task */
      "LeerTemp", /* Name of the task */
      10000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      1,  /* Priority of the task */
      &LeerTemp_h,  /* Task handle. */
      1); /* Core where the task should run */

xTaskCreatePinnedToCore(
      leerPH, /* Function to implement the task */
      "LeerPH", /* Name of the task */
      10000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      1,  /* Priority of the task */
      &LeerPH_h,  /* Task handle. */
      1); /* Core where the task should run */
  //vTaskStartScheduler();

xTaskCreatePinnedToCore(
      controlPH, /* Function to implement the task */
      "ControlPH", /* Name of the task */
      10000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      1,  /* Priority of the task */
      &ControlPH_h,  /* Task handle. */
      1); /* Core where the task should run */

xTaskCreatePinnedToCore(
      bomba, /* Function to implement the task */
      "Bomba", /* Name of the task */
      10000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      1,  /* Priority of the task */
      &Bomba_h,  /* Task handle. */
      1); /* Core where the task should run */
  //vTaskStartScheduler();

xTaskCreatePinnedToCore(
      leerUltrasonico, /* Function to implement the task */
      "Leer Ultrasonico", /* Name of the task */
      10000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      1,  /* Priority of the task */
      &LeerUltrasonico_h,  /* Task handle. */
      1); /* Core where the task should run */
  //vTaskStartScheduler();

xTaskCreatePinnedToCore(
      miniM_fun, /* Function to implement the task */
      "MiniM", /* Name of the task */
      10000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      1,  /* Priority of the task */
      &MiniM_h,  /* Task handle. */
      1); /* Core where the task should run */
  //vTaskStartScheduler();
}



void loop() 
{
  Serial.print(filtered_vel);
  Serial.print(";");
  Serial.print(temp);
  Serial.print(";");
  Serial.print(filtered_pH);
  Serial.println(";*");

  Serial2.print(filtered_vel);
  Serial2.print(";");
  Serial2.print(temp);
  Serial2.print(";");
  Serial2.print(filtered_pH);
  Serial2.println(";*");

  //receivedData[10] = 0;
  if(Serial2.readBytesUntil('*', receivedData, 10)){
    receivedString = String(receivedData).substring(0); //Remove the line break
//    Serial.print("data=");
//    Serial.println(receivedString);
    
    if (receivedData[0] == 'V') {
      ref = receivedString.substring(2).toInt();
      Serial.println(ref);
    }
    else if (receivedData[0] == 'T') {
      ref_temp = receivedString.substring(2).toInt();
      Serial.println(ref_temp);
    }
    else if (receivedData[0] == 'P') {
      phMin = receivedString.substring(2).toFloat()-0.5;
      phMax = receivedString.substring(2).toFloat()+0.5;
      Serial.println(String(phMin) + "-" + String(phMax));
    }
  }
}

void readEncoder(){
  int b = digitalRead(ENCB);
  //pos++;
  if (b > 0){
    pos++;
  }
//  else{
//    pos--;
//  }
}

float filter(float newValue, float vector[], bool firstLoop, int n_filter){
  float value_acum = 0;
  
  for(int i=0; i<n_filter-1; i++){
    vector[i]=vector[i+1];
  }

  if (firstLoop){
    for(int i=0; i<n_filter; i++){
      vector[i] = newValue;
    }
  }
  
  vector[n_filter-1] = newValue;
  
  float sorted[n_filter];
  for (int i=0; i<n_filter; i++) sorted[i] = vector[i];
  
  isort(sorted, n_filter);
  
//  for (int i=0; i<n_filter; i++){
//    Serial.print(sorted[i]);
//    Serial.print(" ");
//  }
//  Serial.println("");
  
  float Q1 = sorted[n_filter/4];
  float Q3 = sorted[3*n_filter/4];
  float RI = Q3-Q1;  

  int counter=0;
  for(int i=0; i<n_filter; i++){
    if (vector[i] <= Q3+1.5*RI && vector[i] >= Q1-1.5*RI) {//&& vector[i]<(voltage*300/255)
      value_acum += vector[i];
      counter++;
    }
  }

  return value_acum/counter;
}

void isort(float a[], int n)
{
  for (int i = 1; i < n; ++i)
  {
    float j = a[i];
    int k;
    for (k = i - 1; (k >= 0) && (j < a[k]); k--)
    {
      a[k + 1] = a[k];
    }
    a[k + 1] = j;
  }
}

void iniciarTemp(){
  sensors.begin();   //Se inicia el sensor
  pinMode(rele, OUTPUT);
  sensors.requestTemperatures();   //Se envía el comando para leer la temperatura
  temp = sensors.getTempCByIndex(0); //Se obtiene la temperatura en ºC   
   
  pidOffV_t=4*(ref_temp-temp);
  pidLimit_t=4*(ref_temp-temp)/5;
  if (pidOffV_t>120.0){
    pidOffV_t=120.0;
  }
  
  error_t = ref_temp - temp;
  tdt_t = millis(); 
}

void controlTemp(void *pvParameters){
  vTaskDelay(1000/portTICK_PERIOD_MS);
  for(;;){
    stateT++;
  
    if (error_t < pidLimit_t && pid_off_t) count_e_t++;
    if (count_e_t > 10 && pid_off_t){
      pid_off_t = false;
      count_e_t = 0;
    }
    if (error_t > pidLimit_t && not pid_off_t) count_e_t++;
    if (count_e_t > 10 && not pid_off_t){
      pid_off_t = true;
      integral_t = 0;
      acum_derivate_t = 0;
      acum_error_t = 0;
      nTemp = 0;
      count_e_t = 0;
    }
  
    if (pid_off_t){
    if (error_t > 0) voltage_t = pidOffV_t;
    else  voltage_t = 0;
    }
    
    if (not pid_off_t){
      derivate_t = acum_derivate_t/(float)nTemp;
      error_t = acum_error_t/(float)nTemp + derivate_t*T_t/2000;
      acum_derivate_t = 0;
      acum_error_t = 0;
      nTemp = 0;
      
      voltage_t = P_temp*error_t + I_temp*integral_t + D_temp*derivate_t;
      if (voltage_t>120) voltage_t = 120;
      if (voltage_t<0) voltage_t = 0;
      if (error_t<=0) voltage_t = 0;
    }
      Ton=(voltage_t*T_t/120)/portTICK_PERIOD_MS;
      digitalWrite(rele, HIGH);
      vTaskDelay(Ton);
      digitalWrite(rele, LOW);
      vTaskDelay(T_t/portTICK_PERIOD_MS-Ton);
  }
}

void controlMotor(void *pvParameters){
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
  for(;;){
//    if (not enca && digitalRead(ENCA)){
//      readEncoder();
//      enca=true;
//    }
//    if (not digitalRead(ENCA)){
//      enca=false;
//    }
//    Serial.print("M = ");
    //taskENTER_CRITICAL(&myMutex);
    dt = micros() - t0;
    pos_freeze = pos;
    t0 = micros();
    //taskEXIT_CRITICAL(&myMutex);
   
    vel = (pos_freeze - old_pos)*111111.1111/dt;
    filtered_vel = filter(vel, vels, firstLoop, n_velFilter);
    integral += error*1000000/dt;
    derivative = ((ref - filtered_vel) - error)*1000000/dt;
    error = ref - filtered_vel;
    voltage = P*error + I*integral + D*derivative;
    if (voltage>10.6) voltage = 10.6;
    if (voltage<0) voltage = 0;
    
    filtered_voltage = filter(voltage, volts, firstLoop, n_voltFilter);
    //    analogWrite(PWM, voltage/10.6*255);
    analogWrite(PWM, 255*filtered_voltage/10.6);
    //analogWrite(PWM,127.5);
    
    if (firstLoop){
      firstLoop = false;
    }

    old_pos = pos_freeze;
    vTaskDelay(2/portTICK_PERIOD_MS);
  }
}

void leerTemp(void *pvParameters){
  int contError=0;
  
  for(;;){
    float temp_old=temp;
    sensors.requestTemperatures();   //Se envía el comando para leer la temperatura
    temp = sensors.getTempCByIndex(0); //Se obtiene la temperatura en ºC 
    if (temp==-127){
      //Serial.println("ERROR");
      temp=temp_old;
      contError++;
    }else{
      contError=0;
    }
    
    if (contError==20){
      //Serial.println(contError);
      //ESP.restart();
    }
    dt_t = millis() - tdt_t;
    tdt_t = millis();
    
    derivate_t = ((ref_temp - temp) - error_t)/dt_t*1000;
    error_t = ref_temp - temp;
    integral_t += error_t*dt_t/1000;
    acum_derivate_t += derivate_t;
    acum_error_t += error_t;
    nTemp++;
  }
}

void iniciarPH(){

}

void leerPH(void *pvParameters){
  bool firstLoopPH = true;
  for(;;){
    float x=(float)analogRead(phPin)/1000;
     x = 0.013636*x*x*x*x*x - 0.15224*x*x*x*x + 0.59311*x*x*x - 0.98844*x*x + 1.5144*x + 0.093552; //Voltage 3.3V
    float phValue=(float)x*1.5; //Voltage 5V
//    Serial.print(phValue,2); 
//    Serial.println("");
    phActual=m_ph*phValue+Offset; //convert the voltage into pH value
    filtered_pH = filter(phActual, pHs, firstLoopPH, n_phFilter);
    
    if (firstLoopPH) firstLoopPH = false;
  }
}

void controlPH(void *pvParameters){
//  ESP32_ISR_Servos.useTimer(2);

//  int servoIndex1 = ESP32_ISR_Servos.setupServo(18, 500, 2500);
//  int servoIndex2 = ESP32_ISR_Servos.setupServo(19, 500, 2500);
  Servo servo;
  servo.attach(4, 500, 2500);
  
  servoBase.write(35);
  servoBase.attach(19,500,2500);
  servoBase.setPeriodHertz(50);

  servoAcido.write(158);
  servoAcido.attach(18.,500,2500);
  servoAcido.setPeriodHertz(50);

  //servo.detach();
//  
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  vTaskDelay(60000/portTICK_PERIOD_MS); 
  for(;;){
    if (filtered_pH>phMax){
        for (posAcido = 158; posAcido >= 0; posAcido -= 1) {
            servoAcido.write(posAcido);
//            ESP32_ISR_Servos.setPosition(servoIndex1, posAcido);
           // Serial.print("acido=");
//            Serial.println(posAcido);
            vTaskDelay(30/portTICK_PERIOD_MS);  
            }
        vTaskDelay(5000/portTICK_PERIOD_MS); 
        for (posAcido = 0; posAcido <= 158; posAcido += 1) {
            servoAcido.write(posAcido);
//            ESP32_ISR_Servos.setPosition(servoIndex1, posAcido);
            vTaskDelay(30/portTICK_PERIOD_MS); 
            }

    }
    if (filtered_pH<phMin){
        for (posBase = 35; posBase <= 180; posBase += 1) {
            servoBase.write(posBase);
//            ESP32_ISR_Servos.setPosition(servoIndex2, posBase);
//            analogWrite(19, (posBase/180*2000+500)/20000*255);
            vTaskDelay(30/portTICK_PERIOD_MS); 
            }
        vTaskDelay(5000/portTICK_PERIOD_MS); 
        
        for (posBase = 180; posBase >= 35; posBase -= 1) {
            servoBase.write(posBase);
//            ESP32_ISR_Servos.setPosition(servoIndex2, posBase);
            vTaskDelay(30/portTICK_PERIOD_MS); 
            }
      }
    vTaskDelay(15000/portTICK_PERIOD_MS); 
    }
  }

  void bomba(void *pvParameters){
    for(;;){
      if(activarBomba==1){
        if (distancia<10){
          digitalWrite(pinBomba,HIGH);
        }
        else{
          digitalWrite(pinBomba,LOW);
        }
    }
  }
  }

  void leerUltrasonico(void *pvParameters){
    for(;;){
      digitalWrite(TRIGGER, LOW);
      delayMicroseconds(2);
      digitalWrite(TRIGGER, HIGH);
      delayMicroseconds(10);
      digitalWrite(TRIGGER, LOW);

//      distancia = tiempo * 0.000001 * 34300.0 / 2.0;
      delay(250);
    }
  }

void miniM_fun(void *pvParameters){
  for (;;){
    if(subirMiniM==1){
//      if(FCA==LOW){
//        digitalWrite(giroSubir,HIGH);
//        digitalWrite(giroBajar,LOW);
//        analogWrite(miniM,255);
//      }else {
//        analogWrite(miniM,0);
//        subirMiniM==0;
//      }
    }
    if(bajarMiniM==1){
//      if(FCB==LOW){
//        digitalWrite(giroSubir,LOW);
//        digitalWrite(giroBajar,HIGH);
//        analogWrite(miniM,255);
//      }else {
//        analogWrite(miniM,0);
//        bajarMiniM==0;
//    }
    }
  }
} 
