/*
 * Simple Power Acquisition - AE9000
 * by Matsu Thornton 08/23/2018
 * Ho'omaluo Laboratories
 */ 

//**********INCLUDES AND LIBRARIES************
#include <SPI.h>
#include <Hoomaluo_ADE9000CalibrationInputs.h>
#include <Hoomaluo_ADE9000RegMap.h>
#include <Hoomaluo_ADE9000.h>
#include <stdint.h>
#include <math.h>

//**********PIN DEFINITIONS************


//*********GLOBAL VARIABLES*****************
float current_time;                     //for tracking time interval for acquire
float last_time = 0;
float acquire_time = 5;                 //number of seconds between acquire
double avrmsaccum,bvrmsaccum,cvrmsaccum,airmsaccum,birmsaccum,cirmsaccum;  //Accumulators
double awattaccum,bwattaccum,cwattaccum,avaaccum,bvaaccum,cvaaccum;
int dccount = 1;                  //counter

//******CALIBRATION********
const int irms_os = 0;          //ADC offset at low current reading
const int vrms_os = 0;          //ADC offset at low voltage reading
float vrms_cal = 0.000017183;   //Use calibrated multimeter to read voltage and adjust this number for calibration
float irms_cal = 0.000004269;   //Same as above...use calibrated clamp meter to adjust current values accordingly
float watt_cal = 0.00989166;    //After calibrating vrms and irms, multiply them together to verify volt-amperes(va) then adjust this number until va matches

//**********INITIALIZE ADE9000**************
ADE9000Class ade9000;         //FIRST POWER MONITOR OBJECT
//ADE9000Class ade9000_2;       //SECOND POWER MONITOR OBJECT uncomment to use more boards on different SS pins
#define SPI_SPEED 500000     //SPI Speed. The mini dev board seems to have problems at higher SPI speeds
#define CS_PIN 7            // SS PIN FOR POWER MONITOR 1
//#define CS_PIN 31            // SS PIN FOR POWER MONITOR 2 just example of how to enable a second monitor device
#define VAL_RUN 1

//***********FUNCTION PROTOTYPES************
void init_ADE_regs(); //INITIALIZE GAIN REGISTERS FOR ADE9000


void setup() {
    Serial.begin(9600); //START UART OVER USB
    Serial.println("INITIALIZATION");
    init_ADE_regs();
    Serial.println("INITIALIZE SUCCESS");
}

void loop() {
     //**************ACQUIRE CALIBRATION DATA FROM SERIAL PORT*********************
    while (Serial.available()) {
    String line = Serial.readStringUntil('\r');
    if(line.endsWith("calibrate"))
    {
      //Send string over serial connection to recalibrate device
      //SAMPLE STRING vrms_cal?irms_cal?watt_cal?calibrate
      //replace calibration values with numbers separated by ?
      char linebuf[100];
      char *one, *two;
      line.toCharArray(linebuf, 100);
      one = strtok(linebuf, "?");
      vrms_cal = atof(one);
      one = strtok(NULL, "?");
      irms_cal = atof(one);
      one = strtok(NULL, "?");
      watt_cal = atof(one);
    }
  }

    //*********ACCUMULATE AC POWER DATA*******************
      avrmsaccum =    avrmsaccum+(ade9000.SPI_Read_32(ADDR_AVRMS1012)-vrms_os)*vrms_cal;
      airmsaccum =    airmsaccum+(ade9000.SPI_Read_32(ADDR_AIRMS1012)-irms_os)*irms_cal;
      awattaccum =    awattaccum+ade9000.twos_compliment(ade9000.SPI_Read_32(ADDR_AWATT)) * -watt_cal;
      avaaccum =      avaaccum+ade9000.twos_compliment(ade9000.SPI_Read_32(ADDR_AVA)) * -watt_cal;

      bvrmsaccum =    bvrmsaccum+(ade9000.SPI_Read_32(ADDR_BVRMS1012)-vrms_os)*vrms_cal;
      birmsaccum =    birmsaccum+(ade9000.SPI_Read_32(ADDR_BIRMS1012)-irms_os)*irms_cal;
      bwattaccum =    bwattaccum+ade9000.twos_compliment(ade9000.SPI_Read_32(ADDR_BWATT)) * -watt_cal;
      bvaaccum =      bvaaccum+ade9000.twos_compliment(ade9000.SPI_Read_32(ADDR_BVA)) * -watt_cal;

      cvrmsaccum =    cvrmsaccum+(ade9000.SPI_Read_32(ADDR_CVRMS1012)-vrms_os)*vrms_cal;
      cirmsaccum =    cirmsaccum+(ade9000.SPI_Read_32(ADDR_CIRMS1012)-irms_os)*irms_cal;
      cwattaccum =    cwattaccum+ade9000.twos_compliment(ade9000.SPI_Read_32(ADDR_CWATT)) * -watt_cal;
      cvaaccum =      cvaaccum+ade9000.twos_compliment(ade9000.SPI_Read_32(ADDR_CVA)) * -watt_cal;

    //*********ACQUIRE DATA IN INTERVALS******************  
  current_time = (float)millis() / 1000.0;    //FIND CURRENT TIME FOR TIMING OF ACQUIRE
  float delta_time = current_time - last_time;
  if(delta_time>acquire_time)
  {
      //*********ACQUIRE AC POWER DATA PHASE A******************
      double avrms,airms,awatt,apf,avar,ava;
      avrms = avrmsaccum/dccount;
      airms = airmsaccum/dccount;
      awatt = awattaccum/dccount;
      ava = avaaccum/dccount;
      if(ava !=0) apf = awatt/ava;
      avar = sqrt(ava*ava - awatt*awatt);

      //*********ACQUIRE AC POWER DATA PHASE B******************
      double bvrms,birms,bwatt,bpf,bvar,bva;
      bvrms =     bvrmsaccum/dccount;
      birms =     birmsaccum/dccount;
      bwatt =     bwattaccum/dccount;
      bva =       bvaaccum/dccount;
      if(bva !=0) bpf = bwatt/bva;

      //*********ACQUIRE AC POWER DATA PHASE C******************
      double cvrms,cirms,cwatt,cpf,cvar,cva;
      cvrms =     cvrmsaccum/dccount;
      cirms =     cirmsaccum/dccount;
      cwatt =     cwattaccum/dccount;
      cva =       cvaaccum/dccount;
      if(cva !=0) cpf = cwatt/cva;
      cvar = sqrt(cva*cva - cwatt*cwatt);
  
        //**************SEND SERIAL DATA**************************
      String senddata = "{\"awatt\":";
      senddata += awatt;
      senddata += ",\"ava\":";
      senddata += ava;
      senddata += ",\"apf\":";
      senddata += apf;
      senddata += ",\"avrms\":";
      senddata += avrms;
      senddata += ",\"airms\":";
      senddata += airms;
      senddata += ",\"bwatt\":";
      senddata += bwatt;
      senddata += ",\"bva\":";
      senddata += bva;
      senddata += ",\"bpf\":";
      senddata += bpf;
      senddata += ",\"bvrms\":";
      senddata += bvrms;
      senddata += ",\"birms\":";
      senddata += birms;
      senddata += ",\"bwatt2\":";
      senddata += cwatt;
      senddata += ",\"cva\":";
      senddata += cva;
      senddata += ",\"cpf\":";
      senddata += cpf;
      senddata += ",\"cvrms\":";
      senddata += cvrms;
      senddata += ",\"cirms\":";
      senddata += cirms;
      senddata += "}";
      Serial.println(senddata);

    //*****************RESET ACCUMULATORS**************
      dccount = 1;
      last_time = current_time;

      avrmsaccum =    0;
      airmsaccum =    0;
      awattaccum =    0;
      avaaccum =      0;
      
      bvrmsaccum =    0;
      birmsaccum =    0;
      bwattaccum =    0;
      bvaaccum =      0;

      cvrmsaccum =    0;
      cirmsaccum =    0;
      cwattaccum =    0;
      cvaaccum =      0;
  }
  dccount++;    //advance counter for accumulator reconciliation
}




void init_ADE_regs()
{
  delay(1000);
  ade9000.SPI_Init(SPI_SPEED,CS_PIN); 
  ade9000.SPI_Write_16(ADDR_RUN,0x0000); 
  ade9000.SPI_Write_32(ADDR_AVGAIN,0x00000002); 
  ade9000.SPI_Write_32(ADDR_BVGAIN,0x00000002); 
  ade9000.SPI_Write_32(ADDR_CVGAIN,0x00000002); 
  ade9000.SPI_Write_32(ADDR_AIGAIN,0x00000002); 
  ade9000.SPI_Write_32(ADDR_BIGAIN,0x00000002); 
  ade9000.SPI_Write_32(ADDR_CIGAIN,0x00000002);
  ade9000.SPI_Write_32(ADDR_APGAIN,0x00000002); 
  ade9000.SPI_Write_32(ADDR_BPGAIN,0x00000002); 
  ade9000.SPI_Write_32(ADDR_CPGAIN,0x00000002); 
  //ade9000.SPI_Write_32(ADDR_AWATTOS,-14831); 
  ade9000.SPI_Write_32(ADDR_ACCMODE,0x0100); 
  delay(500);
  ade9000.SPI_Write_16(ADDR_RUN,VAL_RUN);
}



