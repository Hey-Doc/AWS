/* MIT License
Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 */

#include <aws_customdemo_inc.h>
#include "st_sensordata_collector.h"

#include "main.h"
#include "stm32l4xx_hal_conf.h"
#include "stm32l475e_iot01_accelero.h"
#include "stm32l475e_iot01_psensor.h"
#include "stm32l475e_iot01_gyro.h"
#include "stm32l475e_iot01_hsensor.h"
#include "stm32l475e_iot01_tsensor.h"
#include "stm32l475e_iot01_magneto.h"
#include "vl53l0x_api.h"
#include "math.h"
#define buflen 100
#define SAMPLE_NUMBER 100

#define TEMPERATURE_TASK_READ_DELAY_MS  100

typedef enum {
	LONG_RANGE 		= 0, /*!< Long range mode */
	HIGH_SPEED 		= 1, /*!< High speed mode */
	HIGH_ACCURACY	= 2, /*!< High accuracy mode */
} RangingConfig_e;

extern I2C_HandleTypeDef hI2cHandler;

VL53L0X_Dev_t VL53L0XDev;
VL53L0X_RangingMeasurementData_t RangingMeasurementData;
int LeakyFactorFix8 = (int)( 0.6 *256);

int16_t axyz[3]={};
float Axyz[3]={};
float buffer_a[buflen][3]={};
float V[3]={0,0,0};
float pastAxyz[3]={0,0,0};
float magV[10]={};
float magM[10]={};
float meanV=0.0;
float stdV=0.0;
float meanM=0.0;
float stdM=0.0;
float sizV=0.0;
int cyc = 0;
int CycTrue = 0;
int32_t ADC_data[2]={0,0};
int32_t emg_prev;
int32_t emg;
int32_t f[100] = {0, 0, 0, 0, 0};
int32_t g[100] = {0, 0, 0, 0, 0};
int32_t i = 5;
int muscle_work = 1;
int heart_work = 1;
uint16_t heartrate[1000];
int32_t heart_cnt = 0;
uint16_t value[SAMPLE_NUMBER] = {0};
uint8_t valueCount_=255;
uint32_t nowTim=0,lastTim=0;
char msg[100];
int falling=0;
int heart_rate=0;
char hrstate;
extern hadc1;

float magnitude(float *vec)
{
  return sqrt(pow(vec[0],2) + pow(vec[1],2) + pow(vec[2],2));
}
float mean(float *vec)
{
  float sum=0.0;
  for (int i=0;i<10;i++){sum+=vec[i];}
  return sum/10;
}
float stnd(float *vec)
{
  float s=0;
  float m=mean(vec);
  for (int i=0;i<10;i++){s+=pow((vec[i]-m),2);}
  return sqrt(s/10);
}
void detect(float stdv, float meanv, float stdm, float meanm)
{
	if(meanv<=-0.131){
		if(stdm<=0.055){
			falling=0;
		}
		else{
			falling=1;
		}
	}
	else{
		if(meanm<=1.11){
			falling=1;
		}
		else{
			falling=0;
		}
	}
}
int msh(void){
	  emg_prev = emg;
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 10);
	  ADC_data[0] = HAL_ADC_GetValue(&hadc1);
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 10);
	  ADC_data[1] = HAL_ADC_GetValue(&hadc1);
	  emg = ADC_data[0];
	  int32_t err = (emg - emg_prev > 0)?(emg - emg_prev):(emg_prev - emg);
	  if (err <= 20 * 4) {
	  	f[i] = f[i - 1] + 1;
	  }
	  else {
	  	f[i] = 0;
	  }
	  if (emg_prev + 10 * 4 > emg) {
		g[i] = g[i - 1] + 1;
	  }
	  else {
	  	g[i] = 0;
	  }
	  muscle_work = (f[i] >= 5 || g[i] >= 5)?1:0;
	  i++;
	  if (i == 100) {
	  	i = 5;
	  	for (int j = 0; j < 5; j++) {
	  		f[j] = f[j+95];
	  		g[j] = g[j+95];
	  	}
	  }
	  valueCount_++;
	  if (valueCount_ >= SAMPLE_NUMBER) valueCount_ = 0;
	  value[valueCount_] = ADC_data[1];

	  static uint8_t timeFlag;
	  static unsigned long sampleTime[10];
	  unsigned long valueTime_;
	  uint8_t count_;
	  if (valueCount_) {
	  count_ = valueCount_ - 1;
	  } else {
	  	count_ = SAMPLE_NUMBER - 1;
	  }
	  if((value[valueCount_] > 1000 * 4) && (value[count_] < 20 * 4)) {
	  	nowTim = HAL_GetTick();
	  	uint32_t difTime = nowTim - lastTim;
	  	lastTim = nowTim;
	  	if(difTime > 300 || difTime < 2000) {
	  		sampleTime[timeFlag++] = difTime;
	  		if (timeFlag > 9) timeFlag = 0;
	  		}
	  	    if (0 == sampleTime[9]) {
	  		//continue;
	  		sampleTime[9] = sampleTime[9];
	  	}

	  	uint32_t Arrange[10] = {0};
	  	for (int i = 0; i < 10; i++) {
	  		Arrange[i] = sampleTime[i];
	  	}
	  			uint32_t Arrange_=0;
	  			for(int i=9;i>0;i--){
	  				for(int j=0;j<i;j++){
	  					if(Arrange[j] > Arrange[j+1]){
	  						Arrange_ = Arrange[j];
	  						Arrange[j] = Arrange[j+1];
	  						Arrange[j+1] = Arrange_;
	  					}
	  				}
	  			}
	  			if((Arrange[7]-Arrange[3])>120){
	          //continue;
	  				Arrange[7] = Arrange[7];
	  			}

	  			Arrange_ = 0;
	  			for(int i=3;i<=7;i++){
	  				Arrange_ += Arrange[i];
	  			}

	  			valueTime_ = 300000/Arrange_;///<60*1000*(7-2)
	  			heartrate[heart_cnt] = (uint16_t)valueTime_;
	  			heart_cnt++;
	  			if (heart_cnt >= 1000) heart_cnt = 0;
	  			if ((uint16_t)valueTime_ > 100) {
	  				heart_work = 0;
	  				for (int i = 1; i <= 6; i++) {
	  					int k = heart_cnt - i;
	  					if (k < 0) k += 1000;
	  					if (heartrate[k] > 100) continue;
	  					else {
	  						heart_work = 1;
	  						break;
	  					}
	  				}
	  			}
	  			if ((uint16_t)valueTime_ >= 60 && (uint16_t)valueTime_ <= 100) heart_work =1;
	  		}
	  if(heart_work==1) hrstate="N";
	  else hrstate="F";
	  return heartrate[heart_cnt>0?(heart_cnt-1):999];
}


int BSP_Proximity_Init();
int SetupSingleShot(RangingConfig_e rangingConfig);
float BSP_Proximity_Read();
void Sensor_SetNewRange(VL53L0X_Dev_t *pDev, VL53L0X_RangingMeasurementData_t *pRange);

static void prvSensorsInit(void) {
  
  if (HSENSOR_OK != BSP_HSENSOR_Init()) {
    Error_Handler();
  }
  
  if (TSENSOR_OK != BSP_TSENSOR_Init()) {
    Error_Handler();
  }
  
  if (PSENSOR_OK != BSP_PSENSOR_Init()) {
    Error_Handler();
  }
  
  if (MAGNETO_OK != BSP_MAGNETO_Init()) {
    Error_Handler();
  }
  
  if (GYRO_OK != BSP_GYRO_Init()) {
    Error_Handler();
  }
  
  if (ACCELERO_OK != BSP_ACCELERO_Init()) {
    Error_Handler();
  }
  
  if (0 != BSP_Proximity_Init()) {
    Error_Handler();
  }
}

/* This task read the sensor data from the STM32 target and publishes to the telemetry queue
 *
 */
void onboardSensorReaderTask() {



	//IotMqttConnection_t mqttConnection = gpmqttConnection;

	//IotMqttPublishInfo_t publishInfo = IOT_MQTT_PUBLISH_INFO_INITIALIZER;
	//IotMqttCallbackInfo_t publishComplete = IOT_MQTT_CALLBACK_INFO_INITIALIZER;


	char *pSensorPayload;

//	intptr_t publishCount = 0;
	//IotMqttError_t publishStatus = IOT_MQTT_STATUS_PENDING;

	int j = 0;
	//float temperatureC;
	int16_t ACC_Value[3];
    float PROXIMITY_Value;

	int snprintfreturn = 0;

	gucSensorTopicName[31] = '\0';

	IotLogInfo("ENTRY : onboardSensorReaderTask ");
	prvSensorsInit();

	do {
		vTaskDelay(pdMS_TO_TICKS(TEMPERATURE_TASK_READ_DELAY_MS));

		while(!falling){
		//=============accelero=========================
		BSP_ACCELERO_AccGetXYZ(ACC_Value);
        PROXIMITY_Value = BSP_Proximity_Read();
                
		IotLogInfo("Accelerometer [X: %d] ", ACC_Value);

		pSensorPayload = pvPortMalloc(sizeof(char) * SENSOR_STATUS_MSG_BUF_LEN);

		  Axyz[0]=(float)19.62*ACC_Value[0]/16384;
		  Axyz[1]=(float)19.62*ACC_Value[1]/16384;
		  Axyz[2]=(float)19.62*ACC_Value[2]/16384;
		  pastAxyz[0]=buffer_a[cyc][0];
		  pastAxyz[1]=buffer_a[cyc][1];
		  pastAxyz[2]=buffer_a[cyc][2];
	      buffer_a[cyc][0]=Axyz[0];
	  	  buffer_a[cyc][1]=Axyz[1];
	  	  buffer_a[cyc][2]=Axyz[2];
		  magM[cyc%10]=magnitude(Axyz);
		  for (int i=0;i<3;i++){
		  	V[i]+=(Axyz[i]-pastAxyz[i])/buflen;
		  }
		  sizV=magnitude(V);
		  magV[cyc%10]=0;
		  for (int i=0;i<3;i++){
		  	magV[cyc%10]+=Axyz[i]*V[i]/sizV;
		  }
		  meanM=mean(magM);
		  stdM=stnd(magM);
		  meanV=mean(magV);
		  stdV=stnd(magV);

		  cyc++;
		  if(cyc==buflen && !CycTrue){CycTrue=1;}
		  cyc=cyc%buflen;
		  if(CycTrue){
		  	detect(stdV, meanV, stdM, meanM);
		  }
		  //===========emg&heart===========================
		  heart_rate = msh();
		  snprintfreturn = snprintf(pSensorPayload, SENSOR_STATUS_MSG_BUF_LEN, "falling: NO, heart: %c, heart rate: %d\n", hrstate, heart_rate);
			IotLogInfo(
					"Publishing sensor data as json string: %s of length [ %d]\n",
					pSensorPayload, snprintfreturn);

		   sendToTelemetryQueue(gucSensorTopicName, pSensorPayload,
					            snprintfreturn);
		  HAL_Delay(50);
		}
		falling =0;
		heart_rate = msh();
		/* Format data for transmission to AWS */
		if(muscle_work==1){
						snprintfreturn = snprintf(pSensorPayload, SENSOR_STATUS_MSG_BUF_LEN, "falling: NO, heart: %c, heart rate: %d\n", hrstate, heart_rate);
				  }
				  else{
						snprintfreturn = snprintf(pSensorPayload, SENSOR_STATUS_MSG_BUF_LEN, "falling: NO, heart: %c, heart rate: %d\n", hrstate, heart_rate);
				  }
		//HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
		//HAL_Delay(100);
		//HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);

		IotLogInfo(
				"Publishing sensor data as json string: %s of length [ %d]\n",
				pSensorPayload, snprintfreturn);

	   sendToTelemetryQueue(gucSensorTopicName, pSensorPayload,
				            snprintfreturn);

	} while (j++ < SENSOR_DATA_NUM_POLL_CYCLE);


}

int BSP_Proximity_Init()
{
  RangingConfig_e RangingConfig = LONG_RANGE;
  uint16_t Id;
  int status;
    
  VL53L0XDev.I2cHandle = &hI2cHandler;
  
  VL53L0X_Dev_t *pDev;
  pDev = &VL53L0XDev;
  pDev->I2cDevAddr = 0x52;
  pDev->Present = 0;
  HAL_Delay(2);
  
  /* Set VL53L0X API trace level */
  VL53L0X_trace_config(NULL, TRACE_MODULE_NONE, TRACE_LEVEL_NONE, TRACE_FUNCTION_NONE); // No Trace
  
  status = VL53L0X_WrByte(pDev, 0x88, 0x00);
  
  /* Try to read one register using default 0x52 address */
  status = VL53L0X_RdWord(pDev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
  
  if (status) {
    IotLogInfo("VL53L0X Read id fail\n");
  }
  
  if (Id == 0xEEAA) {
    status = VL53L0X_DataInit(pDev);
    if( status == 0 ){
      pDev->Present = 1;
      status = SetupSingleShot(RangingConfig);
    }
    else{
      IotLogInfo("VL53L0X_DataInit fail\n");
    }
  }
  else {
    IotLogInfo("unknown ID %x\n", Id);
    status = 1;
  }
  
  return status;
}

int SetupSingleShot(RangingConfig_e rangingConfig){

  int status;
  uint8_t VhvSettings;
  uint8_t PhaseCal;
  uint32_t refSpadCount;
  uint8_t isApertureSpads;
  FixPoint1616_t signalLimit = (FixPoint1616_t)(0.25*65536);
  FixPoint1616_t sigmaLimit = (FixPoint1616_t)(18*65536);
  uint32_t timingBudget = 33000;
  uint8_t preRangeVcselPeriod = 14;
  uint8_t finalRangeVcselPeriod = 10;
  
  status=VL53L0X_StaticInit(&VL53L0XDev);
  if( status ){
    IotLogInfo("VL53L0X_StaticInit failed\n");
  }
  
  status = VL53L0X_PerformRefCalibration(&VL53L0XDev, &VhvSettings, &PhaseCal);
  if( status ){
    IotLogInfo("VL53L0X_PerformRefCalibration failed\n");
  }
  
  status = VL53L0X_PerformRefSpadManagement(&VL53L0XDev, &refSpadCount, &isApertureSpads);
  if( status ){
    IotLogInfo("VL53L0X_PerformRefSpadManagement failed\n");
  }
  
  status = VL53L0X_SetDeviceMode(&VL53L0XDev, VL53L0X_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode
  if( status ){
    IotLogInfo("VL53L0X_SetDeviceMode failed\n");
  }
  
  status = VL53L0X_SetLimitCheckEnable(&VL53L0XDev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1); // Enable Sigma limit
  if( status ){
    IotLogInfo("VL53L0X_SetLimitCheckEnable failed\n");
  }
  
  status = VL53L0X_SetLimitCheckEnable(&VL53L0XDev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1); // Enable Signa limit
  if( status ){
    IotLogInfo("VL53L0X_SetLimitCheckEnable failed\n");
  }
  /* Ranging configuration */
  switch(rangingConfig) {
  case LONG_RANGE:
    signalLimit = (FixPoint1616_t)(0.1*65536);
    sigmaLimit = (FixPoint1616_t)(60*65536);
    timingBudget = 33000;
    preRangeVcselPeriod = 18;
    finalRangeVcselPeriod = 14;
    break;
  case HIGH_ACCURACY:
    signalLimit = (FixPoint1616_t)(0.25*65536);
    sigmaLimit = (FixPoint1616_t)(18*65536);
    timingBudget = 200000;
    preRangeVcselPeriod = 14;
    finalRangeVcselPeriod = 10;
    break;
  case HIGH_SPEED:
    signalLimit = (FixPoint1616_t)(0.25*65536);
    sigmaLimit = (FixPoint1616_t)(32*65536);
    timingBudget = 20000;
    preRangeVcselPeriod = 14;
    finalRangeVcselPeriod = 10;
    break;
  default:
    IotLogInfo("Not Supported");
  }
  
  status = VL53L0X_SetLimitCheckValue(&VL53L0XDev,  VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, signalLimit);
  if( status ){
    IotLogInfo("VL53L0X_SetLimitCheckValue failed\n");
  }
  
  status = VL53L0X_SetLimitCheckValue(&VL53L0XDev,  VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, sigmaLimit);
  if( status ){
    IotLogInfo("VL53L0X_SetLimitCheckValue failed\n");
  }
  
  status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&VL53L0XDev,  timingBudget);
  if( status ){
    IotLogInfo("VL53L0X_SetMeasurementTimingBudgetMicroSeconds failed\n");
  }
  
  status = VL53L0X_SetVcselPulsePeriod(&VL53L0XDev,  VL53L0X_VCSEL_PERIOD_PRE_RANGE, preRangeVcselPeriod);
  if( status ){
    IotLogInfo("VL53L0X_SetVcselPulsePeriod failed\n");
  }
  
  status = VL53L0X_SetVcselPulsePeriod(&VL53L0XDev,  VL53L0X_VCSEL_PERIOD_FINAL_RANGE, finalRangeVcselPeriod);
  if( status ){
    IotLogInfo("VL53L0X_SetVcselPulsePeriod failed\n");
  }
  
  status = VL53L0X_PerformRefCalibration(&VL53L0XDev, &VhvSettings, &PhaseCal);
  if( status ){
    IotLogInfo("VL53L0X_PerformRefCalibration failed\n");
  }
  
  VL53L0XDev.LeakyFirst=1;
  
  return status;
}


float BSP_Proximity_Read()
{
  int status;
  float value;
  status = VL53L0X_PerformSingleRangingMeasurement(&VL53L0XDev,&RangingMeasurementData);
  if( status ==0 ){
    /* Push data logging to UART */
    Sensor_SetNewRange(&VL53L0XDev,&RangingMeasurementData);
  }
  else{
    Error_Handler();
  }
  
  if( RangingMeasurementData.RangeStatus == 0 ){
    value = (float)VL53L0XDev.LeakyRange/10;
  }
  else{
    value = 0;
  }
  
  return value;
}

/* Store new ranging data into the device structure, apply leaky integrator if needed */
void Sensor_SetNewRange(VL53L0X_Dev_t *pDev, VL53L0X_RangingMeasurementData_t *pRange){
  if( pRange->RangeStatus == 0 ){
    if( pDev->LeakyFirst ){
      pDev->LeakyFirst = 0;
      pDev->LeakyRange = pRange->RangeMilliMeter;
    }
    else{
      pDev->LeakyRange = (pDev->LeakyRange*LeakyFactorFix8 + (256-LeakyFactorFix8)*pRange->RangeMilliMeter)>>8;
    }
  }
  else{
    pDev->LeakyFirst = 1;
  }
}
