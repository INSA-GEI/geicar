#include "application.h"
#include "cmsis_os.h"
#include "main.h"


osThreadId IMUHandle;
osThreadId GPSHandle;
osThreadId LIDARHandle;
osThreadId UARTHandle;
osThreadId BatterieHandle;
osThreadId CANHandle;
osThreadId I2CHandle;
osThreadId SPIHandle;

extern uint8_t rxBufferGps;
extern char nmeaFrame[200];
extern int nmeaFrameValid;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim8;

#define LIDAR_DATA_LENGTH 47
uint8_t data_buffer[LIDAR_DATA_LENGTH] = {0};

char rx_buffer[LINEMAX];   // Local holding buffer to build line
int rx_index = 0;

uint8_t buffer_ros[6] = {0,0,0,0,0,0};

void Tasks_Init(void)
{
	osThreadDef(UART, StartUart, osPriorityNormal, 0, 128);
	UARTHandle = osThreadCreate(osThread(UART), NULL);

	osThreadDef(IMU, StartIMU, osPriorityHigh, 0, 128);
	IMUHandle = osThreadCreate(osThread(IMU), NULL);

	osThreadDef(GPS, StartGPS, osPriorityAboveNormal, 0, 128);
	GPSHandle = osThreadCreate(osThread(GPS), NULL);

	//osThreadDef(LIDAR, StartLidar, osPriorityBelowNormal, 0, 64);
	//LIDARHandle = osThreadCreate(osThread(LIDAR), NULL);

	/*osThreadDef(Batterie, StartBatterie, osPriorityBelowNormal, 0, 64);
	BatterieHandle = osThreadCreate(osThread(Batterie), NULL);


	osThreadDef(CAN, StartCAN, osPriorityBelowNormal, 0, 64);
	CANHandle = osThreadCreate(osThread(CAN), NULL);

	osThreadDef(I2C, StartI2C, osPriorityBelowNormal, 0, 64);
	I2CHandle = osThreadCreate(osThread(I2C), NULL);

	osThreadDef(SPI, StartSPI, osPriorityBelowNormal, 0, 64);
	SPIHandle = osThreadCreate(osThread(SPI), NULL);*/
}

void Process_pwm_frame(uint8_t * frame)
{
	uint8_t pwm_num = frame [0];
	uint8_t channel_num = frame [1];
	uint32_t pwm_value = (uint32_t)frame[5]<<24 |
						 (uint32_t)frame[4]<<16 |
						 (uint32_t)frame[3]<<8 |
						 (uint32_t)frame[2] ; //passage de little endian a big endian

	TIM_HandleTypeDef htim;
	uint32_t channel;

	switch(pwm_num){
		case 3 :
			htim = htim3;
			break;
		case 8 :
			htim = htim8;
			break;
		default:
			return;
	}

	switch(channel_num){
		case 1:
			channel = TIM_CHANNEL_1;
			break;
		case 2:
			channel = TIM_CHANNEL_2;
			break;
		case 3:
			channel = TIM_CHANNEL_3;
			break;
		case 4:
			channel = TIM_CHANNEL_4;
			break;
		default:
			return;
	}

	if ((pwm_value >= 0x3f800000)&& (pwm_value < 0x3fc00000))
	{
	 	printf("reculer");
	}
	else if ((pwm_value >0x3fc00000)&& (pwm_value <=0x40000000))
	{
	    printf("avancer");
	}

	float pwm_value2 = *((float*)&pwm_value); //passer la valeur de uint32_t à float
	uint32_t ccr_value = (uint32_t)((pwm_value2/20.0)*2000); //ARR = 1999

	__HAL_TIM_SET_COMPARE(&htim,channel, ccr_value); //écrit directement dans le registre du timer pour changer la valeur PWM

}

void IMU_Receive_Transmit_Data()
{
	IMU_GetData();
}

void GetData_GPS(void)
{

	HAL_UART_Receive_IT(&huart2,&rxBufferGps,1);
}

void LIDAR_Receive_Transmit_Data(void){
	HAL_StatusTypeDef status = HAL_UART_Receive_IT(&huart3, data_buffer, LIDAR_DATA_LENGTH);

	if(status == HAL_OK)
	{
		// Vérifiez le CRC
		if(CalCRC8(data_buffer, LIDAR_DATA_LENGTH - 1) == data_buffer[LIDAR_DATA_LENGTH - 1])
		{
		   // Assigner les valeurs
		     LiDARFrameTypeDef frame = AssignValues(data_buffer);
		     MESSAGE_SendMailbox(Appli_Mailbox, MSG_ID_LIDAR, NULL,&frame);

		 }
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{

}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart -> Instance == USART2)
    {
//    	//Decoding NMEA frame
//
//			if ((rxBufferGps == '\r') || (rxBufferGps == '\n')) // Is this an end-of-line condition, either will suffice?
//			{
//			  if (rx_index != 0) // Line has some content
//			  {
//				memcpy((void *)nmeaFrame, rx_buffer, rx_index); // Copy to static line buffer from dynamic receive buffer
//				nmeaFrame[rx_index] = 0; // Add terminating NUL
//				nmeaFrameValid = 1; // flag new line valid for processing
//
//				rx_index = 0; // Reset content pointer
//			  }
//			}
//			else
//			{
//			  if ((rxBufferGps == '$') || (rx_index == LINEMAX)) // If resync or overflows pull back to start
//				rx_index = 0;
//
//			  rx_buffer[rx_index++] = rxBufferGps; // Copy to buffer and increment
//			}
//
//    	HAL_UART_Receive_IT(&huart2, &rxBufferGps, 1);
    }

    if (huart -> Instance == USART3)
    {

    }

    if(huart -> Instance == UART4)
    {
    	/*uint8_t tps =0;
    	for(int i=0; i<2; i++) //changement endian
    	{
    		tps = buffer_ros[i+2];
    		buffer_ros[i+2]= buffer_ros[5-i];
    		buffer_ros[5-i]= tps;
    	}*/

    	/*uint32_t test = (uint32_t)buffer_ros[5]<<24 |
    					(uint32_t)buffer_ros[4]<<16 |
						(uint32_t)buffer_ros[3]<<8 |
						(uint32_t)buffer_ros[2] ;*/

    	Process_pwm_frame(buffer_ros);

    	HAL_UART_Receive_IT(&huart4, buffer_ros, 6);
    }
}



void Transmit_data_to_usb(void)
{
	MESSAGE_Typedef message_appli;
	message_appli = MESSAGE_ReadMailboxNoDelay(Appli_Mailbox);
	//message_appli = MESSAGE_ReadMailbox(Appli_Mailbox);
	switch(message_appli.id){

	case MSG_ID_GPS :
		TransmitGPSFrame(message_appli.data);
		//HAL_UART_Transmit_IT(&huart4, message_appli.data, sizeof(message_appli.data));
		break;
	case MSG_ID_IMU :
		TransmitIMUFrame(message_appli.data);
		break;
	case MSG_ID_LIDAR :
		TransmitLiDARFrame(message_appli.data);
		//HAL_UART_Transmit_IT(&huart4, (uint8_t*)message_appli.data,30);
		break;
	default :
		break;
	}
}

void Receive_data_from_usb(void)
{
	HAL_UART_Receive_IT(&huart4, buffer_ros, 6);
}

void StartUart(void const * argument)
{
  /* USER CODE BEGIN 5 */
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = pdMS_TO_TICKS(100);

	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();
	//tache pour l'envoie de donnees via l'USB
	//utilisation mailbox pour envoyer les donnees
  /* Infinite loop */
  for(;;)
  {

	  Transmit_data_to_usb();
	  Receive_data_from_usb();
	  vTaskDelayUntil(&xLastWakeTime, xFrequency);
	  //osDelay(100);
  }
  /* USER CODE END 5 */
}

void StartIMU(void const * argument)
{
  /* USER CODE BEGIN 5 */
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = pdMS_TO_TICKS(500);

	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();

  /* Infinite loop */
  for(;;)
  {

	IMU_Receive_Transmit_Data();
	vTaskDelayUntil(&xLastWakeTime, xFrequency);
    //osDelay(400);
  }
  /* USER CODE END 5 */
}

void StartGPS(void const * argument)
{
  /* USER CODE BEGIN 5 */
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = pdMS_TO_TICKS(100);

	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
	  GetData_GPS();
	  task_update_gps();
	  task_send_values_GPS();
	  vTaskDelayUntil(&xLastWakeTime, xFrequency);
	  //osDelay(10);
  }
  /* USER CODE END 5 */
}


void StartSPI(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(300);
  }
  /* USER CODE END 5 */
}

void StartBatterie(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(800);
  }
  /* USER CODE END 5 */
}

void StartCAN(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(700);
  }
  /* USER CODE END 5 */
}

void StartLidar(void const * argument)
{
  /* USER CODE BEGIN 5 */
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = pdMS_TO_TICKS(100);

	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
	  LIDAR_Receive_Transmit_Data();

	vTaskDelayUntil(&xLastWakeTime, xFrequency);
    //osDelay(600);
  }
  /* USER CODE END 5 */
}
