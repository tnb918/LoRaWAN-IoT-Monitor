#include <string.h>
#include "app.h"
#include "usart.h"
#include "gpio.h"
#include "lorawan_node_driver.h"
#include "hdc1000.h"
#include "sensors_test.h"
#include "ST7789v.h"
#include "XPT2046.h"
#include "MPL3115.h"
#include "opt3001.h"

extern DEVICE_MODE_T device_mode;
extern DEVICE_MODE_T *Device_Mode_str;
extern Pen_Holder Pen_Point;	//�ʸ˽ṹ��

down_list_t *pphead = NULL; 	//������������

#define TEMP_LIMIT 30					//�¶�����
#define HUMI_LIMIT 60					//ʪ������
#define LUX_LIMIT  300				//��ǿ����
#define PRES_LIMIT 101				//��ѹ����

uint32_t tick = 0;			
uint32_t guitick = 0;
 
uint8_t n = 0;
float temp[100],humi[100],lux[100],pres[100];	//��Ųɼ����Ĵ���������
float temp_ave, humi_ave, lux_ave, pres_ave;	//�����Ĵ���������
char upbuf[50];
char rcbuf[50];
bool isConnect = false;				//�Ƿ�����

uint8_t comStatus;						//ͨ��״̬
uint8_t uptimes = 0;					//�ϴ�����
uint8_t buff[9];							//�ϴ����ݸ�ʽ
uint8_t pageidx = 0;					//������ҳ������
uint8_t paraidx = 0;					//��������������
uint8_t operidx = 0xff;				//��������������

uint32_t INTERVAL = 3000;			// ��ʱ�ϱ�ʱ��ms
uint32_t ALL_UP_TIMES = 10;		//ÿ���ϱ������Ĵ���
uint32_t SENSOR_TYPE = 1;			//����������
char guidatabuf[50];

//-----------------Users application--------------------------
void LoRaWAN_Func_Process(void)
{
    static DEVICE_MODE_T dev_stat = NO_MODE;

    uint16_t temper = 0;

    switch((uint8_t)device_mode)
    {
    /* ָ��ģʽ */
    case CMD_CONFIG_MODE:
    {
        /* �������command Configuration function, �����if���,ִֻ��һ�� */
        if(dev_stat != CMD_CONFIG_MODE)
        {
            dev_stat = CMD_CONFIG_MODE;
            debug_printf("\r\n[Command Mode]\r\n");

            nodeGpioConfig(wake, wakeup);
            nodeGpioConfig(mode, command);
        }
        /* �ȴ�usart2�����ж� */
        if(UART_TO_PC_RECEIVE_FLAG)
        {
            UART_TO_PC_RECEIVE_FLAG = 0;
            lpusart1_send_data(UART_TO_PC_RECEIVE_BUFFER,UART_TO_PC_RECEIVE_LENGTH);
        }
        /* �ȴ�lpuart1�����ж� */
        if(UART_TO_LRM_RECEIVE_FLAG)
        {
            UART_TO_LRM_RECEIVE_FLAG = 0;
            usart2_send_data(UART_TO_LRM_RECEIVE_BUFFER,UART_TO_LRM_RECEIVE_LENGTH);
        }
    }
    break;

    /* ͸��ģʽ */
    case DATA_TRANSPORT_MODE:
    {
        /* �������data transport function,�����if���,ִֻ��һ�� */
        if(dev_stat != DATA_TRANSPORT_MODE)
        {
            dev_stat = DATA_TRANSPORT_MODE;
            debug_printf("\r\n[Transperant Mode]\r\n");

            /* ģ�������ж� */
            if(nodeJoinNet(JOIN_TIME_120_SEC) == false)
            {
                return;
            }

            temper = HDC1000_Read_Temper() / 1000;

            nodeDataCommunicate((uint8_t*)&temper,sizeof(temper),&pphead);
        }

        /* �ȴ�usart2�����ж� */
        if(UART_TO_PC_RECEIVE_FLAG && GET_BUSY_LEVEL)  //Ensure BUSY is high before sending data
        {
            UART_TO_PC_RECEIVE_FLAG = 0;
            nodeDataCommunicate((uint8_t*)UART_TO_PC_RECEIVE_BUFFER, UART_TO_PC_RECEIVE_LENGTH, &pphead);
        }

        /* ���ģ����æ, ����������Ч��������������Ϣ */
        else if(UART_TO_PC_RECEIVE_FLAG && (GET_BUSY_LEVEL == 0))
        {
            UART_TO_PC_RECEIVE_FLAG = 0;
            debug_printf("--> Warning: Don't send data now! Module is busy!\r\n");
        }

        /* �ȴ�lpuart1�����ж� */
        if(UART_TO_LRM_RECEIVE_FLAG)
        {
            UART_TO_LRM_RECEIVE_FLAG = 0;
            usart2_send_data(UART_TO_LRM_RECEIVE_BUFFER,UART_TO_LRM_RECEIVE_LENGTH);
        }
    }
    break;

    /*����ģʽ*/
    case PRO_TRAINING_MODE:
    {
        /* �������Class C��ƽ̨���ݲɼ�ģʽ, �����if���,ִֻ��һ�� */
        if(dev_stat != PRO_TRAINING_MODE)
        {
            dev_stat = PRO_TRAINING_MODE;
            debug_printf("\r\n[Project Mode]\r\n");
        }
				
				/* ���ʵ�����λ�� */
				if(!isConnect)
				{
					nodeCmdConfig("AT+DEVEUI=74F898528DB33D8D,D391010220102816,1");
					nodeCmdConfig("AT+APPEUI=A87D98AC9322E74A");
					nodeCmdConfig("AT+APPKEY=1719E909B4A391C38E582E147733DEA1,0");
					nodeCmdConfig("AT+CLASS=0");
					nodeCmdConfig("AT+CONFIRM=1");
					nodeCmdConfig("AT+DEBUG=0");
					nodeCmdConfig("AT+SAVE");
					if(nodeJoinNet(120))
					{
						isConnect = true;
					}					
				}

				if(GET_SYSTEM_TIME >= tick +INTERVAL)
				{
					temp[n] = HDC1000_Read_Temper() / 1000.0;
					humi[n] = HDC1000_Read_Humidi() / 1000.0;
					lux[n] = OPT3001_Get_Lux();
					pres[n] = MPL3115_ReadPressure() / 1000.0;	
					tick = GET_SYSTEM_TIME;			

					if(++n > ALL_UP_TIMES - 1)
					{
						
						temp_ave = data_process(temp);
						humi_ave = data_process(humi);
						lux_ave = data_process(lux);
						pres_ave = data_process(pres);	
						debug_printf("temp:%.3f�棬humidi:%.3f%%, lux:%.2f Lux��pres:%.2f kPa\r\n",temp_ave,humi_ave,lux_ave,pres_ave);	

						if(lux_ave > LUX_LIMIT)
						{
							LEDWAKE_ON;
							LCD_Fill(55,250,120,275,BLACK);
							debug_printf("Lux Warning! The current lux:%.2f Lux\r\n",lux_ave);
							LCD_ShowString(55,250,"WARNING",RED);						
						}
						else
						{
							LEDWAKE_OFF;
							LCD_Fill(55,250,120,275,BLACK);
							LCD_ShowString(55,250,"NORMAL",WHITE);	
						}		
					
						if(pres_ave > PRES_LIMIT)
						{
							LEDMODE_ON;
							LCD_Fill(165,250,240,275,BLACK);
							debug_printf("Pressure Warning! The current pressure:%.2f kPa\r\n",pres_ave);
							LCD_ShowString(165,250,"WARNING",RED);						
						}
						else
						{
							LEDMODE_OFF;
							LCD_Fill(165,250,240,275,BLACK);
							LCD_ShowString(165,250,"NORMAL",WHITE);	
						}	
						
						if(temp_ave > TEMP_LIMIT)
						{
							LEDBUSY_ON;
							LCD_Fill(55,275,120,300,BLACK);
							debug_printf("Temperature Warning! The current temperature:%.3f��\r\n",temp_ave);
							LCD_ShowString(55,275,"WARNING",RED);
						}
						else
						{
							LEDBUSY_OFF;
							LCD_Fill(55,275,120,300,BLACK);
							LCD_ShowString(55,275,"NORMAL",WHITE);	
						}
								
						if(humi_ave > HUMI_LIMIT)
						{
							LEDSTAT_ON;
							LCD_Fill(165,275,240,300,BLACK);
							debug_printf("Humidity Warning! The current humidity:%.3f%%\r\n",humi_ave);	
							LCD_ShowString(165,275,"WARNING",RED);					
						}
						else
						{
							LEDSTAT_OFF;
							LCD_Fill(165,275,240,300,BLACK);
							LCD_ShowString(165,275,"NORMAL",WHITE);
						}	
						
						switch(SENSOR_TYPE)
						{
							case 1:
								comStatus = data_UP(0x01,uptimes,lux_ave);
								break;
							case 2:
								comStatus = data_UP(0x02,uptimes,pres_ave);
								break;							
							case 3:
								comStatus = data_UP(0x03,uptimes,temp_ave);
								break;
							case 4:
								comStatus = data_UP(0x04,uptimes,humi_ave);
								break;			
							default:break;
						}
						
						if(++uptimes > 0xff)	
							uptimes = 0;
						n = 0;						
					}
				}


				if(Pen_Point.Key_Sta == 1)
				{
					Pen_Point.Key_Sta = 0;
					debug_printf("X:%d Y:%d\n",Pen_Point.X,Pen_Point.Y);
					if(Pen_Point.X < 900 && Pen_Point.Y > 1600)//���Ͻ�ͨ����ʾ
					{
						LCD_Fill(0,0,240,225,BLACK);
						LCD_Fill(5,18,85,40,WHITE);
						pageidx = 0;
					}
					else if(Pen_Point.X > 900 && Pen_Point.Y > 1600)//���Ͻǲ�������
					{
						LCD_Fill(0,0,240,225,BLACK);
						LCD_Fill(115,18,195,40,WHITE);
						LCD_Fill(5,90,85,112,WHITE);
						pageidx = 1;paraidx = 0;
					}
					else if(Pen_Point.X > 0 && Pen_Point.X < 700 && Pen_Point.Y > 1400 && Pen_Point.Y < 1500)//INTERVAL
					{
						paraidx = 0;
						LCD_Fill(5,90,85,112,WHITE);
						LCD_Fill(5,126,85,148,BLACK);
						LCD_Fill(5,162,105,184,BLACK);
					}
					else if(Pen_Point.X > 0 && Pen_Point.X < 700 && Pen_Point.Y > 1200 && Pen_Point.Y < 1400)//UP_TIMES
					{
						paraidx = 1;
						LCD_Fill(5,90,85,112,BLACK);
						LCD_Fill(5,126,85,148,WHITE);
						LCD_Fill(5,162,105,184,BLACK);						
					}
					else if(Pen_Point.X > 0 && Pen_Point.X < 700 && Pen_Point.Y > 1000 && Pen_Point.Y < 1200)//SENSOR_TYPR
					{	
						paraidx = 2;
						LCD_Fill(5,90,85,112,BLACK);
						LCD_Fill(5,126,85,148,BLACK);
						LCD_Fill(5,162,105,184,WHITE);						
					}
					else if(Pen_Point.X > 0 && Pen_Point.X < 1000 && Pen_Point.Y < 1000)//ADD  && Pen_Point.Y > 800 
					{
						operidx = 0;
						LCD_Fill(5,198,85,220,WHITE);
						HAL_Delay(10);
						LCD_Fill(5,198,85,220,BLACK);
					}
					else if(Pen_Point.X > 1000 && Pen_Point.X < 1600 && Pen_Point.Y < 1000)//SUD  && Pen_Point.Y > 800 
					{
						operidx = 1;
						LCD_Fill(90,198,170,220,WHITE);
						HAL_Delay(10);
						LCD_Fill(90,198,170,220,BLACK);
					}
					else if(Pen_Point.X > 1600 && Pen_Point.Y < 1000)//SEND && Pen_Point.Y > 800 
					{
						operidx = 2;
						LCD_Fill(180,198,240,220,WHITE);
						HAL_Delay(10);
						LCD_Fill(180,198,240,220,BLACK);
					}
				}

				if(GET_SYSTEM_TIME >= guitick + 500 || n > ALL_UP_TIMES - 1)
				{
					guitick = GET_SYSTEM_TIME;
					if(pageidx == 0)
					{
						LCD_Fill(5,18,85,40,WHITE);
						LCD_ShowString(10,20,"COMM_PAGE",BLUE);	
						LCD_ShowString(120,20,"PARA_PAGE",BLUE);
						LCD_ShowString(10,56,"CURRENT_SENSOR_TYPE:",BLUE);
						LCD_Fill(180,56,240,72,BLACK);
						LCD_ShowString(180,56,toCharS(SENSOR_TYPE,1,0),BLUE);
						LCD_ShowString(10,92,"DevEui:",BLUE);
						LCD_ShowString(80,92,"74F898528DB33D8D",BLUE);
						LCD_ShowString(10,128,"Status:",BLUE);
						LCD_Fill(80,128,240,144,BLACK);
						comStatue_test(comStatus);
						LCD_ShowString(10,164,"UPdata:",BLUE);
						LCD_Fill(80,164,240,180,BLACK);
						LCD_ShowString(80,164,toChar(buff,9,1),BLUE);
						LCD_ShowString(10,200,"DNdata:",BLUE);
						LCD_Fill(80,200,240,216,BLACK);						
						if(pphead == NULL)
							LCD_ShowString(80,200,"NULL",BLUE);
						else
							LCD_ShowString(80,200,rcbuf,BLUE);							
					}
					else if(pageidx == 1)
					{				
						if(operidx == 0)
						{
							switch(paraidx)
							{
								case 0:
								{
									INTERVAL += 100;
									if(INTERVAL > 5000)
										INTERVAL = 3000;
								}break;
								case 1:
								{
									ALL_UP_TIMES ++;
									if(ALL_UP_TIMES > 30)
										ALL_UP_TIMES = 10;
								}break;
								case 2:
								{
									SENSOR_TYPE < 4 ? SENSOR_TYPE++ : (SENSOR_TYPE = 1);
								}break;
								default:break;
							}
						}
						else if(operidx == 1)
						{
							switch(paraidx)
							{
								case 0:
								{
									INTERVAL -= 100;
									if(INTERVAL < 100)
										INTERVAL = 3000;
								}break;
								case 1:
								{
									ALL_UP_TIMES --;
									if(ALL_UP_TIMES < 1)
										ALL_UP_TIMES = 10;
								}break;
								case 2:
								{
									SENSOR_TYPE > 1 ? SENSOR_TYPE-- : (SENSOR_TYPE = 4);
								}break;
								default:break;
							}
						}
						else if(operidx == 2)
						{
		
							switch(SENSOR_TYPE)
							{
								case 1:
									comStatus = data_UP(0x01,uptimes,lux_ave);
									break;
								case 2:
									comStatus = data_UP(0x02,uptimes,pres_ave);
									break;							
								case 3:
									comStatus = data_UP(0x03,uptimes,temp_ave);
									break;
								case 4:
									comStatus = data_UP(0x04,uptimes,humi_ave);
									break;			
								default:break;
							}							
						}
											
						LCD_ShowString(10,20,"COMM_PAGE",BLUE);	
						LCD_ShowString(120,20,"PARA_PAGE",BLUE);
	
						LCD_ShowString(10,56,"CURRENT_SENSOR_TYPE:",BLUE);
						LCD_Fill(180,56,240,72,BLACK);
						LCD_ShowString(180,56,toCharS(SENSOR_TYPE,1,0),BLUE);
						
						LCD_ShowString(10,92,"INTERVAL:",BLUE);
						LCD_Fill(100,92,240,108,BLACK);
						LCD_ShowString(100,92,toCharS(INTERVAL,0,1),BLUE);
						
						LCD_ShowString(10,128,"UP_TIMES:",BLUE);
						LCD_Fill(100,128,240,144,BLACK);
						LCD_ShowString(100,128,toCharS(ALL_UP_TIMES,0,0),BLUE);
						
						LCD_ShowString(10,164,"SENSOR_TYPE:",BLUE);
						LCD_Fill(120,164,240,180,BLACK);
						LCD_ShowString(120,164,toCharS(SENSOR_TYPE,1,1),BLUE);
						
						LCD_ShowString(10,200,"ADD",BLUE);
						LCD_ShowString(96,200,"SUB",BLUE);
						LCD_ShowString(182,200,"SEND",BLUE);
						
						operidx = 0xff;						
					}									
				}
							
				//debug_printf("INTERVAL=%d  ALL_UP_TIMES=%d  SENSOR_TYPE=%d\r\n",INTERVAL,ALL_UP_TIMES,SENSOR_TYPE);
									
				LCD_ShowString(10,225,"Information:",WHITE);						
				LCD_ShowString(10,250," LUX:",WHITE);
				LCD_ShowString(120,250,"PRES:",WHITE);
				LCD_ShowString(10,275,"TEMP:",WHITE);
				LCD_ShowString(120,275,"HUMI:",WHITE);
				LCD_ShowString(90,300,"Design By 18205223",WHITE);
												
    }
    break;

    default:
        break;
    }
}


/**
 * @brief   ������汾��Ϣ�Ͱ���ʹ��˵����Ϣ��ӡ
 * @details �ϵ����еƻ������100ms
 * @param   ��
 * @return  ��
 */
void LoRaWAN_Borad_Info_Print(void)
{
    debug_printf("\r\n\r\n");
    PRINT_CODE_VERSION_INFO("%s",CODE_VERSION);
    debug_printf("\r\n");
    debug_printf("--> Press Key1 to: \r\n");
    debug_printf("-->  - Enter command Mode\r\n");
    debug_printf("-->  - Enter Transparent Mode\r\n");
    debug_printf("--> Press Key2 to: \r\n");
    debug_printf("-->  - Enter Project Trainning Mode\r\n");
    LEDALL_ON;
    HAL_Delay(100);
    LEDALL_OFF;
}


float data_process(float* a)
{
	float max = a[0];//�����һ��Ϊ���ֵ
	float min = a[0];//�����һ��Ϊ��Сֵ
	float result = 0;
	
	float average = 0;
	for (int i = 0; i < ALL_UP_TIMES; i++) 	//����
	{
		result += a[i];
		if (a[i] > max)
			max = a[i];
		if (a[i] < min)
			min = a[i];
	}
	result = result - max - min;
	average = result / (ALL_UP_TIMES - 2);	//ע����(ALL_UP_TIMES - 2)
	return average;
	
}

char* toChar(uint8_t* a, int len, int type)
{
	char buf[50];
	memset(upbuf,'\0',sizeof(upbuf));
  for(int i = 0;i < len;i++)
  {
		if(type == 0)//ʮ����
			sprintf(buf,"%d",a[i]);//ÿ������Ԫ��ѭ��ת��Ϊ�ַ���
		else if(type == 1)//ʮ������
			 sprintf(buf,"%02x",a[i]);//ÿ������Ԫ��ѭ��ת��Ϊ�ַ���
		strcat(upbuf,buf);
  }
	lower2upper_and_remove_spaces(upbuf,upbuf);
	return upbuf;
}

char* toCharS(uint32_t a,int type,int isAddU)
{
	char buf[20];
	memset(guidatabuf,'\0',sizeof(guidatabuf));
  if(type == 0)//ʮ����
		sprintf(buf,"%d",a);//ÿ������Ԫ��ѭ��ת��Ϊ�ַ���
	else if(type == 1)//ʮ������
			sprintf(buf,"%02x",a);//ÿ������Ԫ��ѭ��ת��Ϊ�ַ���
	strcat(guidatabuf,buf);
	if(isAddU == 1)
	{
		if(type == 0)
			strcat(guidatabuf,"ms");
		if(type == 1)
		{
			switch(a)
			{
				case 1:
					strcat(guidatabuf,"-LUX");break;
				case 2:
					strcat(guidatabuf,"-PRESSURE");break;	
				case 3:
					strcat(guidatabuf,"-TEMPERATURE");break;
				case 4:
					strcat(guidatabuf,"-HUMIDITY");break;				
				default:break;				
			}			
		}

	}
	return guidatabuf;	
	
}
uint8_t data_UP(uint8_t sensor_type,uint8_t times,float data)
{
	uint8_t flag;
	uint16_t data_integer = 0;
	uint8_t data_decimal = 0;
	buff[0]=0xAA;//�̶�ֵ
	buff[1]=0x3D;//deveuiǰ��λ
	buff[2]=0x8D;//deveui����λ
	buff[3]=sensor_type;//����������
	buff[4]=times;//���ʹ���
	data_integer = data;
	data_decimal = (data-data_integer)*100;
	buff[5]=data_integer/256;//��������������>255
	buff[6]=data_integer%256;//��������������<255
	buff[7]=data_decimal;//С��
	buff[8]=0x0F;//�̶�ֵ
	
	flag = nodeDataCommunicate(buff,9,&pphead);	
				
//	LCD_Fill(110,300,240,320,BLACK);
//	LCD_ShowString(110,300,"Sending Data ...",RED);

//	HAL_Delay(500);
	if(pphead != NULL)
	{
		uint8_t recv_len = pphead->down_info.size;
		uint8_t ex[100];		
		debug_printf("recv_len = %d\n",recv_len);
		for(int i = 0;i < recv_len;i++)
		{
			ex[i] = pphead->down_info.business_data[i];
			debug_printf("%02x ",ex[i]);
		}
		strcpy(rcbuf,toChar(ex,recv_len,1));
	}
	
//	LCD_Fill(110,300,240,320,BLACK);
//	LCD_ShowString(110,300,"No Operations",WHITE);
	return flag;
}

void comStatue_test(uint8_t comStatue_type)
{	
	switch(comStatue_type)
	{
		case 0x01:  
			LCD_ShowString(80,128,"COMM_SUCCESS",BLUE);						///< ͨ�ųɹ�
			break;
		case 0x02:
			LCD_ShowString(80,128,"NOT_JOINED",BLUE);					///< ģ��δ����
			break;
		case 0x04:
			LCD_ShowString(80,128,"COMM_NO_ACK",BLUE);					///< ȷ��֡δ�յ�ACK
			break;
		case 0x08:
			LCD_ShowString(80,128,"BUSY_BFE_RECV_UDATA",BLUE);	///< ģ�鵱ǰ����æ״̬
			break;
		case 0x10:
			LCD_ShowString(80,128,"BUSY_ATR_COMM",BLUE);				///< ģ�鴦���쳣״̬
			break;
		case 0x20:
			LCD_ShowString(80,128,"IDLE_ATR_RECV_UDATA",BLUE);	///< ģ�鴮������Ӧ
			break;
		case 0x40:
			LCD_ShowString(80,128,"DATA_SIZE_WRONG",BLUE);			///< ���ݰ����ȴ���
			break;		
		default:
			LCD_ShowString(80,128,"NULL ",BLUE);	
			break;
	}
}