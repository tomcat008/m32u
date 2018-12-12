#include "hostif.h"
#include "save_data.h"
#include "timer.h"
#include "delay.h"
#include "string.h"
#include "SIM800C.h"
#include "control.h"


u8 host_rx_buff[HOST_RX_BUFF_LEN];
u16 host_rx_len = 0;

u8 host_tx_buff[HOST_TX_BUFF_LEN];

u8 dtu_hw_version[] = "V1.2";
u8 dtu_sw_version[] = "V1.181";


APN_GATEWAT temp_apn_para;

u8 test_gsm(u8 cmd);


u8 assic_hex_tab[16] = {0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,0x41,0x42,0x43,0x44,0x45,0x46};

static u8 temp_str[130] = {0};
static int temp[5] = {0};

//sim900a���������,�����յ���Ӧ��
//str:�ڴ���Ӧ����
//����ֵ:0,û�еõ��ڴ���Ӧ����
//    ����,�ڴ�Ӧ������λ��(str��λ��)
//u8 SIM8XX_check_cmd(u8 *str)
//{
//		strx=strstr((const char*)USART_RX_BUF,(const char*)str);
//}
//ϵͳ����
u8 *hostif_cmd_tab[]=
{
	"WKMODE",	
	"GPRSMODE",
	"DBGMODE",
	"DTUID",
	"UART",
	"DFI",
	"HBTIME",
	"HBHEAD",
	"REGPKG",
	"REGHEAD",
	"SERVER1",
	"SERVER2",
	"IDLTIME",
	"APN",
	"HTTPS",
	"HTTPHD",
	"QRYPTM",
	"QRYPOST",
	"QRYPPA",
	"QRYGTM",
	"QRYGET",
	"QRYGPA",

	"SYSPWD",
	"BUFMODE",

	"WAKEUP",
	"OFFLINE",
	"VERSION",
	"LIST",
	"CSQ",
	"GSTATE",
	"DTUIP",
	"NET",
	"RESET",
	"RESTART",
	"CSMINS",
	"TEST_GSM",
};



u8 cmd_cmp(u8 *str1,u8 *str2)
{
	u8 res = 0;
	while(1)
	{
		if(*str2=='\0')
		{
		    if(*str1=='?')
		    {
		    	  DEBUG("query cmd\r\n");
		        res = 1;
		    	  break;
		    }
		    else if(*str1=='=')
		    {
		    	  DEBUG("set cmd\r\n");
		        res = 2;
		    	  break;
		    }
        else if(*str1=='#')
		    {
		    	  DEBUG("operation cmd\r\n");
		        res = 2;
		    	  break;
		    }
				else
				{
					  res = 3;
		    	  break;
				}
		}
		
		if(*str1!=*str2)
		{
			  res = 0;
		    break;
		}
		
		str1++;
		str2++;
	}
	return res;//�����ַ������
}

//��ѯ����
void quert_cmd_process(u8 cmd, u8 *para)
{
	  int res = 0, i = 0;
	  temp[0] = 0;
	  temp[1] = 0;
	  temp[2] = 0;
	  temp[3] = 0;
	  temp[4] = 0;
	  memset(temp_str, 0, 130);
	
    switch(cmd)
    {
    	  case HOST_CMD_WKMODE://����ģʽ
					  res = sprintf((char *)host_tx_buff,"+WKMODE:%d\r\n",para_value.word_mode);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
    		    break;	
				case HOST_CMD_GPRSMODE://����ģʽ
            res = sprintf((char *)host_tx_buff,"+GPRSMODE:%d\r\n",para_value.on_line_mode);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
    		    break;	
				case HOST_CMD_DBGMODE://����ģʽ
					  res = sprintf((char *)host_tx_buff,"+DBGMODE:%d\r\n",para_value.debug_mode);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
    		    break;	
				case HOST_CMD_DTUID://ID����
            res = sprintf((char *)host_tx_buff,"+DTUID:%s\r\n",para_value.id);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
    		    break;	
				case HOST_CMD_UART://���ڲ���
            res = sprintf((char *)host_tx_buff,"+UART:%d,%d,%d,%d,%d\r\n",para_value.usart_para.baurd, para_value.usart_para.data_bits, 
							             para_value.usart_para.parity, para_value.usart_para.stop ,para_value.usart_para.flow_ctr);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
    		    break;	
				case HOST_CMD_DFI://�������ݰ�ʱ����
            res = sprintf((char *)host_tx_buff,"+DFI:%d\r\n",para_value.usart_rx_interval);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
    		    break;	
				case HOST_CMD_HBTIME://����ʱ��
            res = sprintf((char *)host_tx_buff,"+HBTIME:%d\r\n",para_value.heartbeat_time);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
    		    break;	
				case HOST_CMD_HBHEAD://������ͷ
					  if(para_value.heartbeat_head_type == 0)
						{
						    if(para_value.heartbeat_head_len >0)
                {
								    res = sprintf((char *)host_tx_buff,"+HBHEAD:HEX%02X ",para_value.heartbeat_head[0]);
							      for(i=0; i<(para_value.heartbeat_head_len-1); i++)
							      {
								        res = res + sprintf((char *)(&host_tx_buff[14+i*3]),"%02X ",para_value.heartbeat_head[i+1]);
								    }
								}
                else
								{
								    res = sprintf((char *)host_tx_buff,"+HBHEAD: ");
								}									
							  
								host_tx_buff[res-1] = '\r';
								host_tx_buff[res] = '\n';
								res++;
						}
						else if(para_value.heartbeat_head_type == 1)
						{
						    res = sprintf((char *)host_tx_buff,"+HBHEAD:%s\r\n",para_value.heartbeat_head);
						}
            
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
    		    break;	
				case HOST_CMD_REGPKG://ע�������
					  res = sprintf((char *)host_tx_buff,"+REGPKG:%d\r\n",para_value.reg_packet_switch);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
    		    break;	
				case HOST_CMD_REGHEAD://ע���ͷ
					  if(para_value.reg_head_type == 0)
						{
							  if(para_value.reg_head_len >0)
								{
									  res = sprintf((char *)host_tx_buff,"+REGHEAD:HEX%02X ",para_value.reg_head[0]);
							      for(i=0; i<(para_value.reg_head_len-1); i++)
							      {
								        res = res + sprintf((char *)(&host_tx_buff[14+i*3]),"%02X ",para_value.reg_head[i+1]);
								    }
								}
								else
								{
								    res = sprintf((char *)host_tx_buff,"+REGHEAD: ");
								}
								host_tx_buff[res-1] = '\r';
								host_tx_buff[res] = '\n';
								res++;
								
						}
						else if(para_value.reg_head_type == 1)
						{
						    res = sprintf((char *)host_tx_buff,"+REGHEAD:%s\r\n",para_value.heartbeat_head);
						}
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
    		    break;	
				case HOST_CMD_SERVER1://��������1
            res = sprintf((char *)host_tx_buff,"+SERVER1:%d,%s,%d\r\n",para_value.link1.connect_type,
							                                       para_value.link1.ip_addr, para_value.link1.port);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
    		    break;	
				case HOST_CMD_SERVER2://��������2
            res = sprintf((char *)host_tx_buff,"+SERVER2:%d,%s,%d\r\n",para_value.link2.connect_type,
							                                       para_value.link2.ip_addr, para_value.link2.port);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
    		    break;	
				case HOST_CMD_IDLTIME://�Զ�����ʱ��
					  res = sprintf((char *)host_tx_buff,"+IDLTIME:%d\r\n",para_value.auto_off_line_time);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
    		    break;	
				case HOST_CMD_APN://GPRS APN����
					  res = sprintf((char *)host_tx_buff,"+APN:%s,%s,%s\r\n",para_value.apn_para.apn_name, 
							                      para_value.apn_para.user_name, para_value.apn_para.user_pwd);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
    		    break;	
				case HOST_CMD_HTTPS://HTTP����
					  res = sprintf((char *)host_tx_buff,"+HTTPS:%d,%d,%s\r\n", para_value.http_para.type,
							                    para_value.http_para.out_head, para_value.http_para.url);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);	
					  break;
		    case HOST_CMD_HTTPHD://HTTP ͷ����
					  if(res != -1)
						{
						    res = 1;
							  temp[0] = 0;
							  temp[1] = 0;
							  //������Ҫ�жϼ�ת�������Ȼsim800cʶ����
							  while(para_value.http_para.head[temp[1]] != 0)
							  {
									  if((para_value.http_para.head[temp[1]] == 0x5c) && (para_value.http_para.head[temp[1]+1] == 'r'))
										{
										    temp_str[temp[0]] = '\r';
											  temp[1]++;
										}
									  else if((para_value.http_para.head[temp[1]] == 0x5c) && (para_value.http_para.head[temp[1]+1] == 'n'))
										{
											  temp_str[temp[0]] = '\n';
											  temp[1]++;
										}
										else
										{
										    temp_str[temp[0]] = para_value.http_para.head[temp[1]];
										}
						        
										temp[0]++;
										temp[1]++;
								}
						}	
					  res = sprintf((char *)host_tx_buff,"+HTTPHD:%s\r\n", temp_str);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);	
					  break;
						
				case HOST_CMD_QRYPTM://��ѯPOST���ʱ��
					  res = sprintf((char *)host_tx_buff,"+QRYPTM:%d,%d\r\n", para_value.modbus_qry_post_time, para_value.modbus_qry_cmd_time);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);	
					  break;
				case HOST_CMD_QRYPOST://��ѯPOST
					  para = para + 7;
				    //�Ȱѵ�һ��������ȡ����
						while(*para != '?')
						{
						    temp[0] = temp[0]*10 + (*para++ - 0x30);
							  if(*para == '?')
								{
								    break;
								}
						}
						if(temp[0] < 16)
						{
						    res = sprintf((char *)host_tx_buff,"+QRYPOST:%d,", temp[0]);
								for(i=0; i<para_value.modbus_qry_post_para[temp[0]].len;i++)
								{
										host_tx_buff[res+i*2] = assic_hex_tab[para_value.modbus_qry_post_para[temp[0]].buff[i] >> 4];
										host_tx_buff[res+i*2+1] = assic_hex_tab[para_value.modbus_qry_post_para[temp[0]].buff[i] & 0x0f];
								}
								host_tx_buff[res+i*2] = '\r';
								host_tx_buff[res+i*2+1] = '\n';
								i++;
				        usart_send_str(HOSTIF_USART, host_tx_buff, (res+i*2));	
						}
					  break;
				case HOST_CMD_QRYPPA://������ѯpost http����
						res = sprintf((char *)host_tx_buff,"+QRYPPA:%s\r\n", para_value.modbus_http_post_para.url);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);	
						break;
				case HOST_CMD_QRYGTM://��ѯGET���ʱ��
					  res = sprintf((char *)host_tx_buff,"+QRYGTM:%d\r\n", para_value.modbus_qry_get_time);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);	
					  break;
				case HOST_CMD_QRYGET://��ѯGET
						res = sprintf((char *)host_tx_buff,"+QRYGET:");
						for(i=0; i<para_value.modbus_qry_get_para.len;i++)
						{
								host_tx_buff[res+i*2] = assic_hex_tab[para_value.modbus_qry_get_para.buff[i] >> 4];
								host_tx_buff[res+i*2+1] = assic_hex_tab[para_value.modbus_qry_get_para.buff[i] & 0x0f];
						}
						host_tx_buff[res+i*2] = '\r';
						host_tx_buff[res+i*2+1] = '\n';
						i++;
				    usart_send_str(HOSTIF_USART, host_tx_buff, (res+i*2));	
					  break;
				case HOST_CMD_QRYGPA://������ѯget http����
						res = sprintf((char *)host_tx_buff,"+QRYGPA:%s\r\n", para_value.modbus_http_get_para.url);
						usart_send_str(HOSTIF_USART, host_tx_buff, res);
				case HOST_CMD_BUFMODE://���ݻ���ģʽ
            
    		    break;	
				case HOST_CMD_VERSION://��ѯDTU�汾
					  res = sprintf((char *)host_tx_buff,"+VERSION:%s,%s\r\n",dtu_hw_version,dtu_sw_version);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
    		    break;	
				case HOST_CMD_LIST://��ѯDTU���Բ���
					  //
					  res = sprintf((char *)host_tx_buff,"+LIST:\r\n");
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
				    //����ģʽ
					  res = sprintf((char *)host_tx_buff,"+WKMODE:%d\r\n",para_value.word_mode);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
				    //����ģʽ
            res = sprintf((char *)host_tx_buff,"+GPRSMODE:%d\r\n",para_value.on_line_mode);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
				    //DTU ide��
            res = sprintf((char *)host_tx_buff,"+DTUID:%s\r\n",para_value.id);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
				    //�����������
            res = sprintf((char *)host_tx_buff,"+UART:%d,%d,%d,%d,%d\r\n",para_value.usart_para.baurd, para_value.usart_para.data_bits, 
							             para_value.usart_para.parity, para_value.usart_para.stop ,para_value.usart_para.flow_ctr);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
				    //�������ݰ�ʱ���� 
            res = sprintf((char *)host_tx_buff,"+DFI:%d\r\n",para_value.usart_rx_interval);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
				    //����ʱ��
            res = sprintf((char *)host_tx_buff,"+HBTIME:%d\r\n",para_value.heartbeat_time);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
				    //������ͷ
					  if(para_value.heartbeat_head_type == 0)
						{
						    if(para_value.heartbeat_head_len >0)
                {
								    res = sprintf((char *)host_tx_buff,"+HBHEAD:HEX%02X ",para_value.heartbeat_head[0]);
							      for(i=0; i<(para_value.heartbeat_head_len-1); i++)
							      {
								        res = res + sprintf((char *)(&host_tx_buff[14+i*3]),"%02X ",para_value.heartbeat_head[i+1]);
								    }
								}
                else
								{
								    res = sprintf((char *)host_tx_buff,"+HBHEAD: ");
								}									
							  
								host_tx_buff[res-1] = '\r';
								host_tx_buff[res] = '\n';
								res++;
						}
						else if(para_value.heartbeat_head_type == 1)
						{
						    res = sprintf((char *)host_tx_buff,"+HBHEAD:%s\r\n",para_value.heartbeat_head);
						}
            
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
				    //ע�������
					  res = sprintf((char *)host_tx_buff,"+REGPKG:%d\r\n",para_value.reg_packet_switch);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);	
				    //ע���ͷ
					  if(para_value.reg_head_type == 0)
						{
							  if(para_value.reg_head_len >0)
								{
									  res = sprintf((char *)host_tx_buff,"+REGHEAD:HEX%02X ",para_value.reg_head[0]);
							      for(i=0; i<(para_value.reg_head_len-1); i++)
							      {
								        res = res + sprintf((char *)(&host_tx_buff[14+i*3]),"%02X ",para_value.reg_head[i+1]);
								    }
								}
								else
								{
								    res = sprintf((char *)host_tx_buff,"+REGHEAD: ");
								}
								host_tx_buff[res-1] = '\r';
								host_tx_buff[res] = '\n';
								res++;
								
						}
						else if(para_value.reg_head_type == 1)
						{
						    res = sprintf((char *)host_tx_buff,"+REGHEAD:%s\r\n",para_value.heartbeat_head);
						}
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
				    //��������1
            res = sprintf((char *)host_tx_buff,"+SERVER1:%d,%s,%d\r\n",para_value.link1.connect_type,
							                                       para_value.link1.ip_addr, para_value.link1.port);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
    		    //��������2
            res = sprintf((char *)host_tx_buff,"+SERVER2:%d,%s,%d\r\n",para_value.link2.connect_type,
							                                       para_value.link2.ip_addr, para_value.link2.port);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
    		    //�Զ�����ʱ��
					  res = sprintf((char *)host_tx_buff,"+IDLTIME:%d\r\n",para_value.auto_off_line_time);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
    		    //GPRS APN����
					  res = sprintf((char *)host_tx_buff,"+APN:%s,%s,%s\r\n",para_value.apn_para.apn_name, 
							                      para_value.apn_para.user_name, para_value.apn_para.user_pwd);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
    		    break;	
						
						
				case HOST_CMD_CSQ://��ѯ�����ź�����
            //��ȡ�����ź�����
            if(sim900a_send_data_ack("AT+CSQ\r\n",8,"+CSQ",50) == 0)//��ѯ�ź�ǿ��
            {
            	  RSSI = (USART3_RX_BUF[8]-0X30)*10+(USART3_RX_BUF[9]-0X30);
            }
            else
            {
                //�޷���ȡ�ź�ǿ��
							  RSSI = 99;
            }
				    temp[0] = RSSI;

					  res = sprintf((char *)host_tx_buff,"+CSQ:%d\r\n", temp[0]);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);	
    		    break;	
				case HOST_CMD_GSTATE://��ѯGPRS����״̬
						if(sim900a_send_data_ack("AT+CGATT?\r\n",11,"+CGATT",200) == 0)//��������
					  {
							  temp[0] = USART3_RX_BUF[10] - 0x30;
						}
						else
						{
						    temp[0] = 0;
						}
					  res = sprintf((char *)host_tx_buff,"+GPRS:%d\r\n", temp[0]);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);	
    		    break;	
				case HOST_CMD_DTUIP://��ѯDTU IP��ַ
					  //��Ϊ���ָ��ֱ�ӷ���IP������Ҫ�ж��Ƿ�ERROR
						if(sim900a_send_data_ack("AT+CIFSR\r\n",10,"ERROR",200) == 0)//��������
					  {
							  res = 0;
							  res = sprintf((char *)host_tx_buff,"+DTUIP:%s\r\n", "0.0.0.0");
						}
						else
						{
							  
						    res = sprintf((char *)host_tx_buff,"+DTUIP:%s", &USART3_RX_BUF[2]);
						}

					  
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);	
    		    break;	
				case HOST_CMD_NET://��ѯ����ע��״̬
						if(sim900a_send_data_ack("AT+CREG=2\r\n",11,"OK",200) == 0)//��������
					  {
							  res = 1;
						}
						else
						{
						    res = 0;
						}
						delay_ms(100);
						if(sim900a_send_data_ack("AT+CREG?\r\n",10,"+CREG",200) == 0)//��������
					  {
							  res = 1;
						}
						else
						{
						    res = 0;
						}
					  res = sprintf((char *)host_tx_buff,"+NET:%s\r\n", &USART3_RX_BUF[11]);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
						if(sim900a_send_data_ack("AT+CREG=0\r\n",11,"OK",200) == 0)//��������
					  {
							  res = 1;
						}
						else
						{
						    res = 0;
						}
    		    break;	
				
				case HOST_CMD_CSMINS://��ѯSIM���Ƿ����
						if(sim900a_send_data_ack("AT+CSMINS?\r\n", 12, "+CSMINS", 200) == 0)//��������
					  {
							  temp[0] = USART3_RX_BUF[13]-0X30;
						}
						else
						{
						    temp[0] = 0;
						}

					  res = sprintf((char *)host_tx_buff,"+CSMINS:%d\r\n", temp[0]);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);	
    		    break;	

    }
		if(res > 0)
		{
        res = sprintf((char *)host_tx_buff,"OK\r\n");
        usart_send_str(HOSTIF_USART, host_tx_buff, res);
		}
		else
		{
		    res = sprintf((char *)host_tx_buff,"ERROR\r\n");
        usart_send_str(HOSTIF_USART, host_tx_buff, res);
		}
}
//��������
void set_cmd_process(u8 cmd, u8 *para)
{
	  int res = 0, i = 0;

    temp[0] = 0;
    temp[1] = 0;
    temp[2] = 0;
    temp[3] = 0;
    temp[4] = 0;
		memset(temp_str,0,130);
    switch(cmd)
    {
    	  case HOST_CMD_WKMODE://����ģʽ
					  res = para[7] - 0x30;
				    if(res > 8)
						{
						    res = -1;
						}
						else
						{
						    para_value.word_mode = res;
						}
    		    break;	
				case HOST_CMD_GPRSMODE://����ģʽ
				 	  res = para[9] - 0x30;
				    if(res > 3)
						{
						    res = -1;
						}
				    else
						{
						    para_value.on_line_mode = res;
						}
    		    break;	
				case HOST_CMD_DBGMODE://����ģʽ
				 	  res = para[8] - 0x30;
				    if(res > 3)
						{
						    res = -1;
						}
				    else
						{
						    para_value.debug_mode = res;
						}
    		    break;	
				case HOST_CMD_DTUID://ID����
					  para  = para + 6;
            while(*para != HOST_REVEIVE_END_STR)
						{
						    temp_str[i] = *para++;
							  i++;
							  if(i > 32)
								{
								    res = -1;
									  break;
								}
								
						}

						
						if(res != -1)
						{
							  para_value.id_len = i;
							  res = i;
							  memset((para_value.id),0,34);
							  for(i=0; i<34; i++)
								{
								    para_value.id[i] =  temp_str[i];
								}
						}
						
    		    break;	
				case HOST_CMD_UART://���ڲ���
				    para = para + 5;
				    
            //�������Ƿ�����
				    while(*para != HOST_REVEIVE_END_STR)
						{
							  if(*para++ == ',')
								{
								    res++;
								}
						    i++;
						}
						if(res == 4)
						{
						    res = 0;
							  para = para-i;
						}
						else
						{
						    res = -1;
						}
				    //��ȡ���ݵ��������
						if(res != -1)
						{
						    for(res=0; res<5; res++)
				        {
						    	  
						        while(*para != ',')
						    		{
						    		    temp[res] = temp[res]*10 + (*para++ - 0x30);
						    			  if(*para == HOST_REVEIVE_END_STR)
						    				{
						    				    break;
						    				}
						    		}
						    		para++;
						    }
						    //��������������
                if((temp[0] == 1200 ) || (temp[0] == 2400 ) || (temp[0] == 4800) 
                || (temp[0] == 9600 ) || (temp[0] == 14400) || (temp[0] == 19200)
                || (temp[0] == 38400) || (temp[0] == 57600) || (temp[0] == 115200))
                {
                
                }
                else
                {
                     res = -1;
                }
                
                if((temp[1] == 8) || (temp[1] == 9))
                {
                
                }
                else
                {
                    res = -1;
                }
                
                if(temp[2] > 2)
                {
                    res = -1;
                }
                
                if(temp[3] > 3)
                {
                    res = -1;
                }
                
                if(temp[4] > 2)
                {
                    res = -1;
                }
								//������ֵ
						    if(res != -1)
						    {
						        para_value.usart_para.baurd = temp[0];
						    	  para_value.usart_para.data_bits = temp[1];
						    	  para_value.usart_para.parity = temp[2];
						    	  para_value.usart_para.stop = temp[3];
						    	  para_value.usart_para.flow_ctr = temp[4];
									
									  uart2_init(para_value.usart_para);	 //���ڳ�ʼ��Ϊ115200
						    }		
						}
						
    		    break;	
				case HOST_CMD_DFI://�������ݰ�ʱ����
					  para = para + 4;
						while(*para != ',')
						{
						    temp[0] = temp[0]*10 + (*para++ - 0x30);
							  if(*para == HOST_REVEIVE_END_STR)
								{
								    break;
								}
						}
				    if(res > 5000)
						{
						    res = -1;
						}
				    else
						{
						    para_value.usart_rx_interval = temp[0];
							  res = temp[0];
						}
    		    break;	
				case HOST_CMD_HBTIME://����ʱ��
					  para = para + 7;
						while(*para != ',')
						{
						    temp[0] = temp[0]*10 + (*para++ - 0x30);
							  if(*para == HOST_REVEIVE_END_STR)
								{
								    break;
								}
						}
				    if(res > 3600)
						{
						    res = -1;
						}
				    else
						{
						    para_value.heartbeat_time = temp[0];
							  res = temp[0];
						}
						
    		    break;	
				case HOST_CMD_HBHEAD://������ͷ
					  para = para + 7;
					  if((*para == 'H') && (*(para+1) == 'E') && (*(para+2) == 'X'))
						{
							  temp[0] = 0;
						    para = para + 3;
							  while(*para != HOST_REVEIVE_END_STR)
								{
								    if(*para == ' ')
										{
										    para++;
										}
										temp_str[i] = (*para++ - 0x30)<<4 ;
										temp_str[i]  |= (*para++ - 0x30) ;
										i++;
										if(i > 32)
										{
											  res = -1;
											  break;
										}
								}
						}
						else
						{
							  temp[0] = 1;
                while(*para != HOST_REVEIVE_END_STR)
						    {
						        temp_str[i] = *para++;
						    	  i++;
						    	  if(i > 32)
						    		{
											  res = -1;
						    			  break;
						    		}
						    }
						}
						
						if(res != -1)
						{
		            para_value.heartbeat_head_len = i;
			          para_value.heartbeat_head_type = temp[0];
							  res = i;
							  memset((para_value.heartbeat_head),0,34);
							  for(i=0; i<34; i++)
								{
								    para_value.heartbeat_head[i] = temp_str[i];
								}
						    
						}
						
    		    break;	
				case HOST_CMD_REGPKG://ע�������
					  res = para[7] - 0x30;
				    if(res > 1)
						{
						    res = -1;
						}
						else
						{
						    para_value.reg_packet_switch = res;
						}
    		    break;	
				case HOST_CMD_REGHEAD://ע���ͷ
					  para = para + 8;
					  if((*para == 'H') && (*(para+1) == 'E') && (*(para+2) == 'X'))
						{
							  temp[0] = 0;
						    para = para + 3;
							  while(*para != HOST_REVEIVE_END_STR)
								{
								    if(*para == ' ')
										{
										    para++;
										}
										temp_str[i] = (*para++ - 0x30)<<4 ;
										temp_str[i]  |= (*para++ - 0x30) ;
										i++;
										if(i > 32)
										{
											  res = -1;
											  break;
										}
								}
						}
						else
						{
							  temp[0] = 1;
                while(*para != HOST_REVEIVE_END_STR)
						    {
						        temp_str[i] = *para++;
						    	  i++;
						    	  if(i > 32)
						    		{
											  res = -1;
						    			  break;
						    		}
						    }
						}
						
						if(res != -1)
						{
		            para_value.reg_head_len = i;
			          para_value.reg_head_type = temp[0];
							  res = i;
							  memset((para_value.reg_head),0,34);
							 for(i=0; i<34; i++)
								{
								    para_value.reg_head[i] = temp_str[i];
								}
						    
						}
    		    break;	
				case HOST_CMD_SERVER1://��������1
					  para = para + 8;
				    //�������Ƿ�����
				    while(*para != HOST_REVEIVE_END_STR)
						{
							  if(*para++ == ',')
								{
								    res++;
								}
								i++;
						}
						if(res == 2)
						{
						    res = 0;
							  para = para-i;
							  i = 0;
						}
						else
						{
						    res = -1;
						}
				    //��ȡ����
						if(res != -1)
						{
							  //��ȡ��һ������
							  while(*para != ',')
								{
								    temp[0] = temp[0]*10 + (*para++ - 0x30);
								}
								
								//�жϲ����ĺϷ���
								if(temp[0] > 3)
		            {
						    	  res = -1;
		            }
								//��ȡ�ڶ�������
								if(res != -1)
								{
									  para++;
								    while(*para != ',')
								    {
								    	  temp_str[i] = *para++;
								        i++;
											  //�ж����ݳ��ȵĺϷ���
								    	  if(i > 64)
								    		{
								    		    res = -1;
								    			  break;
								    		}
								    }
								}
								//��ȡ����������
								if(res != -1)
								{
									  para++;
								    while(*para != HOST_REVEIVE_END_STR)
								    {
								    	  temp[1] = temp[1]*10 + (*para++ - 0x30);
								    }
								}
								
				        if(res != -1)
						    {
									  res = temp[0];
		                para_value.link1.connect_type = temp[0];
			              para_value.link1.port = temp[1];
									  for(i=0; i<66; i++)
										{
										    para_value.link1.ip_addr[i] = temp_str[i];
										}
						    }
						}
    		    break;	
				case HOST_CMD_SERVER2://��������2
					  para = para + 8;
				    //�������Ƿ�����
				    while(*para != HOST_REVEIVE_END_STR)
						{
							  if(*para++ == ',')
								{
								    res++;
								}
								i++;
						}
						if(res == 2)
						{
						    res = 0;
							  para = para-i;
							  i = 0;
						}
						else
						{
						    res = -1;
						}
				    //��ȡ����
						if(res != -1)
						{
							  //��ȡ��һ������
							  while(*para != ',')
								{
								    temp[0] = temp[0]*10 + (*para++ - 0x30);
								}
								
								//�жϲ����ĺϷ���
								if(temp[0] > 3)
		            {
						    	  res = -1;
		            }
								//��ȡ�ڶ�������
								if(res != -1)
								{
									  para++;
								    while(*para != ',')
								    {
								    	  temp_str[i] = *para++;
								        i++;
											  //�ж����ݳ��ȵĺϷ���
								    	  if(i > 64)
								    		{
								    		    res = -1;
								    			  break;
								    		}
								    }
								}
								//��ȡ����������
								if(res != -1)
								{
									  para++;
								    while(*para != HOST_REVEIVE_END_STR)
								    {
								    	  temp[1] = temp[1]*10 + (*para++ - 0x30);
								    }
								}
								
				        if(res != -1)
						    {
									  res = temp[0];
		                para_value.link2.connect_type = temp[0];
			              para_value.link2.port = temp[1];
									  for(i=0; i<66; i++)
										{
										    para_value.link2.ip_addr[i] = temp_str[i];
										}
						    }
						}
    		    break;	
				case HOST_CMD_IDLTIME://�Զ�����ʱ��
					  para = para + 8;
						while(*para != ',')
						{
						    temp[0] = temp[0]*10 + (*para++ - 0x30);
							  if(*para == HOST_REVEIVE_END_STR)
								{
								    break;
								}
						}
				    if(res > 5000)
						{
						    res = -1;
						}
				    else
						{
						    para_value.auto_off_line_time = temp[0];
							  res = temp[0];
						}
    		    break;	
				case HOST_CMD_APN://GPRS APN����
					  para = para + 4;
				    //�������Ƿ�����
				    while(*para != HOST_REVEIVE_END_STR)
						{
							  if(*para == ',')
								{
								    res++;
								}
								para++;
								i++;
						}
						if(res == 2)
						{
						    res = 0;
							  para = para-i;
							  i = 0;
						}
						else
						{
						    res = -1;
						}
				    //��ȡ����
						if(res != -1)
						{
							  //��ȡ��һ������
							  while(*para != ',')
								{
								    temp_apn_para.apn_name[i] = *para++;
								    i++;
								    //�ж����ݳ��ȵĺϷ���
								    if(i > 16)
								    {
								        res = -1;
								    	  break;
								    }
								}
								
								if(res != -1)
								{
									  para++;
									  temp[0] = i;
									  i = 0;
								    while(*para != ',')
								    {
								    	  temp_apn_para.user_name[i] = *para++;
								        i++;
											  //�ж����ݳ��ȵĺϷ���
								    	  if(i > 32)
								    		{
								    		    res = -1;
								    			  break;
								    		}
								    }
								}
								if(res != -1)
								{
									  para++;
									  temp[1] = i;
									  i = 0;
								    while(*para != HOST_REVEIVE_END_STR)
								    {
								    	  temp_apn_para.user_pwd[i] = *para++;
								        i++;
											  //�ж����ݳ��ȵĺϷ���
								    	  if(i > 32)
								    		{
								    		    res = -1;
								    			  break;
								    		}
								    }
								}
								
				        if(res != -1)
						    {
									  temp[2] = i;
									  res = temp[0];
									  memset((para_value.apn_para.apn_name),0,18);
									  memset((para_value.apn_para.user_name),0,34);
									  memset((para_value.apn_para.user_pwd),0,34);
									  while(temp[0]--)
										{
										    para_value.apn_para.apn_name[temp[0]] = temp_apn_para.apn_name[temp[0]];
										}
									  while(temp[1]--)
										{
										    para_value.apn_para.user_name[temp[1]] = temp_apn_para.user_name[temp[1]];
										}
									  while(temp[2]--)
										{
										    para_value.apn_para.user_pwd[temp[2]] = temp_apn_para.user_pwd[temp[2]];
										}
										if((temp_apn_para.user_name[0] == 0) && (temp_apn_para.user_pwd[0] == 0))
										{
										    res = sprintf((char *)temp_str,"AT+CSTT=\"%s\"\r\n", para_value.apn_para.apn_name);
										}
										else
										{
										    res = sprintf((char *)temp_str,"AT+CSTT=\"%s\",\"%s\",\"%s\"\r\n", para_value.apn_para.apn_name,
										    	                                  para_value.apn_para.user_name, para_value.apn_para.user_pwd);
										}
										
										
						        //if(sim900a_send_data_ack(temp_str,res,"OK",200) == 0)//��������
					          //{
						        //	  res = 1;
						        //}
						        //else
						        //{
						        //    res = -1;
						        //}
										
						    }
						}
				
				
    		    break;	
						
				case HOST_CMD_HTTPS://HTTP����
					  para = para + 6;
				    while(*para != HOST_REVEIVE_END_STR)
						{
							  if(*para++ == ',')
								{
								    res++;
								}
								i++;
						}
						if(res == 2)
						{
						    res = 0;
							  para = para-i;
							  i = 0;
						}
						else
						{
						    res = -1;
						}
				    //��ȡ����
						if(res != -1)
						{
							  //��ȡ��һ������
							  while(*para != ',')
								{
								    temp[0] = temp[0]*10 + (*para++ - 0x30);
								}
								//�жϲ����ĺϷ���
								if(temp[0] > 2)
		            {
						    	  res = -1;
		            }
								//��ȡ�ڶ�������
								if(res != -1)
								{
									  para++;
							      while(*para != ',')
								    {
								        temp[1] = temp[1]*10 + (*para++ - 0x30);
								    }
								    //�жϲ����ĺϷ���
								    if(temp[1] > 2)
		                {
						        	  res = -1;
		                }
								}
								//��ȡ����������
								if(res != -1)
								{
									  para++;
								    while(*para != HOST_REVEIVE_END_STR)
								    {
								    	  temp_str[i] = *para++;
								        i++;
											  //�ж����ݳ��ȵĺϷ���
								    	  if(i > 100)
								    		{
								    		    res = -1;
								    			  break;
								    		}
								    }
								}
								
				        if(res != -1)
						    {
									  res = temp[0];
		                para_value.http_para.type = temp[0];
			              para_value.http_para.out_head = temp[1];
									  memset(para_value.http_para.url, 0, 130);
									  for(i=0; i<130; i++)
										{
										    para_value.http_para.url[i] = temp_str[i];
										}
						    }
						}
					  break;
				
				case HOST_CMD_HTTPHD://HTTP����
            para = para + 7;
						//��ȡ��һ������
				    while(*para != HOST_REVEIVE_END_STR)
						{
						    temp_str[i] = *para++;
						    i++;
						    //�ж����ݳ��ȵĺϷ���
						    if(i > 100)
						    {
						        res = -1;
						    	  break;
						    }
						}
				    if(res != -1)
						{
						    res = 1;
							  temp[0] = 0;
							  temp[1] = 0;
							  memset(para_value.http_para.head, 0, 130);
							  //������Ҫ�жϼ�ת�������Ȼsim800cʶ����
							  for(temp[0]=0; temp[0]<i;)
							  {
									  if(temp_str[temp[0]] == '\r')
										{
										    para_value.http_para.head[temp[1]] = 0x5c;//   '\'
											  temp[1]++;
                        para_value.http_para.head[temp[1]] = 'r';//
											  temp[1]++;
										}
										else if(temp_str[temp[0]] == '\n')
										{
										    para_value.http_para.head[temp[1]] = 0x5c;//   '\'
											  temp[1]++;
                        para_value.http_para.head[temp[1]] = 'n';//   
											  temp[1]++;
										}
										else
										{
						            para_value.http_para.head[temp[1]] = temp_str[temp[0]];
										    temp[1]++;
										}
										temp[0]++;
								}
								
								for(i=0; i<temp[1]; i++)
								{
								     para_value.modbus_http_get_para.head[i]  = para_value.http_para.head[i];
									   para_value.modbus_http_post_para.head[i]  = para_value.http_para.head[i];
								}
						}

					  break;
				case HOST_CMD_QRYPTM://��ѯPOST���ʱ��
					  para = para + 7;
				    //��ȡ��һ������
						while(*para != ',')
						{
						    temp[0] = temp[0]*10 + (*para++ - 0x30);
							  if(*para == HOST_REVEIVE_END_STR)
								{
								    break;
								}
						}
					  para++;
				    //��ȡ�ڶ�������
						while(*para != ',')
						{

						    temp[1] = temp[1]*10 + (*para++ - 0x30);
							  if(*para == HOST_REVEIVE_END_STR)
								{
								    break;
								}
						}
				    if((temp[0] > 3600) || (temp[1] > 3000))
						{
						    res = -1;
						}
				    else
						{
						    para_value.modbus_qry_post_time = temp[0];
							  para_value.modbus_qry_cmd_time = temp[1];
							  res = temp[0];
						}
					  break;
				case HOST_CMD_QRYPOST://��ѯPOST
					  para = para + 8;
				    //�Ȱѵ�һ��������ȡ����
						while(*para != ',')
						{
						    temp[0] = temp[0]*10 + (*para++ - 0x30);
							  if(*para == HOST_REVEIVE_END_STR)
								{
								    break;
								}
						}
						para++;
						
						if(temp[0] >=16)
						{
						    res = -1;
						}
						
						//�жϵڶ��������ĺϷ���
						
						if(res != -1)
						{
						    i = 0;
				        while(*para != HOST_REVEIVE_END_STR)
						    {
						        para++;
						    	  i++;
						    }
						    if((i%2 != 0) || (i > 64))
						    {
						        res = -1;
						    }
						}

				    //��ȡ�ڶ�������
				    if(res != -1)
						{
							  para = para - i;
							  i = 0;
							  while(*para != HOST_REVEIVE_END_STR)
								{
								    if(*para == ' ')
										{
										    para++;
										}
										
										for(temp[1]=0;temp[1]<16;temp[1]++)
										{
										    if(assic_hex_tab[temp[1]] == *para)
												{
													   break;
												}
										}
										para++;
										for(temp[2]=0;temp[2]<16;temp[2]++)
										{
										    if(assic_hex_tab[temp[2]] == *para)
												{
													   break;
												}
										}
										para++;
										temp_str[i] = (temp[1]<<4) | (temp[2]);

										i++;
										if(i > 32)
										{
											  res = -1;
											  break;
										}
								}
						}
						
						//�ѻ�ȡ�����ݸ�ֵ��  para_value
						if(res != -1)
						{
							  res = i;
							  para_value.modbus_qry_post_para[temp[0]].len = i;
						    for(i=0; i<34; i++)
								{
								    para_value.modbus_qry_post_para[temp[0]].buff[i] = temp_str[i];
								}
						}
					  break;
				case HOST_CMD_QRYPPA://������ѯpost http����
					  para = para + 7;
						//��ȡ��һ������
				    while(*para != HOST_REVEIVE_END_STR)
						{
						    temp_str[i] = *para++;
						    i++;
						    //�ж����ݳ��ȵĺϷ���
						    if(i > 128)
						    {
						        res = -1;
						    	  break;
						    }
						}
						//�ѻ�ȡ�����ݸ�ֵ��  para_value
						if(res != -1)
						{
							  res = i;
							  para_value.modbus_http_post_para.type = 0;
							  para_value.modbus_http_post_para.out_head = 0;
							  for(i=0; i<sizeof(para_value.http_para.head); i++)
							  {
								    para_value.modbus_http_post_para.head[i] = para_value.http_para.head[i];
								}
						    for(i=0; i<130; i++)
								{
								    para_value.modbus_http_post_para.url[i] = temp_str[i];
								}
						}
						break;
				case HOST_CMD_QRYGTM://��ѯGET���ʱ��
					  para = para + 7;
						while(*para != ',')
						{
						    temp[0] = temp[0]*10 + (*para++ - 0x30);
							  if(*para == HOST_REVEIVE_END_STR)
								{
								    break;
								}
						}
				    if(res > 3600)
						{
						    res = -1;
						}
				    else
						{
						    para_value.modbus_qry_get_time = temp[0];
							  res = temp[0];
						}
					  break;
				case HOST_CMD_QRYGET://��ѯGET
					  para = para + 7;  
						//�жϵ�һ�������ĺϷ���
						i = 0;
				    while(*para != HOST_REVEIVE_END_STR)
						{
						    para++;
							  i++;
						}
						if((i%2 != 0) || (i > 64))
						{
						    res = -1;
						}
				    //��ȡ��һ������
				    if(res != -1)
						{
							  para = para - i;
							  i = 0;
							  while(*para != HOST_REVEIVE_END_STR)
								{
								    if(*para == ' ')
										{
										    para++;
										}

								    if(*para == ' ')
										{
										    para++;
										}
										
										for(temp[1]=0;temp[1]<16;temp[1]++)
										{
										    if(assic_hex_tab[temp[1]] == *para)
												{
													   break;
												}
										}
										para++;
										for(temp[2]=0;temp[2]<16;temp[2]++)
										{
										    if(assic_hex_tab[temp[2]] == *para)
												{
													   break;
												}
										}
										para++;
										temp_str[i] = (temp[1]<<4) | (temp[2]);
										
										i++;
										if(i > 32)
										{
											  res = -1;
											  break;
										}
								}
						}
						
						//�ѻ�ȡ�����ݸ�ֵ��  para_value
						if(res != -1)
						{
							  res = i;
							  para_value.modbus_qry_get_para.len = i;
						    for(i=0; i<34; i++)
								{
								    para_value.modbus_qry_get_para.buff[i] = temp_str[i];
								}
						}
					  break;
				case HOST_CMD_QRYGPA://������ѯget http����
					  para = para + 7;
						//��ȡ��һ������
				    while(*para != HOST_REVEIVE_END_STR)
						{
						    temp_str[i] = *para++;
						    i++;
						    //�ж����ݳ��ȵĺϷ���
						    if(i > 128)
						    {
						        res = -1;
						    	  break;
						    }
						}
						//�ѻ�ȡ�����ݸ�ֵ��  para_value
						if(res != -1)
						{
							  res = i;
							  para_value.modbus_http_get_para.type = 1;
							  para_value.modbus_http_get_para.out_head = 0;
							  for(i=0; i<sizeof(para_value.http_para.head); i++)
							  {
								    para_value.modbus_http_get_para.head[i] = para_value.http_para.head[i];
								}
						    for(i=0; i<130; i++)
								{
								    para_value.modbus_http_get_para.url[i] = temp_str[i];
								}
						}
						break;
						
						
						
				
				case HOST_CMD_SYSPWD://DTU�����޸�
            para = para + 7;
						//��ȡ��һ������
				    while(*para != ',')
						{
						    temp_str[i] = *para++;
						    i++;
						    //�ж����ݳ��ȵĺϷ���
						    if(i > 6)
						    {
						        res = -1;
						    	  break;
						    }
						}
						//�ж������Ƿ���ȷ
						if(res != -1)
						{
							  i = para_value.password_len;
	              while(i--)
	              {
	              	  if(para_value.password[i] != temp_str[i])
	              	  {
	              	  	  res = -1;
	              	      break;
	              	  }
	              }
								//��ȡ�µ�����
								if(res != -1)
								{
								    para++;
									  i=0;
				            while(*para != HOST_REVEIVE_END_STR)
						        {
						            temp_str[i] = *para++;
						            i++;
						            //�ж����ݳ��ȵĺϷ���
						            if(i > 6)
						            {
						                res = -1;
						            	  break;
						            }
						        }
								}
								//��ֵ�µ�����
								if(res != -1)
								{
									  para_value.password_len = i;
								    res = i;
									  memset((para_value.password),0,8);
									  for(i=0; i<8; i++)
										{
										    para_value.password[i] = temp_str[i];
										}
								}
						}
    		    break;		
				case HOST_CMD_BUFMODE:
						
    		    break;	
						
				case HOST_CMD_WAKEUP://��������
					  //����TCP���Ӳ���
				    if(para_value.on_line_mode != 0)
						{
						    res = connect_tcp_udp(para_value.link1);
						}
    		    break;	
				case HOST_CMD_OFFLINE://��������
            res = off_line();
    		    break;	
				
	
				case HOST_CMD_RESET://DTU�ָ���������
            para = para + 6;
						//��ȡ��һ������
				    while(*para != HOST_REVEIVE_END_STR)
						{
						    temp_str[i] = *para++;
						    i++;
						    //�ж����ݳ��ȵĺϷ���
						    if(i > 6)
						    {
						        res = -1;
						    	  break;
						    }
						}
						//�ж������Ƿ���ȷ
						if(res != -1)
						{
							  i = para_value.password_len;
	              while(i--)
	              {
	              	  if(para_value.password[i] != temp_str[i])
	              	  {
	              	  	  res = -1;
	              	      break;
	              	  }
	              }
						}
						
						if(res != -1)
						{
						    define_para_value();
							  res = 1;
						}
    		    break;	
				case HOST_CMD_RESTART://DTU����
                    res = sprintf((char *)host_tx_buff,"OK\r\n");
                    usart_send_str(HOSTIF_USART, host_tx_buff, res);
				    //����SIM800C
				    
				    //����
				    NVIC_SystemReset();
    		    break;	
				
				case HOST_CMD_TEST_GSM:
					  para = para + 9;
				    temp[0] = (*para - 0x30);
				    para++;
				    temp[0] = temp[0]*10 + (*para - 0x30);
				    temp[1] = test_gsm(temp[0]);
            res = sprintf((char *)host_tx_buff,"test gsm %d\r\n",temp[1]);
            usart_send_str(HOSTIF_USART, host_tx_buff, res);
				    break;

    }

		if(res >= 0)
		{
        res = sprintf((char *)host_tx_buff,"OK\r\n");
        usart_send_str(HOSTIF_USART, host_tx_buff, res);
		}
		else
		{
		    res = sprintf((char *)host_tx_buff,"ERROR\r\n");
        usart_send_str(HOSTIF_USART, host_tx_buff, res);
		}

		
}

u8 test_gsm(u8 cmd)
{
	  char res = 0;
    switch (cmd)
		{
			  case 0:
						if(sim900a_send_data_ack("AT&F0\r\n",7,"OK",200) == 0)//��������
					  {
							  res = 1;
						}
						else
						{
						    res = 0;
						}
					  break;
				case 1:
						res = read_imei(host_tx_buff);
				    usart_send_str(HOSTIF_USART, host_tx_buff, res);
						break;
				case 2:

						break;
				case 3:
            res = http_post(para_value.link1, para_value.http_para, "act=get_token", 15);
						break;
				case 4:
            res = sprintf((char *)host_tx_buff,"test gsm %d  \\r   \\n    \r\n",temp[1]);
            usart_send_str(HOSTIF_USART, host_tx_buff, res);
					  break;
				case 5:

						break;
				case 6:
            if(sim900a_send_data_ack("AT+CGDCONT=1,\"IP\",\"CMNET\"", 26, "OK", 1000) == 0)//����PDP������,��������Э��,��������Ϣ
					  {
							  res = 1;
						}
						else
						{
						    res = 0;
						}
						break;
				case 7:

						break;

				case 8://�����
						if(sim900a_send_data_ack("AT+CIICR\r\n",10,"OK",2000) == 0)//��������
					  {
							  res = 1;
						}
						else
						{
						    res = 0;
						}
						break;
				case 9://
						if(sim900a_send_data_ack("AT+CSPN?\r\n",10,"OK",200) == 0)//��������
					  {
							  res = 1;
						}
						else
						{
						    res = 0;
						}
						break;
				case 10://��ѯ����״̬
						if(sim900a_send_data_ack("AT+CIPSTATUS\r\n",14,"CONNECT OK",500) == 0)	
						{
							//connectsta=1;//��������
						}
						else
						{
							//connectsta=2;//��������ȫ�����������쳣��ֱ������TCP
						}
						break;
				case 11:
					
				
						break;
				case 12://����TCP����
					  res = connect_tcp_udp(para_value.link1);
				    if(res > 0)
						{
						    res = tcp_udp_send("qwertyu", strlen((char *)"qwertyu"));
						}
						
						break;
						

		}

		usart_send_str(HOSTIF_USART, USART3_RX_BUF, USART3_MAX_RECV_LEN);
		return (u8)res;
}


u8 cmd_process(u8 *str, u16 len)
{
	  u8 i = 0;
	  u8 res = 0;
    for(i=0;i<sizeof(hostif_cmd_tab)/4;i++)//֧�ֵ�ϵͳָ��
		{
			  res = cmd_cmp(str,hostif_cmd_tab[i]);
				if(res != 0)
				{
				    break;
				}
		}
		if((i == 17) && (res == 3))
		{
		    DEBUG("query cmd\r\n");
		    res = 1;
		}
		DEBUG("CMD is %s\r\n",hostif_cmd_tab[i]);
		//��ѯ����
		if(res == 1)
		{
			  quert_cmd_process(i, str);

		}
		//��������
		else if(res == 2)
		{
			  if(str[len-3] == '#')
				{
			      set_cmd_process(i, str);
					  if(i<=HOST_CMD_BUFMODE)
						{
						    write_para_to_flash();
						}
				}
				else
				{
				
				}
			  
		}
		return res;
		
}

u8 tcp_udp_auto_line(void)
{
	  u8 res = 0;
	
	  if(check_tcp_udp_connet() == 0)
		{
        if((para_value.link1.connect_type == 0) || (para_value.link1.connect_type == 1))
        {
        	  if(connect_tcp_udp(para_value.link1) == 0)
        		{
        		    if((para_value.link2.connect_type == 0) || (para_value.link2.connect_type == 1))
        				{
        				    if(connect_tcp_udp(para_value.link2))
										{
										    res = 0; 
										}
										else
										{
										    res = 2;
										}
		    					  
        				}
        		}
						else
						{
						    res = 1;
						}
        }
        else if((para_value.link2.connect_type == 0) || (para_value.link2.connect_type == 1))
        {
            if(connect_tcp_udp(para_value.link1) == 0)
						{
						    res = 0;
						}
						else
						{
						    res = 2;
						}
		    	  
        }
		}
		else
		{
		    res = 3;
		}

		return res;
}



void host_receive_packet(void)
{
    static unsigned int time_stamp = 0;
	static u16 old_receive_len = 0, new_receive_len = 0;
	u16 i = 0;
	u8 res = 0;
	
    new_receive_len = circulation_buff_available(HOSTIF_USART_RX_DMA_CH, HOSTIF_USART_CIR_BUFF);
	if(old_receive_len != new_receive_len)
	{
		old_receive_len = new_receive_len;
	    time_stamp=run_ms;
	}
	
	if(new_receive_len != 0)
	{
		  
	    if(time_diff_ms(time_stamp) > HOST_UASRT_RX_TIME_OUT)
	    {
		    DEBUG("HOST time_stamp is %d\r\n", time_stamp);
		    DEBUG("HOST old_receive_len is %d\r\n", old_receive_len);
			new_receive_len = 0;
			old_receive_len = 0;
			//�������
	    	host_rx_len = circulation_buff_read(HOSTIF_USART_RX_DMA_CH, &HOSTIF_USART_CIR_BUFF, host_rx_buff, 512);
			//��ATָ��ģʽ�����������ݷ���
			if((host_rx_buff[0] == 'A')  && (host_rx_buff[1] == 'T') && (host_rx_buff[host_rx_len-2] == '\r') && (host_rx_buff[host_rx_len-1] == '\n'))
			{
			    if(host_rx_len == 4)
				  {
				      USART_dma_send(HOSTIF_USART_TX_DMA_CH, "OK\r\n", 4);
				  }
				  else if(host_rx_len > 7)
				  {
				      //ʶ������
				  	  res = cmd_process(&host_rx_buff[3], (host_rx_len -3));
				  }
			}
			
			//������յ�����
			if(para_value.word_mode == WOKE_MODE_AT)
			{
			
			}
			if(para_value.word_mode == WOKE_MODE_TS)
			{
			    //ͨ��TCP/UDP�������ݣ��жϳ������������ݣ����䷢�ͳ�ȥ
			    if((res == 0) && (para_value.on_line_mode != 0))
			    {
						  host_rx_buff[host_rx_len-2] = 0;
						  host_rx_buff[host_rx_len-1] = 0;
						  if(para_value.on_line_mode == 2)
						  {
			                if(check_tcp_udp_connet() == 1)
							  {
							      tcp_udp_send(host_rx_buff, host_rx_len);
							  }
						  }
						  else
						  {
			                if(tcp_udp_auto_line() != 0)
						      {
                               tcp_udp_send(host_rx_buff, host_rx_len);      
						      }
						  }
			    }
			}
			else if(para_value.word_mode == WOKE_MODE_HTTP)
			{
			    //ͨ��HTTP�������ݣ��жϳ������������ݣ����䷢�ͳ�ȥ
			    if(res == 0)
			    {
					  host_rx_buff[host_rx_len-2] = '\0';
					  host_rx_buff[host_rx_len-1] = '\0';
			        if(para_value.http_para.type == 0) //post
					  {
					      http_post(para_value.link1, para_value.http_para, host_rx_buff, sizeof(host_rx_buff));
					  }
					  else if(para_value.http_para.type == 1) //get
					  {
					  	  if(http_get(para_value.link1, para_value.http_para, host_rx_buff) == 1)
						  {
						      usart_send_str(HOSTIF_USART, http_get_buff, http_get_rx_len);
						  }
					  }
			    }
			}
			else if(para_value.word_mode == WOKE_MODE_MODBUS_HTTP)
			{
			    //HTTP MODBUS ģʽ������Ҫ�ж���ʲô���ݣ�
			    if((res == 0) && (modbus_get_flog == 0))
			    {
                    for(i=0;i<(host_rx_len);i++)
				    {
				        modbus_http_post_buff[modbus_http_post_len++] = host_rx_buff[i];
				    }
			    }
			}
			
			memset(host_rx_buff, 0, host_rx_len);
			host_rx_len = 0;
				
				  
	    		//DEBUG("HOST usart rx time out\r\n");
	    }
	}
	
}

