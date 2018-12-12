#ifndef __SAVE_DATA_H
#define __SAVE_DATA_H 
#include "sys.h"
#include "usart.h"	

#define SAVE_PARA_ADDR  0X0800e000		//����FLASH �����ַ(����Ϊż��������ֵҪ���ڱ�������ռ��FLASH�Ĵ�С+0X08000000)

typedef struct 
{
	  //  0-none   1--tcp   2--udp
    u8  connect_type;
	  //max len is 64
	  u8  ip_addr[66];
	  u16 port;
	
 }LINK_PARA;

typedef struct 
{
	  //max len is 16
    u8  apn_name[18];
	  //max len is 32
	  u8  user_name[34];
	  //max len is 32
	  u8  user_pwd[34];
}APN_GATEWAT;

typedef struct 
{
	  //  0-POST     1--GET
    u8  type;
	  //  0-�����   1--���
	  u8  out_head;
	  //max len is 64
	  u8  head[130];
	  u8  url[130];
	
}HTTP_PARA;


typedef struct 
{
	  //����ĳ���
    u8  len;
	  //����modbus����
	  u8  buff[34];
	
}MODBUS_PARA;

 
typedef struct 
{
	  //����ģʽ  
    u8  word_mode;
	  //����ģʽ  0--off online   1--alway online  
	  //          2--wake online  3--need online
	  u8  on_line_mode;
	  //����ģʽ  0--close   1--gprs state  2trace(define 0)
	  u8  debug_mode;
	  //id�ŵĳ���  max  32
	  u8  id_len;
	  //id��
	  u8  id[34];
	  //�������ò���  usart para define  baurd--3--9600  data_bits--8
	  //               parity--0     stop--1    flow_ctr---0
	  USART_PARA  usart_para;          
	  //���ڼ��ʱ��max 5000ms    (define  100)
	  u16  usart_rx_interval;
	  //����ʱ��   max 3600s  define 60
	  u16 heartbeat_time;
	  //�������ĳ��� max 32
	  u8  heartbeat_head_len;
	  //����������    0--hex   1--string
	  u8  heartbeat_head_type;
		//������ͷ����
	  u8  heartbeat_head[34];
	  //ע�������  0--close  1--open
	  u8  reg_packet_switch;
	  //ע�������  max 32
	  u8  reg_head_len;
		//ע�������    --hex   1--string
	  u8  reg_head_type;
		//ע���ͷ����
		u8  reg_head[34];
	  //��������1   define 3--none
		LINK_PARA link1;
		//��������2   define 3--none
		LINK_PARA link2;
		//�Զ�����ʱ��    max 3600s   define 300s
		u16 auto_off_line_time;
		//apn��������
		APN_GATEWAT apn_para;
		
		HTTP_PARA http_para;
		u16 modbus_qry_cmd_time;
		u16 modbus_qry_post_time;
		MODBUS_PARA modbus_qry_post_para[16];
		HTTP_PARA modbus_http_post_para;
		
		u16 modbus_qry_get_time;
		MODBUS_PARA modbus_qry_get_para;
		HTTP_PARA modbus_http_get_para;
		
		u8 password_len;
		u8 password[8];
	
}SAVE_DATA;



extern SAVE_DATA para_value;


extern void define_para_value(void);

extern void write_para_to_flash(void);
extern void read_para_from_flash(void);


#endif
