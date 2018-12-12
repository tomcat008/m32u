#ifndef __HOST_IF_H
#define __HOST_IF_H	 
#include "sys.h"

#define HOST_UASRT_RX_TIME_OUT             para_value.usart_rx_interval


#define HOST_REVEIVE_END_STR               '#'

#define HOST_RX_BUFF_LEN              512
#define HOST_TX_BUFF_LEN              512


#define WOKE_MODE_AT                   0
#define WOKE_MODE_TS                   1

#define WOKE_MODE_MODBUS_HTTP          6
#define WOKE_MODE_HTTP                 8

enum CMD_LIST 
{ 
    HOST_CMD_WKMODE = 0,   //����ģʽ
    HOST_CMD_GPRSMODE,     //����ģʽ
    HOST_CMD_DBGMODE,      //����ģʽ
    HOST_CMD_DTUID,        //ID���� 
    HOST_CMD_UART,         //���ڲ���
    HOST_CMD_DFI,          //�������ݰ�ʱ����
    HOST_CMD_HBTIME,       //����ʱ��
    HOST_CMD_HBHEAD,       //������ͷ
    HOST_CMD_REGPKG,       //ע�������
    HOST_CMD_REGHEAD,      //ע���ͷ
    HOST_CMD_SERVER1,      //��������1
    HOST_CMD_SERVER2,      //��������2
    HOST_CMD_IDLTIME,      //�Զ�����ʱ��
    HOST_CMD_APN,          //GPRS APN����
	HOST_CMD_HTTPS,        //HTTP����
	HOST_CMD_HTTPHD,       //HTTP ͷ����
	HOST_CMD_QRYPTM,       //��ѯPOST���ʱ��
	HOST_CMD_QRYPOST,      //��ѯPOST
	HOST_CMD_QRYPPA,       //������ѯpost http����
	HOST_CMD_QRYGTM,       //��ѯGET���ʱ��
	HOST_CMD_QRYGET,       //��ѯGET
	HOST_CMD_QRYGPA,       //������ѯget http����
		
    HOST_CMD_SYSPWD,       //DTU�����޸�
    HOST_CMD_BUFMODE,      //���ݻ���ģʽ
    //�������������Ϣ
    HOST_CMD_WAKEUP,       //��������
    HOST_CMD_OFFLINE,      //��������
    HOST_CMD_VERSION,      //��ѯDTU�汾
    HOST_CMD_LIST,         //��ѯDTU���Բ���
    HOST_CMD_CSQ,          //��ѯ�����ź�����
    HOST_CMD_GSTATE,       //��ѯGPRS����״̬
    HOST_CMD_DTUIP,        //��ѯDTU IP��ַ
    HOST_CMD_NET,          //��ѯ����ע��״̬
    HOST_CMD_RESET,        //DTU�ָ���������
    HOST_CMD_RESTART,      //DTU����
    HOST_CMD_CSMINS,       //��ѯSIM���Ƿ����
    HOST_CMD_TEST_GSM,
};

extern u8 host_rx_buff[HOST_RX_BUFF_LEN];
extern u16 host_rx_len;


extern u8 host_tx_buff[HOST_TX_BUFF_LEN];

extern void host_receive_packet(void);
extern u8 tcp_udp_auto_line(void);


#endif
