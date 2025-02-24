 #ifndef ET16S_RF209_H
 #define ET16S_RF209_H
 /* ����ͷ�ļ� ----------------------------------------------------------------*/
 #include "struct_typedef.h"

 /* ���Ͷ��� --------------------------------------------------------------*/
 typedef struct
 {
     int16_t switch_value;
     int16_t last_switch_value;
     int16_t switch_state;
 }WBUS_Switch_t;


 typedef struct{//WBUS
 	int16_t Ch1;
 	int16_t Ch2;
 	int16_t Ch3;
 	int16_t Ch4;
 	int16_t Ch5;
 	int16_t Ch6;
 	int16_t Ch7;
 	int16_t Ch8;
 	int16_t Ch9;
 	int16_t Ch10;
 	int16_t Ch11;
 	int16_t Ch12;
 	int16_t Ch13;
 	int16_t Ch14;
 	int16_t Ch15;
 	int16_t Ch16;
 	uint8_t Flag;
     uint8_t End;

    struct 
    {
     /*��ط�ET16s�Ĳ��˿���*/
     /* ��ز���  */
     int16_t SF;
     int16_t SH;
     int16_t SG;
     int16_t SE;
     /* ������ */
     int16_t LD;
     int16_t RD;
     /* ����ģʽ�� */
     int16_t SA;
     int16_t SD;
     int16_t SB;
     int16_t SC;
    /*ʹ���µĲ���ʱ�����ڴ�����µı���*/
    }toggle_switch;
 }WBUS_info_t;
 /* �궨�� ----------------------------------------------------------------*/
 #define WBUS_RX_LEN	 25 //25?
 #define StartByte 0x0f

 #define RC_ET_STICK_OFFSET     (1024u)     // �����м�ֵ
 #define RC_ET_RESOLUTION       (671.5f)    // ң��ȡֵ��Χ����ת����  //DT7 660f ET16S 671.5f
 #define RC_ET_VALUE_MIN        (352u)      //DT7 364 ET16S 352
 #define RC_ET_VALUE_MAX        (1695u)     //DT7 1684 ET16S 1695
 #define RC_ET_DEADBAND         (5)         // ң������������Ϊң�����Ĳ�������λʱ��һ��Ϊ0
 #define Rising_edge          (1)
 #define Falling_edge         (0)

 #define REMOTE_ET_SWITCH_VALUE_UP          1695    //DT7 0x01u ET16S 352
 #define REMOTE_ET_SWITCH_VALUE_DOWN        352   //DT7 0x02u ET16S 1695
 #define REMOTE_ET_SWITCH_VALUE_OFF         0x00u  
 #define REMOTE_ET_SWITCH_VALUE_CENTRAL     1024  //DT7 0x03u ET16S 1024
 /* ˽�б��� ------------------------------------------------------------------*/

 /* ��չ���� ------------------------------------------------------------------*/
 extern uint8_t WBUS_buffer[18];
 /* ˽�к���ԭ�� --------------------------------------------------------------*/
 void RC_WBUSDataParser(WBUS_info_t *WBUS, uint8_t *buf, uint16_t len);
 void RC_ET_SwitchAction(WBUS_Switch_t *sw, uint16_t value);
 void RC_gimbal_DataParser(WBUS_info_t *SBUS, uint8_t *buf, uint16_t len);
 static void Frame_Loss_protection(WBUS_info_t *WBUS);
 WBUS_info_t* WBUS_GetDataPointer(void);
 #endif // __ET16S_RF209_H__

