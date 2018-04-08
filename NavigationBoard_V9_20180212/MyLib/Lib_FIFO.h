
#ifndef __LIB_FIFO_H
#define __LIB_FIFO_H

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>


/***********************************************************************
 * �Զ�������
 ************************************************************************/

//����FIFO
#define MAX_FIFO_LGTH  247

typedef struct
{
    short begin_pos; //�ǳ���Ҫ: �������MAX_FIFO_LGTH��дFIFO��� !! �����ʼ��Ϊ��
    short end_pos;   //�ǳ���Ҫ: �������MAX_FIFO_LGTH��дFIFO��� !! �����ʼ��Ϊ��
    short size;
    char *buf;

}fifo;

//����֡��ʽ
#define MAX_FRAME_LGTH        150
#define FRAME_STATUS_EMPTY    0
#define FRAME_STATUS_LOADING  1
#define FRAME_STATUS_FINISH   2

typedef struct            //����֡һ��Ҫ��ʼ��Ϊ��
{
    short lgth;           //���ǳ���Ҳ��buf��д����
    unsigned char status; //0:��֡,�ȴ��������ͷ��1�ȴ���������������β 2���һ֡
    short size;
    char *buf;

}frame;


typedef struct
{
    unsigned char size;
    unsigned char free;
    unsigned char used;
    short *buf;

}data_list;


/***********************************************************************
 * ��궨��
 ************************************************************************/

/***********************************************************************
 * ȫ�ֱ�������
 ************************************************************************/

/***********************************************************************
 * ȫ�ֺ�������
 ************************************************************************/


#ifdef __cplusplus
extern "C" {
#endif

/*����ѭ��FIFO��FIFOָ������������, bufָ���������׵�ַ��sizeָ����С*/
void  CreateFIFO(fifo *FIFO,
                 char *buf,
                 short size);

/*��������֡��pframe�洢֡��Ϣ��buf�󶨻��棬sizeָ��buf��С  */
void  CreateFrame(frame *pframe,
                  char *buf,
                  short size);

/*д FIFO , ÿ��дһ���ֽ� */
void  Write_FIFO(fifo *FIFO,
                 unsigned char ch);

/*��FIFO , ÿ�ζ�һ���ֽ� ��readPosά��ÿһ�ζ�����λ�ã�������ǩ����һ�ζ���
    ���ϴε�λ�ÿ�ʼ����readPos��ʼ��ΪFIFO�����ʼ��ַ
    */
char* Read_FIFO(fifo *FIFO,
                short *readPos);

/*����FIFO���ȣ��ӵ�ǰλ��curpos���� */
uint16_t  FIFOLength(fifo *FIFO,
                     short curpos);

/*�ӵ�ǰλ�����𣬼���FIFOʣ��ռ�  */
uint16_t  FIFORemaind(fifo *FIFO,
                      short curpos);

/*��FIFO��ץȡһ֡���ݣ�readPos���ϴζ�ȡλ�ÿ�ʼ��beginStr ֡��ͷ��ƥ�䣬endStr֡��β��ƥ�䣬
  pframe ֡�Ĵ�ŵ�ַ
*/
void  FetchFrame_BE(fifo *FIFO,
                    short *readPos,
                    const char *beginStr,
                    const char *endStr,
                    frame *pframe);

/*��FIFO��ץȡһ֡���ݣ�readPos���ϴζ�ȡ��λ�ÿ�ʼ��beginStr��ʼƥ��ͷ����������
Pos_dataLgth���ȴ洢λ�ã�invariantLgth�۳����ݲ��ֵĹ̶����ȣ�pframe֡���ݴ洢��λ��

*/
void  FetchFrame_BL(fifo *FIFO,
                    short *readPos,
                    const char *beginStr,
                    unsigned char Pos_dataLgth,
                    unsigned char invariantLgth,
                    frame *pframe);


/* ������֡���뷢��FIFO, �ɷ���FIFO������ȡ���Զ���������

*/
void  Mov_TxFrame_to_TxFifo(const frame *TX_frame_x,
                            fifo *TX_FIFO_x);


#ifdef __cplusplus
}
#endif

#endif


