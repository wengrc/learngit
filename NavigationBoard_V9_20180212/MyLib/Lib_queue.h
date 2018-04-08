#ifndef __QUEUE_H
#define __QUEUE_H
struct queue
{
  void *        elem;
  struct queue *rest;
};
typedef struct queue queue;

struct qhead
{
  queue head;
  queue tail;

};
typedef struct qhead qhead;

#ifdef __cplusplus
extern "C"{
#endif

/*��������  */
int   CreateQueue ( qhead *qh );

/*�ڶ���β������һ��Ԫ�� */
int   PutQueue ( qhead *qh, void *x );

/*�Ӷ�����ͷ������һ��Ԫ��*/
void* ReadQueue ( qhead *qh );

/*�Ӷ���ͷ���Ƴ�һ��Ԫ��*/
void* RemoveQueue ( qhead *qh );

/*�ͷ���������*/
int   FreeQueue ( qhead *qh );

/*���ض��г���*/
int   LengthQueue( qhead *qh );

#ifdef __cplusplus
}
#endif

#endif


