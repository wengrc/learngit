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

/*创建队列  */
int   CreateQueue ( qhead *qh );

/*在队列尾部放入一个元素 */
int   PutQueue ( qhead *qh, void *x );

/*从读队列头部读出一个元素*/
void* ReadQueue ( qhead *qh );

/*从队列头部移除一个元素*/
void* RemoveQueue ( qhead *qh );

/*释放整个队列*/
int   FreeQueue ( qhead *qh );

/*返回队列长度*/
int   LengthQueue( qhead *qh );

#ifdef __cplusplus
}
#endif

#endif


