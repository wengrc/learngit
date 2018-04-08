#ifndef __STACK_H
#define __STACK_H

struct stackElem
{
  void *            elem;
  struct stackElem *rest;

};

typedef struct stackElem stackElem;


#ifdef __cplusplus
extern "C"{
#endif

int CreateStack ( stackElem *head );
int PushStack ( stackElem *head, void *x );
void* PopStack ( stackElem *head );
void* ReadStack ( stackElem *Head );
int FreeStack ( stackElem *head );
int LengthStack ( stackElem *head );


#ifdef __cplusplus
}
#endif



#endif
