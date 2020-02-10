#include "DFRobot_queue.h"
#include "MemoryFree.h"
struct sQueueData *cumsgBufHead=NULL;
struct sQueueData *cumsgBufTail=NULL;

void cuappEnqueue(uint8_t *pbuf, uint16_t len, uint16_t conn_id){	
    struct sQueueData *p;
    p = (struct sQueueData*)malloc(sizeof(struct sQueueData)+len+1);
    if(p == NULL){
/* 		Serial.print("Memory==");Serial.println(getFreeMemory());
		Serial.print("pbuf==");Serial.println((char*)pbuf); */
		//Serial.println("==================");
        free(p);
        return;
    }
    p->next = NULL;
    if(cumsgBufHead==NULL){
        cumsgBufHead=p;
        cumsgBufTail=p;
    }else{
        cumsgBufTail->next = p;
        cumsgBufTail = p;
    }
    p->len = len;
    p->handle = conn_id;
    memset(p->data,'\0',len+1);
    memcpy(p->data, pbuf, len);
	//Serial.print("conn_id==");Serial.println(conn_id);
	//Serial.print("p->data==");Serial.println((char*)p->data);
	//Serial.println((char*)(getQueueHead())->data);
}

struct sQueueData *cuappDequeue(void){
    struct sQueueData *p;
    p = cumsgBufHead;
    if(cumsgBufHead != NULL){
        cumsgBufHead = p->next;
    }
    return p;
}

struct sQueueData *getQueueTail(void){
    struct sQueueData *p;
    p = cumsgBufTail;
    return p;
}

struct sQueueData *getQueueHead(void){
    struct sQueueData *p;
    p = cumsgBufHead;
    return p;
}


