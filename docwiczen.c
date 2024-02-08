#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#define MSG_CMD_SIZE 10
#define BUFFER_SIZE 3

//   wiadomosc
typedef struct
{
    uint8_t sender;
    uint8_t receiver;
    uint8_t data_lenght;
    uint8_t data[MSG_CMD_SIZE];
    uint8_t crc;
}Message;

//  bufor kolowy
typedef struct
{
    int head;
    int tail;
    int count;
    Message messages[BUFFER_SIZE];
}CircullarBuffer;

//  inicjalizacja bufora
void InitializeBuffer(CircullarBuffer *buffer){
    buffer->head=0;
    buffer->tail=0;
    buffer->count=0;
}

bool isBufferEmpty(CircullarBuffer *buffer){
    return buffer->count<=0;
}

bool isBufferFull(CircullarBuffer *buffer){
    return buffer->count>=BUFFER_SIZE;
}

int addMessage(CircullarBuffer *buffer, Message msg){
    if(!isBufferFull(buffer))
    {
        buffer->messages[buffer->tail]=msg;
        buffer->tail=(buffer->tail+1)%BUFFER_SIZE;
        buffer->count++;
    }
    return 0;
}

int readMessage(CircullarBuffer *buffer, Message *msg){
    if(!isBufferEmpty(buffer)){
        *msg=buffer->messages[buffer->head];
        buffer->head=(buffer->head+1)%BUFFER_SIZE;
        buffer->count--;
    }
    return 0;
}

int main(){
    CircullarBuffer Buffer;
    InitializeBuffer(&Buffer);


//dodanie
    Message snd;
    snd.data_lenght=3;
    snd.data[0]='a';
    snd.data[1]='b';
    snd.data[2]='c';
    addMessage(&Buffer, snd);


//odczytanie
    Message rcv;
    readMessage(&Buffer, &rcv);
    for(int i=0; i<rcv.data_lenght; i++){
        printf("%c\n", rcv.data[i]);
    }

    return 0;
}