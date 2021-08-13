#ifndef EMBEDDEDSYSTEMS18_REF_MESSAGE_H
#define EMBEDDEDSYSTEMS18_REF_MESSAGE_H

//      Mnemonic    ID        Payload       Payload Length
#define HELLO       0x42    //Robot’s id    8 bit
#define CONFIG      0x43    //Channel       8 bit
#define GO          0x44    //none          0 bit
#define END         0x45    //Outcome       8 bit
#define PING        0x50    //Nonce n       16 bit
#define PONG        0x51    //Nonce n+1     16 bit
#define POS         0x60    //θ, x, and y   48 bit
#define OOB         0x61    //θ, x, and y   48 bit
#define COLL        0x62    //θ, x, and y   48 bit
#define MSG         0x70    //Any message   variable

#endif //EMBEDDEDSYSTEMS18_REF_MESSAGE_H
