/*
 * packet.h
 *
 *  Created on: 26-Apr-2009
 *      Author: Neil MacMillan
 */

#ifndef PACKET_H_
#define PACKET_H_

#include <stdint.h>

/*****					Add labels for the packet types to the enumeration					*****/



/*****							Construct payload format structures							*****/

// structures must be 29 bytes long or less.


/*****							Add format structures to the union							*****/

/// The application-dependent packet format.  Add structures to the union that correspond to the packet types defined
/// in the PACKET_TYPE enumeration.  The format structures may not be more than 29 bytes long.  The _filler array must
/// be included to ensure that the union is exactly 29 bytes long.


/*****						Leave the radiopacket_t structure alone.						*****/

typedef struct _rp {
    uint8_t payload[32];
} radiopacket_t;

#endif /* PACKET_H_ */
