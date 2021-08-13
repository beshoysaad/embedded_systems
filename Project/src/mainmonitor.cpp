#include <stdio.h>
#include <stdlib.h>

#define DEBUG   1

#if DEBUG == 1
#define DEBUGMSG(...)   printf(__VA_ARGS__)
#else
#define DEBUGMSG(...)
#endif
int messagesent(int sshp) {

    if (sshp == 0) {

        DEBUGMSG("Warning: waiting for position message to be sent\r\n");
        return 1;
    } else {
        DEBUGMSG("OK: position message was sent, waiting no more.\r\n");
        return 0;

    }

}
int ProxOrHarvest(int cps, int dhp) {
    while (1) {
        if (dhp == 0) {
            if (cps == 0) {
                DEBUGMSG(
                        "ERROR: I am not at a harvesting position and neither am I checking proximity sensors, haha\r\n");
                return 2;
                break;
            } else
                DEBUGMSG("OK: checking my proximity sensors since I'm not at a decent harvesting position.\r\n");
            return 0;

        } else {
            if (cps == 1) {

                DEBUGMSG("Warning: at a decent harvesting position checking proximity sensors(unneeded overhead)\r\n");
                return 1;
            } else {

                DEBUGMSG("OK: harvesting, do not disturb.\r\n");
                return 0;
            }

        }

    }

}
int pingmsg(int iPm) {
    while (1) {
        if (iPm == 0) {

            DEBUGMSG("OK: PING message answered\r\n");
            pingmsg(iPm);
            return 0;
        } else if (iPm == 0) {
            pingmsg(iPm);
            DEBUGMSG("Warning: PING message answered after 1 ignore.\r\n");
            return 1;
        } else if (iPm == 0) {

            pingmsg(iPm);
            DEBUGMSG("Warning: PING message answered after 2 ignores\r\n");
            return 1;
        } else {
            DEBUGMSG("Error: 3 consequent PING messages ignored\r\n");
            return 2;
            break;
        }
    }
}
int emptybuffer(int time, int bufferemptied) {
    if (bufferemptied == 0) {
        if (time < 500)
            return "OK: buffer has been emptied\r\n";
        else {

            return 2;
            DEBUGMSG("Error: buffer has been emptied too late.\r\n");
        }
    } else if (time < 500) {
        return 1;
        DEBUGMSG("Warning: buffer was not emptied for some time\r\n");
    } else {

        return 2;
        DEBUGMSG("Error: buffer has not been emptied for last 500 seconds \r\n");
    }
}

int main() {
    int i = 0;
    while (i < 20) {
        if (i % 2 == 0) {
            messagesent(0);
            ProxOrHarvest(0, 0);
        } else {
            messagesent(1);
            ProxOrHarvest(1, 1);

        }
        i++;
    }

    return 0;
}
