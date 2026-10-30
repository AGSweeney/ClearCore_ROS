/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */

#ifndef __CCROS_TRANSPORT_H__
#define __CCROS_TRANSPORT_H__

#include <stdbool.h>
#include <stdint.h>

void TransportInitUsb();
void TransportInitEthernet();
void TransportPoll();
void TransportSendLine(const char *line);
/* USB only. Setup menu text must not reach the TCP session. */
void TransportUsbSend(const char *text);
void TransportUsbSendLine(const char *line);
void TransportSetUsbInteractive(bool on);
const char *TransportIpString();
/* fromUsb may be null. USB returns blank lines; TCP skips them. */
bool TransportReadLine(char *outLine, uint16_t maxLen, bool *fromUsb);
/* Session/stream Accept and Close counts since boot (TcpData ownership). */
void TransportTcpCounters(uint32_t *sessionAccepts, uint32_t *sessionCloses,
                          uint32_t *streamAccepts, uint32_t *streamCloses);

#endif
