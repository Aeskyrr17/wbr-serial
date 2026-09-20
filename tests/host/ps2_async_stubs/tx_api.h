#pragma once
using UINT = unsigned;
using ULONG = unsigned long;
using CHAR = char;
struct TX_SEMAPHORE { unsigned count = 0; };
struct TX_THREAD {};
#define TX_SUCCESS 0U
#define TX_NO_WAIT 0U
#define TX_INTERRUPT_SAVE_AREA
#define TX_DISABLE
#define TX_RESTORE
UINT tx_semaphore_create(TX_SEMAPHORE*, CHAR*, ULONG);
UINT tx_semaphore_delete(TX_SEMAPHORE*);
UINT tx_semaphore_get(TX_SEMAPHORE*, ULONG);
UINT tx_semaphore_put(TX_SEMAPHORE*);
TX_THREAD* tx_thread_identify();
