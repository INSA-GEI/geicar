#include "Drivers/uartdriver.h"
#include <cstdio>

class UartManager {
public:
    UartManager(UART_HandleTypeDef& uartHandle);
    void sendMessage(const char* msg);

private:
    UartDriver _uart;

    // Callbacks privés
    void onTxComplete();
    void onRxComplete();
    void onError();
};
