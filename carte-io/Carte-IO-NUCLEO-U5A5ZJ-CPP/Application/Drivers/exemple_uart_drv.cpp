#include <Drivers/exemple_uart_drv.h>
#include <cstring>

// Implémentation du constructeur
UartManager::UartManager(UART_HandleTypeDef& uartHandle) : _uart(uartHandle) {
    _uart.configure(115200);

    // Enregistrement des méthodes d'instance comme callbacks
    _uart.setTxCompleteCallback([this]() { onTxComplete(); });
    _uart.setRxCompleteCallback([this]() { onRxComplete(); });
    _uart.setErrorCallback([this]() { onError(); });
}

// Fonction pour envoyer un message
void UartManager::sendMessage(const char* msg) {
    _uart.sendDataPolling((uint8_t*)msg, strlen(msg), 1000);
}

// Callbacks privés
void UartManager::onTxComplete() {
    printf("Transmission terminée !\n");
}

void UartManager::onRxComplete() {
    printf("Réception terminée !\n");
}

void UartManager::onError() {
    printf("Erreur de communication !\n");
}

