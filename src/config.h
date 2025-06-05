#include <soc/gpio_num.h>

#define GPIO_RX GPIO_NUM_27
#define RECEIVER_ADDRESS {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}
#define MAX_RETRANSMITS 1 // Max number of retries for a packet (Do not increase this beyond 3, because it could mess up the timing with the ULP serial reader if we take too long (assuming we get a UART message every 5 seconds))
//#define DEBUG
