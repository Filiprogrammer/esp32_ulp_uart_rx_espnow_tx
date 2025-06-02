# ESP32 Ultra Low Power UART reader and ESP-NOW sender

When running this program, the ESP32 stays in deep sleep mode and uses the ULP (Ultra Low Power) co-processor to wait for and receive data over UART. During this process the ESP32 consumes less than ~400 uA of current.

Once 246 bytes of data have accumulated, the ULP co-processor wakes up the main processor, which then sends the data along with a sequence number via ESP-NOW to a receiver ESP32. The transmission is retried if it was unsuccessful. Afterwards the ESP32 goes back to deep sleep.

### The expected UART parameters are:

* **8** data bits
* **Even** parity bit
* **1** stop bit
* Baud rate: **2400**

### Why is the buffer size 246 bytes?

The ESP-NOW protocol allows for a maximum message size of 250 bytes. Since this program uses the first 4 bytes for a sequence number, that leaves 246 bytes available for actual data.

## Build & Upload instructions

With ESP-IDF:
```
idf.py build
idf.py flash
```

With PlatformIO:
```
pio run --target upload
```

## TODO:

* Stop the ULP co-processor instead of calling the `WAIT` instruction and wake it up periodically with `ulp_set_wakeup_period`. This could reduce idle current consumption below 50 uA.
* Make the baud rate configurable
