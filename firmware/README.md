# FIRMWARE (Low-Level)

Folder zawiera kod logiki niskopoziomowej, sterowanie napędem oraz obsługę czujników uruchamiane na mikrokontrolerze (**STM32 NUCLEO-H755ZI-Q**).

---

### Środowisko i Zależności
* **OS:** FreeRTOS
* **Język:** C
* **Biblioteki:** 
	* `micro-ROS`
    * `CMSIS-DSP`
    * `LwIP`
    * `drivers for sensors etc..`
* **Komunikacja**
    * `i2c`
    * `spi`
    * `udp`
