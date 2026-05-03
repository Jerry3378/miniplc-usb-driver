# miniplc-usb-driver

## Background
This project originated from a real-world debugging experience where an intermittent data corruption bug in an industrial PLC driver was traced to a driver-level address arithmetic error. The APNX protocol was designed to address reliability gaps observed during that analysis — incorporating Fail-Fast address validation and explicit error codes in NAK responses.

## Architecture
- Protocol frame: STX / LEN / ID / CMD / ADDR / DATA / CRC16
- FSM-based packet parsing (IDLE → READ_LEN → READ_PAYLOAD → VERIFY_CRC)
- Kernel-side: URB, DMA (usb_alloc_coherent), kfifo ring buffer
- Synchronization: spinlock (IRQ context), mutex (process context), semaphore (write throttle)

## Driver Lifecycle
probe → open → read/write → release → disconnect

Each stage manages USB endpoint setup, URB submission, 
kfifo-based buffering, and safe resource cleanup on disconnect.

## Build
make
sudo insmod miniPLC_driver.ko

## Environment
- Linux kernel 6.12.75
- ATmega32U4 (LUFA stack, Full-speed USB)
