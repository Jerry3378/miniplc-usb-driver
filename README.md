# miniplc-usb-driver

## Background
This project originated from a real-world debugging experience where an intermittent data corruption bug in an industrial PLC driver was traced to a driver-level address arithmetic error. The APNX protocol was designed to address reliability gaps observed during that analysis — incorporating Fail-Fast address validation and explicit error codes in NAK responses.

## Protocol: APNX

APNX is a custom request-response protocol over USB Bulk transfer,
inspired by Modbus RTU framing.

### Frame Structure (Request)

| Field     | Size     | Description                        |
|-----------|----------|------------------------------------|
| STX       | 1B       | Start byte (0x02)                  |
| LEN       | 1B       | Total packet length                |
| ID        | 1B       | Device ID                          |
| CMD       | 1B       | READ(0) / WRITE(1)                 |
| CMD_TYPE  | 1B       | Address mode (Single: 0x01)        |
| DATA_TYPE | 1B       | 8 / 16 / 32 / 64-bit               |
| COUNT     | 1B       | Number of entries                  |
| PAYLOAD   | Variable | READ: ADDR×n / WRITE: (ADDR+DATA)×n |
| CRC       | 2B       | CRC16 (Modbus RTU, Big Endian)     |

### Response Types

- **ACK_READ (0x05)**: Returns requested data
- **ACK_WRITE (0x06)**: Confirms write success  
- **NAK (0x0F)**: Returns 2-byte error code

### Error Code Hierarchy

| Prefix | Category        |
|--------|-----------------|
| 0x00xx | Command error   |
| 0x01xx | Data error      |
| 0x02xx | Address error   |
| 0x03xx | Frame error     |
| 0xFFxx | System error    |

### Memory Map

| Region | Base   | Size |
|--------|--------|------|
| INPUT  | 0x100  | 16B  |
| OUTPUT | 0x200  | 16B  |
| DATA   | 0x300  | 64B  |
| FLAG   | 0x400  | 32B  |

Address validation uses Fail-Fast pre-check:
addr_type = address & 0xF00 — invalid range returns NAK immediately.

All multi-byte values transmitted in Big Endian.
Device-side conversion to Little Endian is handled internally.

## Test Results
<img width="1132" height="1774" alt="image" src="https://github.com/user-attachments/assets/1d29a06a-bb7f-412e-9f73-c2b0aee37c29" />


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
