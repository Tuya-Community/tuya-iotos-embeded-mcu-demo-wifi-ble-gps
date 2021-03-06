# Tuya IoTOS Embedded MCU Demo Wifi Ble GPS

[English](./README.md) | [中文](./README_zh.md)

## Introduction  

This Demo uses the Tuya smart cloud platform, Tuya smart APP, and IoTOS Embedded MCU SDK to realize GPS positioning.

The implemented features include:

+ GPS positioning


## Quick start  

### Migration

+ [Get MCU SDK](https://developer.tuya.com/en/docs/iot/device-development/embedded-software-development/mcu-development-access/mcu-access-guide?id=K9hrbv1ub5owo#title-2-产品创建)
+ [MCU SDK Migration](https://developer.tuya.com/en/docs/iot/device-development/embedded-software-development/mcu-development-access/wifi-mcu-sdk-solution/overview-of-migrating-tuyas-mcu-sdk?id=K9hhi0xr5vll9)


### File introduction 

```
├── user
│   ├── main.c
│   └── MY_ST_config.h
├── CMSIS
│   ├── system_stm32g0xx.c
│   └── startup_stm32g071xx.s
├── SYSTEM
│   ├── sys.c
│   ├── sys.h
│   ├── RCC.c
│   ├── RCC.h
│   ├── delay.c
│   ├── delay.h
│   ├── USART.c
│   ├── USART.h
│   ├── IO.c
│   ├── IO.h
│   ├── TIM.c
│   └── TIM.h
└── SYSTEM
    ├── mcu_api.c
    ├── mcu_api.h
    ├── protocol.c
    ├── protocol.h
    ├── system.c
    ├── system.h
    └── wifi.h
    
```



### Demo entry

Entry file：main.c

Important functions：main()

+ Initialize and configure MCU IO port, USART, timer, etc. All events are polled and judged in while(1)。




### DataPoint related

+ DP point processing: mcu_dp_string_update()

| function name | unsigned char mcu_dp_string_update(unsigned char dpid,const unsigned char value[],unsigned short len) |
| ------------- | ------------------------------------------------------------ |
| dpid          | DP ID number                                                 |
| value         | DP data                                                      |
| len           | the length of data                                           |
| Return        | SUCCESS: Success ERROR: Failure                              |



### I/O List  

| UASRT2  | UASRT1  |
| :-----: | :-----: |
| PD5 TXD | PC4 TXD |
| PD6 RXD | PC5 RXD |

## 

## Related Documents

Tuya Demo Center: https://developer.tuya.com/demo/



## Technical Support

  You can get support for Tuya by using the following methods:

- Developer Centre: https://developer.tuya.com/
- Help Centre: https://support.tuya.com/help
- Technical Support Work Order Centre: [https://service.console.tuya.com](https://service.console.tuya.com/)

