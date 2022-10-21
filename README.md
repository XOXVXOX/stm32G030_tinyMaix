# Port TinyMaix to STM32G030

## Chip

| Item  | Parameter              |
| ----- | ---------------------- |
| Chip  | STM32G030               |
| Arch  | RV32 IMAC (QingKe V3A) |
| Freq  | 64M                    |
| Flash | 64KB                   |
| RAM   | 20KB                   |

## Board 

CH32V103R-R0-1v2

<a href="assets/CH32V103R.jpg"><img width=300 src="assets/CH32V103R.jpg"/></a>

## Development Environment

MounRiver Studio

## Step/Project

Just simple edit `tm_port.h`

https://github.com/dreamcmi/CH32V103-TinyMaix

## Result

| config | mnist | Note |
| ------ | ----- | ---- |
| O0 CPU | 18    |      |



## Author

[Darren Cheng](https://github.com/dreamcmi) 