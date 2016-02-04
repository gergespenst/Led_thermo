/******************************************************************************
  libwh1602b - Library for working with wh1602b compatible lcd display's
  Copyright (c) 2009, Maxim Pshevlotski <mpshevlotsky@gmail.com>

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
******************************************************************************/

#include <avr/io.h>
#include "conf.h"

#include "delay.S"

#define p2h r27
#define p2l r26
#define p1h r23
#define p1l r22
#define p0h r25
#define p0l r24

.macro tx_data x
  ldi p0l,\x
  rcall data
.endm

.global wh1602b_init ; void wh1602b_init();
/* инициализировать дисплей */
wh1602b_init:
  ; настройка линий и их включение
  sbi DDR_RS,NPIN_RS
  sbi DDR_RW,NPIN_RW
  sbi DDR_EN,NPIN_EN
  sbi DDR_DB7,NPIN_DB7
  sbi DDR_DB6,NPIN_DB6
  sbi DDR_DB5,NPIN_DB5
  sbi DDR_DB4,NPIN_DB4
  ; инициализация
  delay_ms 15
  tx_data 0b00110000
  delay_ms 4
  tx_data 0b00110000
  delay_us 100
  tx_data 0b00110000
  tx_data 0b00100000 ; set interface to 4 bit long
  delay_us 39
  ; function set
  tx_data 0b00100000 ; DL = 4bit
  tx_data 0b11000000 ; NLines and Font
  delay_us 39
  ret

.global wh1602b_displayctl ; void wh1602b_displayctl(uint8_t t);
/* настроить дисплей */
wh1602b_displayctl:
  mov r0,p0l
  tx_data 0b00000000
  mov p0l,r0
  rcall data ; D, C, B
  delay_us 39
  ret

.global wh1602b_clear ; void wh1602b_clear();
/* очистить дисплей */
wh1602b_clear:
  tx_data 0b00000000
  tx_data 0b00010000
  delay_ms 2
  ret

.global wh1602b_home ; void wh1602b_home();
/* вернуть курсор в начальное положение */
wh1602b_home:
  tx_data 0b00000000
  tx_data 0b00100000
  delay_ms 2
  ret

.global wh1602b_entrymode ; void wh1602b_entrymode(uint8_t t);
/* установить направление курсора и смещение */
wh1602b_entrymode:
  mov r0,p0l
  tx_data 0b00000000
  mov p0l,r0
  cbr p0l,0b10001111
  rcall data ; I/D SH
  delay_us 39
  ret

.global wh1602b_putc ; void wh1602b_putc(uint8_t c);
/* вывести символ */
wh1602b_putc:
  mov p0h,p0l
  cbr p0h,0b11110000
  swap p0h ; p0h - младшая часть, p0l - старшая
  mov r1,p0h
  sbi PORT_RS,NPIN_RS
  cbi PORT_RW,NPIN_RW
  rcall data
  mov p0l,r1
  rcall data
  cbi PORT_RS,NPIN_RS
  delay_us 43
  clr r1 ; to be always zero in C code
  ret

.global wh1602b_putsP ; void wh1602b_putsP(const char* s);
/* вывести строку из памяти программ на дисплей */
wh1602b_putsP:
  mov ZL,p0l
  mov ZH,p0h
0:lpm p0l,Z+
  tst p0l
  breq 1f
  rcall wh1602b_putc
  rjmp 0b
1:ret

.global wh1602b_puts ; void wh1602b_puts(const char* s);
/* вывести строку из озу на дисплей */
wh1602b_puts:
  mov ZL,p0l
  mov ZH,p0h
0:ld p0l,Z+
  tst p0l
  breq 1f
  rcall wh1602b_putc
  rjmp 0b
1:ret

.global wh1602b_funcset ; void wh1602b_funcset(uint8_t t);
/* установить количество строк и размер шрифта */
wh1602b_funcset:
  mov r0,p0l
  tx_data 0b00100000 ; DL = 4bit
  mov p0l,r0
  cbr p0l,0b00111111
  rcall data ; NLines and Font
  delay_us 39
  ret

.global wh1602b_move ; void wh1602b_move(uint8_t r, uint8_t c);
/* переместить курсор */
wh1602b_move:
  ldi p0h,0x40
  mul p0l,p0h ; r0=r*0x40
  add r0,p1l ; r0+=c
  mov p0l,r0
  sbr p0l,0b10000000
  rcall data
  mov p0l,r0
  cbr p0l,0b11110000
  swap p0l
  rcall data
  delay_us 39
  ret

#ifdef USE_GENC
.global wh1602b_genc ; void wh1602b_genc(uint8_t n, uint8_t* p);
wh1602b_genc:
  movw ZL,p1l ; *p
  ldi p1l,8
  mul p0l,p1l ; r0 - адрес начала
  mov p1h,r0
0:ld p0h,Z+ ; данные
  sbr p0h,0b11100000 ; данные
  mov p0l,p1h ; смещение адреса
  rcall 1f
  inc p1h
  dec p1l
  brne 0b
  ret
1:; p0h - данные, p0l - смещение
  cbr p0l,0b11000000
  sbr p0l,0b01000000
  rcall data ; set cgram, hi
  swap p0l
  rcall data ; set cgram, lo
  delay_us 39
  mov p0l,p0h
  rcall wh1602b_putc
  ret
#endif

data: ; p0l - данные для передачи
  cbi PORT_DB7,NPIN_DB7
  cbi PORT_DB6,NPIN_DB6
  cbi PORT_DB5,NPIN_DB5
  cbi PORT_DB4,NPIN_DB4
  sbrc p0l,7 ; db7?
  sbi PORT_DB7,NPIN_DB7
  sbrc p0l,6 ; db6?
  sbi PORT_DB6,NPIN_DB6
  sbrc p0l,5 ; db5?
  sbi PORT_DB5,NPIN_DB5
  sbrc p0l,4 ; db4?
  sbi PORT_DB4,NPIN_DB4
  ; послать импульс
  sbi PORT_EN,NPIN_EN
  delay_ns 150
  cbi PORT_EN,NPIN_EN
  ret

#ifdef USE_PUTN
/* вывод на индикатор десятичного числа */
.global wh1602b_putn ; void wh1602b_putn(uint16_t n)
wh1602b_putn:
  clt
  ldi ZH,hi8(3f)
  ldi ZL,lo8(3f)
  movw p1l,p0l
  ldi p2l,4
0:rcall func
  brts 1f
  cpi p2l,1
  brlo 1f
  tst p0l
  breq 2f
1:subi p0l,-'0'
  rcall wh1602b_putc
  set
2:dec p2l
  brge 0b
  clr r1 ; to be always zero in C code
  ret
3:; д.тыс., тысячи, сотни, десятки, и еденицы
.word 0x2710, 0x03e8, 0x0064, 0x000a, 0x0001
#endif

#ifdef USE_PUTH
/* вывод на индикатор шестнадцатеричного числа */
.global wh1602b_puth ; void wh1602b_puth(uint8_t w, uint16_t h)
wh1602b_puth:
  clt
  ldi ZH,hi8(3f)
  ldi ZL,lo8(3f)
  mov p2h,p0l ; w
  ldi p2l,3
0:rcall func
  brts 1f
  cp p2l,p2h
  brlo 1f
  tst p0l
  breq 5f
1:cpi p0l,0x0a
  brge 2f
  subi p0l,-'0'
  rjmp 4f
2:subi p0l,0x0a-'A'
4:rcall wh1602b_putc
  set
5:dec p2l
  brge 0b
  clr r1 ; to be always zero in C code
  ret
3:; д.тыс., тысячи, сотни, десятки, и еденицы
.word 0x1000, 0x0100, 0x0010, 0x0001
#endif

#if defined(USE_PUTN) || defined(USE_PUTH)
func:
  clr p0l
  lpm r0,Z+
  lpm r1,Z+
0:sub p1l,r0
  sbc p1h,r1
  brcs 1f
  inc p0l
  rjmp 0b
1:add p1l,r0
  adc p1h,r1
  ret
#endif
