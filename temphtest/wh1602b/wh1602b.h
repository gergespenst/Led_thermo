/******************************************************************************
  libwh1602b - Library for working with wh1602b compatible lcd display's
  Copyright (c) 2009, Maxim Pshevlotski <mpshevlotsky@gmail.com>

  This file is part of libwh1602b.

  libwh1602b is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  libwh1602b is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with libwh1602b.  If not, see <http://www.gnu.org/licenses/>.
******************************************************************************/

#include <stdint.h>

// выполняет инициализацию модуля дисплея
void wh1602b_init();
// очищает дисплей
void wh1602b_clear();
// выводит символ на экран дисплея
void wh1602b_putc(uint8_t c);
// перемещает курсор
void wh1602b_move(uint8_t r, uint8_t c);
// возвращает курсор в начальное положение
void wh1602b_home();
// записывает символ в знакогенератор
void wh1602b_genc(uint8_t n, const uint8_t* p);
// выводит строку из озу на дисплей
void wh1602b_puts(const char* s);
// выводит строку из памяти программ на дисплей
void wh1602b_putsP(const char* s);
// выводит десятичное число на дисплей
void wh1602b_putn(uint16_t n);
// выводит шестнадцатеричное число на дисплей
void wh1602b_puth(uint8_t w, uint16_t h);

#define DC_DISPLAY_ON    0xC0
#define DC_CURSOR_BLINK  0xB0
#define DC_CURSOR_ON     0xA0
#define DC_DISPLAY_OFF   0x00
// устанавливает параметры управления дисплеем
void wh1602b_displayctl(uint8_t t);

#define EM_CURSOR_DIR    0x60
#define EM_DISPLAY_SHIFT 0x50
// настраивает режима ввода
void wh1602b_entrymode(uint8_t t);

#define FS_LINES_2   0x80
#define FS_FONT_5x11 0x40
#define FS_LINES_1   0x00
// устанавливает число строк и размера шрифта
void wh1602b_funcset(uint8_t t);
