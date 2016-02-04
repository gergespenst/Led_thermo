#include "wh1602b/wh1602b.h"
#include "ds18b20/ds18b20.h"
#include <avr/pgmspace.h>

void conf_my_lcd();

int main(void)
{
  conf_my_lcd(); // настроить вывод на жк-дисплей

  uint8_t id[8];
  if (!ds18b20_readROM(id)) { // прочитать уникальный код датчика в id
    wh1602b_putsP(PSTR("Ошибка CRC!")); // если ошибка
    for (;;)
      ;
  }
  wh1602b_putsP(PSTR("ROM Code ="));
  wh1602b_move(1,0);
  uint8_t i;
  for (i = 0; i < 8; ++i)
    wh1602b_puth(2, id[i]);

  for (;;)
    ;
}

void conf_my_lcd()
{
  wh1602b_init(); // инициализировать модуль, порты
  // настроить параметры управления дисплеем
  wh1602b_displayctl(DC_DISPLAY_ON); // дисплей включён
  // задать число строк
  wh1602b_funcset(FS_LINES_2); // 2 строки
  // очистить область
  wh1602b_clear();
}
