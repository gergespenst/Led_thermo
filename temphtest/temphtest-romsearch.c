#define F_CPU 20000000ULL /* частота мк, гц (для delay) */
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "wh1602b/wh1602b.h"
#include "ds18b20/ds18b20.h"

void conf_my_lcd();
void loop();

int main(void)
{
  conf_my_lcd(); // настроить вывод на жк-дисплей
  for (;;)
    loop();  // цикл обработки
}

void loop()
{
  uint8_t i, dc = 0, id[8];
  int8_t r;
  // поиск устройств на шине 1-wire
  while ((r = ow_searchROM(id))) {
    if (r < 0)
     continue; // повтор если ошибка crc
    wh1602b_move(0,0);
    // вывести уникальный код датчика на дисплей
    for (i = 0; i < 8; ++i)
      wh1602b_puth(2, id[i]);
    _delay_ms(2000); // пауза 2с.
    ++dc;
  }
  wh1602b_move(1,0);
  wh1602b_putn(dc); // вывести число датчиков
  wh1602b_putsP(PSTR(" devices."));
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