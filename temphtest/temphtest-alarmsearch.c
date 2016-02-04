#define F_CPU 20000000ULL
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "wh1602b/wh1602b.h"
#include "ds18b20/ds18b20.h"

void conf_my_lcd();
void loop();

int main(void)
{
  const uint8_t devid[8] = {0x11,0x00,0x00,0x01,0x78,0xDB,0xC8,0x28};
  sspad d;
  conf_my_lcd(); // настроить вывод на жк-дисплей
  ds18b20_selectSensor(devid); // выбрать датчик
  /* установить допустимый температурный порог */
  ds18b20_readScratchpad(&d);
  d.t_h = 25; // верхний допустимый порог
  d.t_l = 20; // нижний допустимый порог
  ds18b20_writeScratchpad(&d);
  for (;;)
    loop();  // цикл обработки
}

void loop()
{
  uint8_t i, dc = 0, id[8];
  int8_t r;

  ds18b20_convert(); // измерить температуру на текущем датчике
  // поиск датчиков с отклонением пороговых значений температуры
  while ((r = ds18b20_searchAlarm(id))) {
    if (r < 0)
      continue; // повтор если ошибка crc
    wh1602b_move(0,0);
    // вывести rom-id датчика на дисплей
    for (i = 0; i < 8; ++i)
      wh1602b_puth(2, id[i]);
     _delay_ms(2000); // пауза 2с.
    ++dc;
  }
  wh1602b_move(1,0);
  wh1602b_putn(dc); // вывести число датчиков с отклонениями
  wh1602b_putsP(PSTR(" alarms."));
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
