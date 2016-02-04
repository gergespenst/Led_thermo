#include "wh1602b/wh1602b.h"
#include "ds18b20/ds18b20.h"
#include <avr/pgmspace.h>

// знак градуса
static const uint8_t degree_char[8] = {
  0x0e, // 0 1 1 1 0
  0x0a, // 0 1 0 1 0
  0x0e, // 0 1 1 1 0
  0x00, // 0 0 0 0 0
  0x00, // 0 0 0 0 0
  0x00, // 0 0 0 0 0
  0x00, // 0 0 0 0 0
  0x00  // 0 0 0 0 0
};

const uint8_t devid[8] = {0x11,0x00,0x00,0x01,0x78,0xDB,0xC8,0x28};

void loop();

int main(void)
{
  wh1602b_init(); // инициализировать модуль, порты
  // настроить параметры управления дисплеем
  wh1602b_displayctl(DC_DISPLAY_ON); // дисплей включён
  // задать число строк
  wh1602b_funcset(FS_LINES_2); // 2 строки
  // очистить область
  wh1602b_clear();
  // записать символ градуса ассоциировав его с кодом 1
  wh1602b_genc(1, degree_char);
  wh1602b_home();
  // выбрать текущий датчик
  if (!ds18b20_selectSensor(devid)) {
    wh1602b_putsP(PSTR("Ошибка на линии!"));
    for (;;)
      ;
  }
  sspad d;
  if (!ds18b20_readScratchpad(&d)) {
    wh1602b_putsP(PSTR("Ошибка CRC!"));
    for (;;)
      ;
  }
  if (d.config != RES_11BIT) {
    d.config = RES_11BIT;
    ds18b20_writeScratchpad(&d);
    ds18b20_copyScratchpad();
    wh1602b_move(1,0);
    wh1602b_putsP(PSTR("Уст:11б. "));
  }
  else {
    wh1602b_move(1,0);
    wh1602b_putsP(PSTR("Исп:11б. "));
  }
  if (ds18b20_readPowerSupply())
    wh1602b_putsP(PSTR("Внешнее"));
  else
    wh1602b_putsP(PSTR("Паразит"));

  // войти в бесконечный цикл
  for (;;)
    loop();
}

void loop()
{
  // попросить у датчика измерить температуру
  ds18b20_convert();  // время измерения ~750ms
  wh1602b_move(0,0);
  wh1602b_putsP(PSTR("t="));
  int16_t t = ds18b20_readTemp(); // прочитать значение
  if (t < 0)
    wh1602b_putc('-');
  wh1602b_puth(1, ds18b20_extractInt(t)); // вывести целую часть
  wh1602b_putc('.');
  wh1602b_puth(1, ds18b20_extractFract(t, 1)); // вывести дробную часть
  wh1602b_putsP(PSTR("\1C"));
}
