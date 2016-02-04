#include "wh1602b/wh1602b.h"
#include "ds18b20/ds18b20.h"
#include <avr/pgmspace.h>

void conf_my_lcd();
void loop();

int main(void)
{
  conf_my_lcd(); // настроить вывод на жк-дисплей
  if (!ds18b20_selectSensor(0)) { // выбрать текущий датчик
    wh1602b_putsP(PSTR("Ошибка на линии!"));
    for (;;)
      ;
  }
  for (;;)
    loop(); // цикл обработки
}

void loop()
{
  ds18b20_convert();  // попросить у датчика измерить температуру
  wh1602b_move(0,0);
  wh1602b_putsP(PSTR("t="));
  int16_t t = ds18b20_readTemp(); // прочитать значение
  if (t < 0) // вывести минус если отрицательно
    wh1602b_putc('-');
  wh1602b_puth(1, ds18b20_extractInt(t)); // вывести целую часть на дисплей
  wh1602b_putc('.');
  wh1602b_puth(1, ds18b20_extractFract(t, 1)); // вывести дробную часть на дисплей
  wh1602b_putsP(PSTR("\1C"));
}

void conf_my_lcd()
{
  const uint8_t degree_char[8] = {  // знак градуса
    0x0e, // 0 1 1 1 0
    0x0a, // 0 1 0 1 0
    0x0e, // 0 1 1 1 0
    0x00, // 0 0 0 0 0
    0x00, // 0 0 0 0 0
    0x00, // 0 0 0 0 0
    0x00, // 0 0 0 0 0
    0x00  // 0 0 0 0 0
  };
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
}
