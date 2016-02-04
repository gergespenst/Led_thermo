
#define F_CPU 20000000 /* частота мк, Гц */

#define USE_PUTN /* включить поддержку wh1602b_putn() */
#define USE_PUTH /* включить поддержку wh1602b_puth() */
#define USE_GENC /* включить поддержку wh1602b_genc() */

/* ножки к которым подключены выводы RS, RW, EN */
#define NPIN_RS PA6
#define NPIN_RW PA5
#define NPIN_EN PA4

/* порты к которым подключены выводы RS, RW, EN */
#define PORT_RS _SFR_IO_ADDR(PORTA)
#define PORT_RW _SFR_IO_ADDR(PORTA)
#define PORT_EN _SFR_IO_ADDR(PORTA)
#define DDR_RS  _SFR_IO_ADDR(DDRA)
#define DDR_RW  _SFR_IO_ADDR(DDRA)
#define DDR_EN  _SFR_IO_ADDR(DDRA)

/* ножки к которым подключены выводы DB4-DB7 */
#define NPIN_DB7  PA0
#define NPIN_DB6  PA1
#define NPIN_DB5  PA2
#define NPIN_DB4  PA3

/* порты к которым подключены выводы DB4-DB7 */
#define PORT_DB7  _SFR_IO_ADDR(PORTA)
#define PORT_DB6  _SFR_IO_ADDR(PORTA)
#define PORT_DB5  _SFR_IO_ADDR(PORTA)
#define PORT_DB4  _SFR_IO_ADDR(PORTA)
#define DDR_DB7   _SFR_IO_ADDR(DDRA)
#define DDR_DB6   _SFR_IO_ADDR(DDRA)
#define DDR_DB5   _SFR_IO_ADDR(DDRA)
#define DDR_DB4   _SFR_IO_ADDR(DDRA)
