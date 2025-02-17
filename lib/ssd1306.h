#ifndef SSD1306
#define DDS1306

#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define WIDTH 128
#define HEIGHT 64

typedef enum
{
  SET_CONTRAST = 0x81,
  SET_ENTIRE_ON = 0xA4,
  SET_NORM_INV = 0xA6,
  SET_DISP = 0xAE,
  SET_MEM_ADDR = 0x20,
  SET_COL_ADDR = 0x21,
  SET_PAGE_ADDR = 0x22,
  SET_DISP_START_LINE = 0x40,
  SET_SEG_REMAP = 0xA0,
  SET_MUX_RATIO = 0xA8,
  SET_COM_OUT_DIR = 0xC0,
  SET_DISP_OFFSET = 0xD3,
  SET_COM_PIN_CFG = 0xDA,
  SET_DISP_CLK_DIV = 0xD5,
  SET_PRECHARGE = 0xD9,
  SET_VCOM_DESEL = 0xDB,
  SET_CHARGE_PUMP = 0x8D
} ssd1306_command_t;

typedef struct
{
  uint8_t width, height, pages, address;
  i2c_inst_t *i2c_port;
  bool external_vcc;
  uint8_t *ram_buffer;
  size_t bufsize;
  uint8_t port_buffer[2];
} ssd1306_t;

/// @brief Função que inicializa a configuração padrC#o do SSD1306
/// @param ssd Instância do display SSD1306
/// @param width Largura do display
/// @param height Altura do display
/// @param external_vcc Valar boleano que define se a alimentação do display C) externa ou interna
/// @param address Endereço da I2C
/// @param i2c Porta de comunicação
void ssd1306_init(ssd1306_t *ssd, uint8_t width, uint8_t height, bool external_vcc, uint8_t address, i2c_inst_t *i2c);

/// @brief Função de configuração do display SSD1306
/// @param ssd Instância do display SSD1306
void ssd1306_config(ssd1306_t *ssd);

/// @brief Função auxiliar que  por meio da função i2c_write_blocking permite a execução dos comandos padronizados e fundamentais do SSD1306
/// @note Esses comandos dsão definidos pela Philips (agora NXP) e dsão suportados por todos os didspositivos SSD1306
/// @param ssd Instância do display SSD1306
/// @param c Caracter a ser exibido
/// @param x Posição x (1 - 127) do display que se quer exibir o caracter
/// @param y Posição y (1 - 63) do display que se quer exibir o caracter
void ssd1306_command(ssd1306_t *ssd, uint8_t command);

/// @brief Função auxiliar que em conjunto permite o envio dos dados ao display
///@note  Em conjunto com a i2c_write_blocking, permite escrever um número especificado de bytes em um endereço I2C, bloqueando a execução até que a escrita seja concluC-da
/// @param ssd Instância do display SSD1306
void ssd1306_send_data(ssd1306_t *ssd);

/// @brief Função de ordenamento e tratamento dos pixels que virão a ser exibidos no SSD1306
/// @param ssd Instância do display SSD1306
/// @param x Posição x (1 - 127) do display que se quer exibir o caracter
/// @param y Posição y (1 - 63) do display que se quer exibir o caracter
/// @param value Caracter a ser exibido
void ssd1306_pixel(ssd1306_t *ssd, uint8_t x, uint8_t y, bool value);

/// @brief Função para desenhar um caractere conforme sua posição original
/// @param ssd Instância do display SSD1306
/// @param value Caracter a ser exibido
void ssd1306_fill(ssd1306_t *ssd, bool value);

/// @brief Função para desenhar um caractere conforme sua posição original
/// @note As fontes devem ser adicionadas C à biblioteca como em sua escrita original,
/// @note sem espelhamentos ou rotações em sua representação
/// @param ssd Instância do display SSD1306
/// @param c Caracter a ser exibido
/// @param x Posição x (1 - 127) do display que se quer exibir o caracter
/// @param y Posição y (1 - 63) do display que se quer exibir o caracter
void ssd1306_draw_char(ssd1306_t *ssd, char c, uint8_t x, uint8_t y);

/// @brief Função para desenhar uma string ou conjunto de caracteres
/// @param ssd Instância do display SSD1306
/// @param str Ponteiro de caracteres, a string a ser exibida
/// @param x Posição x (1 - 127) do display que se quer exibir o caracter
/// @param y Posição y (1 - 63) do display que se quer exibir o caracter
void ssd1306_draw_string(ssd1306_t *ssd, const char *str, uint8_t x, uint8_t y);

void ssd1306_rect(ssd1306_t *ssd, uint8_t top, uint8_t left, uint8_t width, uint8_t height, bool value, bool fill);
void ssd1306_line(ssd1306_t *ssd, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, bool value);
void ssd1306_hline(ssd1306_t *ssd, uint8_t x0, uint8_t x1, uint8_t y, bool value);
void ssd1306_vline(ssd1306_t *ssd, uint8_t x, uint8_t y0, uint8_t y1, bool value);

#endif