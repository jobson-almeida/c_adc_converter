#include <stdio.h>        // Biblioteca padrão de entrada e saída
#include "pico/stdlib.h"  // Biblioteca padrão do Raspberry Pi Pico
#include "pico/bootrom.h" // Biblioteca de acesso a funções e dados no bootrom
#include "hardware/adc.h" // Biblioteca para manipulação do ADC no RP2040
#include "hardware/irq.h" // Biblioteca para controle de interrupções no RP2040
#include "hardware/pwm.h" // Biblioteca para controle de PWM no RP2040

// Definição dos pinos usados para o joystick e LEDs
#define VRX 26          // Pino de leitura do eixo X do joystick (conectado ao ADC)
#define VRY 27          // Pino de leitura do eixo Y do joystick (conectado ao ADC)
#define ADC_CHANNEL_0 0 // Canal ADC para o eixo X do joystick
#define ADC_CHANNEL_1 1 // Canal ADC para o eixo Y do joystick
#define SW 22           // Pino de leitura do botão do joystick

#define LED_G 11 // Pino para controle do LED verde
#define LED_B 12 // Pino para controle do LED azul via PWM
#define LED_R 13 // Pino para controle do LED vermelho via PWM

#define BUTTON_BOOT 6 // GPIO definida para uso do botão de bootsel - entrar em modo gravação
#define BUTTON_A 5    // GPIO definida para o botão com a função de habilitar e desabilitar o PWM nos LEDs vermelho e azul

const float DIVIDER_PWM = 16.0; // Divisor fracional do clock para o PWM
const uint16_t PERIOD = 4096;   // Período do PWM (valor máximo do contador)

uint16_t led_b_level, led_r_level = 100; // Inicialização dos níveis de PWM para os LEDs
uint slice_led_b, slice_led_r;           // Variáveis para armazenar os slices de PWM correspondentes aos LEDs
uint32_t last_time = 0;                  // variável de tempo, auxiliar À comtramedida deboucing

// handler de interrupção dos botões
void button_interruption_gpio_irq_handler(uint gpio, uint32_t events)
{
  uint32_t current_time = to_us_since_boot(get_absolute_time());
  // verificar se passou tempo o bastante desde o último evento
  if (current_time - last_time > 250000) // 250 ms de debouncing
  {
    last_time = current_time; // atualiza o tempo do último evento

    if (gpio_get(BUTTON_BOOT) == 0)
    {
      // habilita o bootsel da placa - reinicia em modo gravação
      reset_usb_boot(0, 0);
    }

    if (gpio_get(BUTTON_A) == 0)
    {
      // habilita e desabilita o PWM dos LEDs vermelho e azul
    }

    if (gpio_get(SW) == 0) ////////////////////////////////////
    {
      // altera o estado do LED verde (ligado/desligado).
    }
  }
  gpio_acknowledge_irq(gpio, events); // limpa a interrupção
}

// Função para configurar o joystick (pinos de leitura e ADC)
void setup_joystick()
{
  // Inicializa o ADC e os pinos de entrada analógica
  adc_init();         // Inicializa o módulo ADC
  adc_gpio_init(VRX); // Configura o pino VRX (eixo X) para entrada ADC
  adc_gpio_init(VRY); // Configura o pino VRY (eixo Y) para entrada ADC

  // Inicializa o pino do botão do joystick
  gpio_init(SW);             // Inicializa o pino do botão
  gpio_set_dir(SW, GPIO_IN); // Configura o pino do botão como entrada
  gpio_pull_up(SW);          // Ativa o pull-up no pino do botão para evitar flutuações
}

// Função para configurar o PWM de um LED (genérica para azul e vermelho)
void setup_pwm_led(uint led, uint *slice, uint16_t level)
{
  gpio_set_function(led, GPIO_FUNC_PWM); // Configura o pino do LED como saída PWM
  *slice = pwm_gpio_to_slice_num(led);   // Obtém o slice do PWM associado ao pino do LED
  pwm_set_clkdiv(*slice, DIVIDER_PWM);   // Define o divisor de clock do PWM
  pwm_set_wrap(*slice, PERIOD);          // Configura o valor máximo do contador (período do PWM)
  pwm_set_gpio_level(led, level);        // Define o nível inicial do PWM para o LED
  pwm_set_enabled(*slice, true);         // Habilita o PWM no slice correspondente ao LED
}

// Função de configuração geral
void setup()
{
  stdio_init_all();                                // Inicializa a porta serial para saída de dados
  setup_joystick();                                // Chama a função de configuração do joystick
  setup_pwm_led(LED_B, &slice_led_b, led_b_level); // Configura o PWM para o LED azul
  setup_pwm_led(LED_R, &slice_led_r, led_r_level); // Configura o PWM para o LED vermelho

  gpio_init(LED_G);
  gpio_set_dir(LED_G, GPIO_OUT);
  gpio_put(LED_G, false);

  gpio_init(BUTTON_A);
  gpio_set_dir(BUTTON_A, GPIO_IN);
  gpio_pull_up(BUTTON_A);

  gpio_init(BUTTON_BOOT);
  gpio_set_dir(BUTTON_BOOT, GPIO_IN);
  gpio_pull_up(BUTTON_BOOT);
}

// Função para ler os valores dos eixos do joystick (X e Y)
void joystick_read_axis(uint16_t *vrx_value, uint16_t *vry_value)
{
  // Leitura do valor do eixo X do joystick
  adc_select_input(ADC_CHANNEL_0); // Seleciona o canal ADC para o eixo X
  sleep_us(2);                     // Pequeno delay para estabilidade
  *vrx_value = adc_read();         // Lê o valor do eixo X (0-4095)

  // Leitura do valor do eixo Y do joystick
  adc_select_input(ADC_CHANNEL_1); // Seleciona o canal ADC para o eixo Y
  sleep_us(2);                     // Pequeno delay para estabilidade
  *vry_value = adc_read();         // Lê o valor do eixo Y (0-4095)
}

// Função principal
int main()
{
  uint16_t vrx_value, vry_value, sw_value; // Variáveis para armazenar os valores do joystick (eixos X e Y) e botão
  setup();                                 // Chama a função de configuração

  // habilita as interrupções para os botão de boot, de ativação do pwm dos LEDs
  // e do botão de controle do LED verde e bordas exibidas no display
  gpio_set_irq_enabled_with_callback(BUTTON_A, GPIO_IRQ_EDGE_FALL, true, &button_interruption_gpio_irq_handler);
  gpio_set_irq_enabled_with_callback(SW, GPIO_IRQ_EDGE_FALL, true, &button_interruption_gpio_irq_handler);
  gpio_set_irq_enabled_with_callback(BUTTON_BOOT, GPIO_IRQ_EDGE_FALL, true, &button_interruption_gpio_irq_handler);

  // Loop principal
  while (1)
  {
    joystick_read_axis(&vrx_value, &vry_value); // Lê os valores dos eixos do joystick
    // Ajusta os níveis PWM dos LEDs de acordo com os valores do joystick
    pwm_set_gpio_level(LED_B, vrx_value); // Ajusta o brilho do LED azul com o valor do eixo X
    pwm_set_gpio_level(LED_R, vry_value); // Ajusta o brilho do LED vermelho com o valor do eixo Y

    // Pequeno delay antes da próxima leitura
    sleep_ms(100); // Espera 100 ms antes de repetir o ciclo
  }
}