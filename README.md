# Conversor ADC

A proposta desse experimento teve o objetivo de consolidar alguns dos conceitos sobre o uso de conversores analógico-digitais (ADC) no RP2040 e explorar as funcionalidades da placa de desenvolvimento BitDogLab. 

Além do esforço em compeender o funcionamento do conversor ADC em questão, tal estudo comprendeu utilizar o PWM no controle da intensidade de dois dos LEDs RGB com base nos valores originados do joystick, em conjunto com a representação gráfica das coordenadas dos eixos XY do joystick no display SSD1306. Tal representação se deu por meio de um quadrado móvel (um cursor) e o protocolo de comunicação I2C na integração ao display de 128x64 pixels.

O LED Azul teve seu brilho ajustado conforme o valor do eixo Y do joystick que, enquanto solto (posição central - valor 2048), mantém-se apagado. À medida que o joystick é movido para cima (valores menores) ou para baixo (valores maiores), o LED aumenta seu brilho gradualmente, atingindo a intensidade máxima nos extremos (0 e 4095).

O LED Vermelho segue o mesmo princípio, mas de acordo com o eixo X do joystick. Ambos os LEDs são controlados via PWM para permitir variação suave da intensidade luminosa.

No display SSD1306, o quadrado de 8x8 pixels, por mim apelidado de "cursor", inicialmente surge centralizado, mas é capaz de se mover proporcionalmente aos valores capturados pelo joystick. Ajsutes necessários foram realizados conforme a placa de desenvolvimento utilizada nesse experiemento e os valores retornados, particularmente, pelo seu conversor ADC

Adicionalmente, o botão do joystick permite alternar o estado do LED Verde a cada acionamento, ao mesmo tempo em que modifica a borda do display para indicar quando o mesmo é pressionado, alternando seu estilo a cada novo acionamento.

Finalmente, o botão A, o push button à esquerda da placa, coube a funcionalidade de ativar e desativar os LEDs PWM, a cada acionamento.

Para eese desenvolvimento foram aplicados alguns requisitos, tais como o uso de interrupções (IRQ) e contramedidas deboucing, via software, nas funcionalidades relacionadas aos botões. Além disso, um timer repetitivo foi utilizado opcionalmente na execução e controle do cursor e a alternância das molduras, uma eestratégia eficiente à liberação do loop na função main para quaisquer outras responsabilidades. 

O resultado pode do estudo pode ser conferido no link 
