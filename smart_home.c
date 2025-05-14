#include <stdio.h>               // Biblioteca padrão para entrada e saída
#include <string.h>              // Biblioteca manipular strings
#include <stdlib.h>              // funções para realizar várias operações, incluindo alocação de memória dinâmica (malloc)
#include <time.h>                // Biblioteca para manipulação de tempo e data

#include "pico/stdlib.h"         // Biblioteca da Raspberry Pi Pico para funções padrão (GPIO, temporização, etc.)
#include "hardware/adc.h"        // Biblioteca da Raspberry Pi Pico para manipulação do conversor ADC
#include "pico/cyw43_arch.h"     // Biblioteca para arquitetura Wi-Fi da Pico com CYW43  
#include "pico/bootrom.h"        // Biblioteca para manipulação do bootloader USB
#include "hardware/i2c.h"        // Biblioteca da Raspberry Pi Pico para manipulação do barramento I2C
#include "hardware/clocks.h"     // Biblioteca da Raspberry Pi Pico para manipulação de clocks
#include "hardware/timer.h"      // Biblioteca da Raspberry Pi Pico para manipulação de timers
#include "hardware/pwm.h"        // Biblioteca da Raspberry Pi Pico para manipulação de PWM

#include "lib/ssd1306.h"         // Biblioteca para controle do display OLED SSD1306
#include "lib/font.h"            // Biblioteca para manipulação de fontes no display OLED    
#include "lib/icons.h"           // Biblioteca para manipulação de ícones na matriz
#include "smart_home.pio.h"      // Biblioteca para manipulação do PIO (Programmable Input/Output) da Raspberry Pi Pico

#include "lwip/pbuf.h"           // Lightweight IP stack - manipulação de buffers de pacotes de rede
#include "lwip/tcp.h"            // Lightweight IP stack - fornece funções e estruturas para trabalhar com o protocolo TCP
#include "lwip/netif.h"          // Lightweight IP stack - fornece funções e estruturas para trabalhar com interfaces de rede (netif)

// Credenciais WIFI - Tome cuidado se publicar no github!
#define WIFI_SSID "ID-DO-WIFI"
#define WIFI_PASSWORD "SENHA-DO-WIFI"

// Definição dos pinos dos LEDs
#define LED_PIN CYW43_WL_GPIO_LED_PIN   // GPIO do CI CYW43
#define LED_BLUE_PIN 12                 // GPIO12 - LED azul
#define LED_GREEN_PIN 11                // GPIO11 - LED verde
#define LED_RED_PIN 13                  // GPIO13 - LED vermelho
#define BTN_A 5                         // GPIO5 - Botão A
#define BTN_B 6                         // GPIO6 - Botão B
#define BTN_J 22                        // GPIO22 - Botão Joystick
#define JOYSTICK_X 26                   // Pino do Joystick X
#define BUZZER_A_PIN 10                 // GPIO10 - Buzzer A
#define BUZZER_B_PIN 21                 // GPIO21 - Buzzer B

uint32_t last_time = 0; // Variável para armazenar o tempo do último evento para o debouncing

#define I2C_PORT i2c1  // Define o barramento I2C
#define I2C_SDA 14     // Define o pino SDA
#define I2C_SCL 15     // Define o pino SCL
#define endereco 0x3C  // Endereço do display OLED

#define NUM_PIXELS 25   // Número de pixels da matriz de LEDs
#define MATRIX_PIN 7    // Pino da matriz de LEDs

ssd1306_t ssd; // Estrutura para o display OLED

bool alarm_state = false; // Variável para armazenar o estado do alarme
bool alarm_active = false; // Variável para armazenar o estado do alarme
bool lock_state = false;

// Variáveis para controle de cor e ícone exibido na matriz de LEDs
double red = 0.0, green = 255.0 , blue = 0.0; // Variáveis para controle de cor
int icon = 0; //Armazena o número atualmente exibido
double* icons[6] = {icon_zero, icon_one, icon_two, icon_three, icon_four, icon_five}; //Ponteiros para os desenhos dos números

// Inicializar os Pinos GPIO para acionamento dos LEDs da BitDogLab
void gpio_bitdog(void);

// Função de callback ao aceitar conexões TCP
static err_t tcp_server_accept(void *arg, struct tcp_pcb *newpcb, err_t err);

// Função de callback para processar requisições HTTP
static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);

// Leitura da temperatura interna
float temp_read(void);

// Tratamento do request do usuário
void user_request(char **request);

// Função de callback para tratamento de interrupção dos botões
void gpio_irq_handler(uint gpio, uint32_t events);

// Função para desenhar uma tela no display OLED
void draw_screen(); 

// Função para configurar o PWM
void pwm_setup_gpio(uint gpio, uint freq);

// Função para configuração de cores RGB para a matriz de LEDs
uint32_t matrix_rgb(double r, double g, double b);

// Função para desenhar na matriz de LEDs
void desenho_pio(double *desenho, uint32_t valor_led, PIO pio, uint sm, double r, double g, double b);

// Função principal
int main()
{
    //Inicializa todos os tipos de bibliotecas stdio padrão presentes que estão ligados ao binário.
    stdio_init_all();

    // Inicializar os Pinos GPIO para acionamento dos LEDs da BitDogLa();
    gpio_bitdog();

    // Configuração do LED do CI CYW43 como saída
    gpio_set_irq_enabled_with_callback(BTN_A, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    gpio_set_irq_enabled_with_callback(BTN_B, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    gpio_set_irq_enabled_with_callback(BTN_J, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    //Configurações da PIO
    PIO pio = pio0; 
    uint offset = pio_add_program(pio, &pio_matrix_program);
    uint sm = pio_claim_unused_sm(pio, true);
    pio_matrix_program_init(pio, sm, offset, MATRIX_PIN);
    
    // Inicializa o I2C e configura os pinos SDA e SCL para o display OLED 
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Inicializa e configura o display OLED
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT); 
    ssd1306_config(&ssd);

    ssd1306_fill(&ssd, false); // Limpa o display
    ssd1306_send_data(&ssd); // Envia os dados para o display

    //Inicializa a arquitetura do cyw43
    while (cyw43_arch_init())
    {
        printf("Falha ao inicializar Wi-Fi\n");
        sleep_ms(100);
        return -1;
    }

    // GPIO do CI CYW43 em nível baixo
    cyw43_arch_gpio_put(LED_PIN, 0);

    // Ativa o Wi-Fi no modo Station, de modo a que possam ser feitas ligações a outros pontos de acesso Wi-Fi.
    cyw43_arch_enable_sta_mode();

    // Conectar à rede WiFI - fazer um loop até que esteja conectado
    printf("Conectando ao Wi-Fi...\n");
    while (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 20000))
    {
        printf("Falha ao conectar ao Wi-Fi\n");
        sleep_ms(100);
        return -1;
    }
    printf("Conectado ao Wi-Fi\n");

    // Caso seja a interface de rede padrão - imprimir o IP do dispositivo.
    if (netif_default)
    {
        printf("IP do dispositivo: %s\n", ipaddr_ntoa(&netif_default->ip_addr));
    }

    // Configura o servidor TCP - cria novos PCBs TCP. É o primeiro passo para estabelecer uma conexão TCP.
    struct tcp_pcb *server = tcp_new();
    if (!server)
    {
        printf("Falha ao criar servidor TCP\n");
        return -1;
    }

    //vincula um PCB (Protocol Control Block) TCP a um endereço IP e porta específicos.
    if (tcp_bind(server, IP_ADDR_ANY, 80) != ERR_OK)
    {
        printf("Falha ao associar servidor TCP à porta 80\n");
        return -1;
    }

    // Coloca um PCB (Protocol Control Block) TCP em modo de escuta, permitindo que ele aceite conexões de entrada.
    server = tcp_listen(server);

    // Define uma função de callback para aceitar conexões TCP de entrada. É um passo importante na configuração de servidores TCP.
    tcp_accept(server, tcp_server_accept);
    printf("Servidor ouvindo na porta 80\n");

    // Inicializa o conversor ADC
    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_gpio_init(JOYSTICK_X); 

    while (true)
    {
        adc_select_input(0); 
        uint16_t adc_value = adc_read();
        uint32_t current_time = to_us_since_boot(get_absolute_time()); // Pega o tempo atual em ms
        if (alarm_state == true && current_time - last_time > 1000000 && !alarm_active) { gpio_put(LED_RED_PIN, !gpio_get(LED_RED_PIN)); last_time = current_time; } // Debouncing de 1000ms

        if (alarm_state && (adc_value > 2500 || adc_value < 1500) && !alarm_active) { // Se o alarme estiver ativo e o valor do ADC for maior que 2500
            alarm_active = true; // Ativa o alarme
        }

        if (alarm_active && alarm_state) {
            gpio_put(LED_RED_PIN, 1); // Liga o LED vermelho
            pwm_setup_gpio(BUZZER_A_PIN, 1000); // Ativa o buzzer A
            sleep_ms(800); // Aguarda 1 segundo
            pwm_setup_gpio(BUZZER_A_PIN, 0); // Desativa o buzzer A
        }

        desenho_pio(icons[icon], 0, pio, sm, red, green, blue); // Desenha o ícone na matriz de LEDs

        draw_screen(); // Atualiza a tela do display OLED

        cyw43_arch_poll(); // Necessário para manter o Wi-Fi ativo
        sleep_ms(100);      // Reduz o uso da CPU
    }

    //Desligar a arquitetura CYW43.
    cyw43_arch_deinit();
    return 0;
}

// -------------------------------------- Funções ---------------------------------

// Inicializar os Pinos GPIO da BitDogLab
void gpio_bitdog(void){
    // Configuração dos LEDs como saída
    gpio_init(LED_BLUE_PIN);
    gpio_set_dir(LED_BLUE_PIN, GPIO_OUT);
    gpio_put(LED_BLUE_PIN, false);
    
    gpio_init(LED_GREEN_PIN);
    gpio_set_dir(LED_GREEN_PIN, GPIO_OUT);
    gpio_put(LED_GREEN_PIN, false);
    
    gpio_init(LED_RED_PIN);
    gpio_set_dir(LED_RED_PIN, GPIO_OUT);
    gpio_put(LED_RED_PIN, false);

    gpio_init(BTN_A);
    gpio_set_dir(BTN_A, GPIO_IN);
    gpio_pull_up(BTN_A);

    gpio_init(BTN_B);
    gpio_set_dir(BTN_B, GPIO_IN);
    gpio_pull_up(BTN_B);

    gpio_init(BTN_J);
    gpio_set_dir(BTN_J, GPIO_IN);
    gpio_pull_up(BTN_J);

    gpio_init(BUZZER_A_PIN);  
    gpio_set_dir(BUZZER_A_PIN, GPIO_OUT);
    gpio_init(BUZZER_B_PIN);  
    gpio_set_dir(BUZZER_B_PIN, GPIO_OUT);
}

// Função de callback ao aceitar conexões TCP
static err_t tcp_server_accept(void *arg, struct tcp_pcb *newpcb, err_t err)
{
    tcp_recv(newpcb, tcp_server_recv);
    return ERR_OK;
}

// Tratamento do request do usuário - digite aqui
void user_request(char **request){

    if (strstr(*request, "GET /all_on") != NULL)
    {
        gpio_put(LED_BLUE_PIN, 1);
        gpio_put(LED_GREEN_PIN, 1);
        gpio_put(LED_RED_PIN, 1);
    }
    else if (strstr(*request, "GET /all_off") != NULL)
    {
        gpio_put(LED_BLUE_PIN, 0);
        gpio_put(LED_GREEN_PIN, 0);
        gpio_put(LED_RED_PIN, 0);
    }
    else if (strstr(*request, "GET /alarm_on") != NULL)
    {
        pwm_setup_gpio(BUZZER_A_PIN, 500);
        sleep_ms(500);
        pwm_setup_gpio(BUZZER_A_PIN, 1000);
        sleep_ms(250);
        pwm_setup_gpio(BUZZER_A_PIN, 0);
        alarm_state = true;
    }
    else if (strstr(*request, "GET /alarm_off") != NULL)
    {
        pwm_setup_gpio(BUZZER_A_PIN, 1000);
        sleep_ms(500);
        pwm_setup_gpio(BUZZER_A_PIN, 500);
        sleep_ms(250);
        pwm_setup_gpio(BUZZER_A_PIN, 0);
        alarm_state = false;
        alarm_active = false;
        gpio_put(LED_RED_PIN, 0);
    }
    else if (strstr(*request, "GET /lock_on") != NULL)
    {
        red = 0.0; green = 255.0; blue = 0.0;
        pwm_setup_gpio(BUZZER_B_PIN, 1250);
        sleep_ms(250);
        pwm_setup_gpio(BUZZER_B_PIN, 1500);
        sleep_ms(250);
        pwm_setup_gpio(BUZZER_B_PIN, 1750);
        sleep_ms(250);
        pwm_setup_gpio(BUZZER_B_PIN, 0);
        icon = 0;
        lock_state = 0;
    }
    else if (strstr(*request, "GET /lock_off") != NULL)
    {
        red = 255.0; green = 0.0; blue = 0.0;
        pwm_setup_gpio(BUZZER_B_PIN, 1750);
        sleep_ms(250);
        pwm_setup_gpio(BUZZER_B_PIN, 1500);
        sleep_ms(250);
        pwm_setup_gpio(BUZZER_B_PIN, 1250);
        sleep_ms(250);
        pwm_setup_gpio(BUZZER_B_PIN, 0);
        icon = 1;
        lock_state = 1;
    }

    else if (strstr(*request, "GET /on") != NULL)
    {
        cyw43_arch_gpio_put(LED_PIN, 1);
    }
    else if (strstr(*request, "GET /off") != NULL)
    {
        cyw43_arch_gpio_put(LED_PIN, 0);
    }
};

// Leitura da temperatura interna
float temp_read(void){
    adc_select_input(4);
    uint16_t raw_value = adc_read();
    const float conversion_factor = 3.3f / (1 << 12);
    float temperature = 27.0f - ((raw_value * conversion_factor) - 0.706f) / 0.001721f;
        return temperature;
}

// Função de callback para processar requisições HTTP
static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    if (!p)
    {
        tcp_close(tpcb);
        tcp_recv(tpcb, NULL);
        return ERR_OK;
    }

    // Alocação do request na memória dinámica
    char *request = (char *)malloc(p->len + 1);
    memcpy(request, p->payload, p->len);
    request[p->len] = '\0';

    printf("Request: %s\n", request);

    // Tratamento de request - Controle dos LEDs
    user_request(&request);
    
    // Leitura da temperatura interna
    float temperature = temp_read();

    // Cria a resposta HTML
    char html[1024];

    // Instruções html do webserver
    snprintf(html, sizeof(html), // Formatar uma string e armazená-la em um buffer de caracteres
             "HTTP/1.1 200 OK\r\n"
             "Content-Type: text/html\r\n"
             "\r\n"
             "<!DOCTYPE html>\n"
             "<html>\n"
             "<head>\n"
             "<title> Smart Home - House Control </title>\n"
             "<style>\n"
             "body { background-color: #b5e5fb; font-family: Arial, sans-serif; text-align: center; margin-top: 50px; }\n"
             "h1 { font-size: 64px; margin-bottom: 30px; }\n"
             "button { background-color: LightGray; font-size: 36px; margin: 10px; padding: 20px 40px; border-radius: 10px; }\n"
             ".temperature { font-size: 48px; margin-top: 30px; color: #333; }\n"
             "</style>\n"
             "</head>\n"
             "<body>\n"
             "<h1>Smart Home - House Control</h1>\n"
             "<form action=\"./all_on\"><button>Ligar luzes</button></form>\n"
             "<form action=\"./all_off\"><button>Desligar luzes</button></form>\n"
             "<form action=\"./alarm_on\"><button>Ligar alarme</button></form>\n"
             "<form action=\"./alarm_off\"><button>Desligar alarme</button></form>\n"
             "<form action=\"./lock_on\"><button>Abrir fechadura</button></form>\n"
             "<form action=\"./lock_off\"><button>Fechar fechadura</button></form>\n"
             "<p class=\"temperature\">Temperatura Interna: %.2f &deg;C</p>\n"
             "</body>\n"
             "</html>\n",
             temperature);

    // Escreve dados para envio (mas não os envia imediatamente).
    tcp_write(tpcb, html, strlen(html), TCP_WRITE_FLAG_COPY);

    // Envia a mensagem
    tcp_output(tpcb);

    //libera memória alocada dinamicamente
    free(request);
    
    //libera um buffer de pacote (pbuf) que foi alocado anteriormente
    pbuf_free(p);

    return ERR_OK;
}

// Função de callback para tratamento de interrupção dos botões
void gpio_irq_handler(uint gpio, uint32_t events) { 
    uint32_t current_time = to_us_since_boot(get_absolute_time()); // Pega o tempo atual em ms
    if (current_time - last_time > 250000) { // Debouncing de 250ms
        last_time = current_time;
        if (gpio == BTN_A) { // Verifica se o botão A foi pressionado 
            printf("Botão A pressionado!\n");
        } else if (gpio == BTN_B) { // Verifica se o botão B foi pressionado e envia feedback para UART
            printf("Botão B pressionado!\n");
        } else if (gpio == BTN_J) { // Verifica se o botão do joystick foi pressionado e entra no modo bootsel
            printf("Botão do joystick pressionado!\n");
            reset_usb_boot(0, 0);
        }
    }
}

// Função para desenhar uma tela no display OLED
void draw_screen() {
    ssd1306_fill(&ssd, false); // Limpa o display
    char buffer[32]; // Buffer para armazenar valores de temperatura como string

    ssd1306_rect(&ssd, 0, 0, 128, 64, true, false);
    ssd1306_rect(&ssd, 0, 0, 128, 12, true, false);  
    ssd1306_draw_string(&ssd, "SMART HOME", 25, 2); // Desenha o texto "EMBARCATECH" na posição (0, 0)

    ssd1306_draw_string(&ssd, "Temp: ", 2, 14); 
    sprintf(buffer, "%.1f", temp_read()); // Formata a temperatura como string
    ssd1306_draw_string(&ssd, buffer, 48, 14);

    if (alarm_state) { ssd1306_draw_string(&ssd, "Alarme: ON", 2, 24); } else { ssd1306_draw_string(&ssd, "Alarme: OFF", 2, 24); }
    if (lock_state) { ssd1306_draw_string(&ssd, "Fechadura: ON", 2, 34); } else { ssd1306_draw_string(&ssd, "Fechadura: OFF", 2, 34); }
    
    ssd1306_send_data(&ssd); // Envia os dados para o display
}

// Função para configurar o PWM
void pwm_setup_gpio(uint gpio, uint freq) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);  // Define o pino como saída PWM
    uint slice_num = pwm_gpio_to_slice_num(gpio);  // Obtém o slice do PWM

    if (freq == 0) {
        pwm_set_enabled(slice_num, false);  // Desabilita o PWM
        gpio_put(gpio, 0);  // Desliga o pino
    } else {
        uint32_t clock_div = 4; // Define o divisor do clock
        uint32_t wrap = (clock_get_hz(clk_sys) / (clock_div * freq)) - 1; // Calcula o valor de wrap

        // Configurações do PWM (clock_div, wrap e duty cycle) e habilita o PWM
        pwm_set_clkdiv(slice_num, clock_div); 
        pwm_set_wrap(slice_num, wrap);  
        pwm_set_gpio_level(gpio, wrap / 5);
        pwm_set_enabled(slice_num, true);  
    }
}

// Rotina para definição da intensidade de cores do led
uint32_t matrix_rgb(double r, double g, double b) {
    unsigned char R, G, B;
    R = r * red;
    G = g * green;
    B = b * blue;
    return (G << 24) | (R << 16) | (B << 8);
}

// Rotina para acionar a matrix de leds - ws2812b
void desenho_pio(double *desenho, uint32_t valor_led, PIO pio, uint sm, double r, double g, double b) {
    for (int16_t i = 0; i < NUM_PIXELS; i++) {
        valor_led = matrix_rgb(desenho[24-i], desenho[24-i], desenho[24-i]);
        pio_sm_put_blocking(pio, sm, valor_led);
    }
}
