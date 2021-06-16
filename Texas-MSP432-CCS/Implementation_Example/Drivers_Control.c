#include "Drivers_Control.h"
void DR_leds_sw_pin()
{   // Selects the function of the pins
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0); // led 1
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0); // led 2 (RGB - RED)
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN1); // led 2 (RGB - GREEN)
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN2); // led 2 (RGB - BLUE)

    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1); // switch 1
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN4); // switch 2
}
inline void DR_leds_init(bool leds_on)
{
    if (leds_on)
    {
        Dr_set_led;
        Dr_set_RGB_blue;
        Dr_set_RGB_red;
        Dr_set_RGB_green;
    }
    else
    {
        Dr_clc_led;
        Dr_clc_RGB_blue;
        Dr_clc_RGB_green;
        Dr_clc_RGB_red;
    }
}
void DR_leds_alterar(uint16_t contador)
{
//    uint16_t static contador = 0;
//
//    if (contador > 7)
//    {
//        contador = 0;
//    }
//    else
//    {
//        contador++;
//    }

    if (contador == 1)
    {// AZUL
        Dr_clc_RGB_blue;
        Dr_clc_RGB_green;
        Dr_clc_RGB_red;

        Dr_set_RGB_blue;
    }
    if (contador == 2)
    {// VERDE
        Dr_clc_RGB_blue;
        Dr_set_RGB_green;
    }
    if (contador == 3)
    {// VERMELHO

        Dr_clc_RGB_green;
        Dr_set_RGB_red;
    }
    if (contador == 4)
    {// AZUL+VERDE

        Dr_clc_RGB_blue;
        Dr_clc_RGB_green;
        Dr_clc_RGB_red;
        Dr_set_RGB_blue;
        Dr_set_RGB_green;

    }
    if (contador == 5)
    {//AZUL+VERMELHO

        Dr_clc_RGB_blue;
        Dr_clc_RGB_green;
        Dr_clc_RGB_red;
        Dr_set_RGB_blue;
        Dr_set_RGB_red;
    }
    if (contador == 6)
    {//VERMELHO+VERDE

        Dr_clc_RGB_blue;
        Dr_clc_RGB_green;
        Dr_clc_RGB_red;
        Dr_set_RGB_red;
        Dr_set_RGB_green;
    }
    if (contador == 7)
    {// BRANCO

        Dr_set_RGB_blue;
        Dr_set_RGB_green;
        Dr_set_RGB_red;
    }

}
void DR_leds_alterar_pela_sw()
{
    int estado_SW1 = Dr_read_SW1
    ;
    int estado_SW2 = Dr_read_SW2
    ;
    static int k = 0;
    if (estado_SW1 == 1)
    {
        k++;
        if (k == 8)
        {
            k = 7;
        }
        DR_leds_alterar(1);
        DR_delay_k(5);
    }
    if (estado_SW2 == 1)
    {
        if (k == 0)
        {
            k = 1;
        }
        k--;
        DR_leds_alterar(2);
        DR_delay_k(5);
    }

}
void DR_delay_k(double maximo)
{
    double contagem;
    contagem = 1000 * maximo;
    uint32_t ii;
    for (ii = 0; ii < contagem; ii++)
        ;
}
void DR_delay_ms(uint16_t n)
{ /*for 12Mhz de DCO:MCLK*/
    uint16_t i, j;
    for (j = 0; j < n; j++)
        for (i = 1000; i > 0; i--)
            ;
}
void DR_delay_s(uint16_t n)
{ /*for 12Mhz de DCO:MCLK*/
    uint16_t i, j, k;

    for (j = 0; j < n; j++)
        for (i = 1000; i > 0; i--)
            for (k = 1000; k > 0; k--)
                ;
}
int fputc(int _c, register FILE *_fp)
{
//    while (!(UCA0IFG & UCTXIFG))
//        ;
//    UCA0TXBUF = (unsigned char) _c;
    while (!(EUSCI_A0 -> IFG & 0x02));
    EUSCI_A0->TXBUF = (unsigned char) _c;

    return ((unsigned char) _c);
}
int fputs(const char *_ptr, register FILE *_fp)
{
    unsigned int i, len;

    len = strlen(_ptr);

    for (i = 0; i < len; i++)
    {
        while (!(UCA0IFG & UCTXIFG))
            ;
        UCA0TXBUF = (unsigned char) _ptr[i];
    }
    return len;
}
void DR_uart_pin()
{
    /* Selecting P1.2 and P1.3 in UART mode */
    GPIO_setAsPeripheralModuleFunctionInputPin(
    GPIO_PORT_P1,
                                               GPIO_PIN2 | GPIO_PIN3,
                                               GPIO_PRIMARY_MODULE_FUNCTION);
}
void DR_uart_config(bool fast_mode)
{ /* Setting DCO to 12MHz */
    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12); /*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/

    uint_fast16_t clockPrescalar;
    uint_fast8_t firstModReg;
    uint_fast8_t secondModReg;
    if (fast_mode)
    {		// BAUD_RATE_115200_Kbps
        clockPrescalar = 6;
        firstModReg = 8;
        secondModReg = 0;
    }
    else
    {	// BAUD_RATE_9600_Kbps
        clockPrescalar = 78;
        firstModReg = 2;
        secondModReg = 0;

    }
    const eUSCI_UART_ConfigV1 uartConfig = {
            EUSCI_A_UART_CLOCKSOURCE_SMCLK, clockPrescalar, firstModReg,
            secondModReg,
            EUSCI_A_UART_NO_PARITY,
            EUSCI_A_UART_LSB_FIRST,
            EUSCI_A_UART_ONE_STOP_BIT,
            EUSCI_A_UART_MODE,
            EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION,
            EUSCI_A_UART_8_BIT_LEN

    };

    /* Configuring UART Module */
    UART_initModule(EUSCI_A0_BASE, &uartConfig);
}
void DR_uart_init()
{ /* Enable UART module */
    UART_enableModule(EUSCI_A0_BASE);

    // printf para testar
    printf("\r\nPrintf support for the launchpad\r\n");
    printf("Decimal(10) :%d\r\n", 10);
    printf("Hex(10)     :%x\r\n", 10);
    printf("float       :%f\r\n", 4.32);
}
void DR_uart_interrupt_receive()
{
    //	void EUSCIA0_IRQHandler(void)
    /* Enabling interrupts */
    UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    Interrupt_enableInterrupt(INT_EUSCIA0);
    //Interrupt_enableSleepOnIsrExit();
    /* Interruptions Handler */
    // void EUSCIA0_IRQHandler(void)
}
void DR_interrupt_on()
{
    Interrupt_enableMaster();
}
void DR_clks_print()
{
    uint_fast32_t MCLK_timer = CS_getMCLK();
    uint_fast32_t SMCLK_timer = CS_getSMCLK();
    uint_fast32_t HSMCLK_timer = CS_getHSMCLK();
    uint_fast32_t DCO_timer = CS_getDCOFrequency();
    uint_fast32_t ACLK_timer = CS_getACLK();
    uint_fast32_t BCLK_timer = CS_getBCLK();
    printf("\n\r MCLK:%u, SMCLK:%u, HSMCLK:%u,\n\r DCO:%u, ACLK:%u, BCLK%u",
           MCLK_timer, SMCLK_timer, HSMCLK_timer, DCO_timer, ACLK_timer,
           BCLK_timer);
}
uint32_t DR_tick_start()
{
    estouro_Systick = SYSTICK_OVERFLOW;  				// 10 seg
    uint32_t period = CS_getMCLK();
    SysTick_setPeriod(period);        // 1seg
    SysTick->VAL = 0;       		 // reiniciar contagem
    SysTick_enableInterrupt();
    SysTick_enableModule();       	// inicia de fato a contagem
    return SysTick_getValue();
}
double DR_tick_stop(bool ultra_precision_mode)
{
    uint32_t tick = SysTick_getValue();      // para a contagem

    double rate = CS_getMCLK() / 1000000; // Variavel de precisão não otimizada
    tick = SysTick_getPeriod() - tick; // Sem uso do hardware de ponto flutuante

    double tempo_us = tick / rate;      	// tempo em us
    double tempo_ms = tempo_us / 1000;      		// tempo em ms
    tempo_ms = (10 - SYSTICK_OVERFLOW) * 1000 + tempo_ms;
    SysTick_disableInterrupt();
    SysTick_disableModule();
    if (!ultra_precision_mode) // todas essas fun��es duram em torno de 0.71 ms
    {
        //printf("\n\rO valor do Systick: %.4fms,MCLK:%.2fMHz", tempo_ms, rate);
        return tempo_ms;
    }
    else
    {
//        printf("\n\rO valor do Systick: %.4fus,MCLK:%.2fMHz", tempo_us, rate);
        return tempo_us;
    }
}
void SysTick_Handler(void)
{
    if (estouro_Systick == 0)
    {
        printf("\n\r!!!!!!!!!Tempo_do_Sys_Tick_estorou !!!!!!!");
        printf("\n\rintervalo maior que 10seg!");
        estouro_Systick = SYSTICK_OVERFLOW;
    }
    else
    {
        estouro_Systick--;
    }
}
void DR_t32_config_seg(uint_fast8_t n_Timer, float tempo_seg)
{ /* Fazer um estudo mais preciso dos limites dessa função */
    uint32_t timer;
    /* Sele��o do Timer */
    if (n_Timer == 0)
        timer = TIMER32_0_BASE;
    else
        timer = TIMER32_1_BASE;

    /* C�lculo do periodo do timer */
    uint32_t mclk = CS_getMCLK();
    double count_period = mclk / 256;	//Dividindo pelo prescaler
    count_period = count_period * tempo_seg;
    /* Inicializa��o e configura��o do m�dulo */
    Timer32_initModule(timer,
    TIMER32_PRESCALER_256,
                       TIMER32_32BIT,
                       TIMER32_PERIODIC_MODE);
    Timer32_setCount(timer, count_period);
}
void DR_t32_config_Hz(uint_fast8_t n_Timer, float freq_Hz)
{ /* Fazer um estudo mais preciso dos limites dessa função */
    uint32_t timer;
    /* Sele��o do Timer */
    if (n_Timer == 0)
        timer = TIMER32_0_BASE;
    else
        timer = TIMER32_1_BASE;

    /* C�lculo do periodo do timer */
    uint32_t mclk = CS_getMCLK();
    double count_period = mclk / freq_Hz;

    /* Inicializa��o e configura��o do m�dulo */
    Timer32_initModule(timer,
    TIMER32_PRESCALER_1,
                       TIMER32_32BIT,
                       TIMER32_PERIODIC_MODE);
    Timer32_setCount(timer, count_period);
}
void DR_t32_init(uint_fast8_t n_Timer)
{
    uint32_t timer;
    /* Sele��o do Timer */
    if (n_Timer == 0)
        timer = TIMER32_0_BASE;
    else
        timer = TIMER32_1_BASE;

    Timer32_startTimer(timer, false);
}
void DR_t32_interrupt_init(uint_fast8_t n_Timer, void rotina(void))
{
    /* Sele��o do Timer */
    uint32_t timer;
    if (n_Timer == 0)
        timer = INT_T32_INT1;
    else
        timer = INT_T32_INT2;
    // Sempre definir uma rotina para receber a interrup��o como a registrada aqui
    Interrupt_registerInterrupt(timer, rotina);
    Interrupt_enableInterrupt(timer);
}
double DR_t32_getPeriod_seg(uint_fast8_t n_Timer)
{ /* Sele��o do Timer 0 ou 1 */
    double period;
    if (n_Timer == 0)
        period = TIMER32_1->LOAD * 256;
    else
        period = TIMER32_2->LOAD * 256;

    /* C�lculo do Periodo */
    period = period / (CS_getMCLK());
    return period;
}
double DR_t32_get_freq(uint_fast8_t n_Timer)
{
    double period;
    /* Sele��o do Timer */
    if (n_Timer == 0)
        period = TIMER32_1->LOAD;
    else
        period = TIMER32_2->LOAD;

    /* C�lculo do Periodo */
    period = CS_getMCLK() / period;
    return period;
}
void DR_i2c_pin()
{ /* I2C0 1.6(SDA) e 1.7(SCL) e I2C1 6.4(SDA) e 6.5(SCL) */
    /* Select Port 1 for I2C - Set Pin 6, 7 to input Primary Module Function,
     *   (UCB0SIMO/UCB0SDA, UCB0SOMI/UCB0SCL). and setting P5.5 for input mode
     *   with pull-up enabled
     */
    GPIO_setAsPeripheralModuleFunctionInputPin(
    GPIO_PORT_P1,
                                               GPIO_PIN6 + GPIO_PIN7,
                                               GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionInputPin(
    GPIO_PORT_P6,
                                               GPIO_PIN4 + GPIO_PIN5,
                                               GPIO_PRIMARY_MODULE_FUNCTION);

}
void DR_i2c_config(uint_fast8_t n_I2C)
{
    /* Selecionar I2C */
    uint32_t moduleI2C;
    switch (n_I2C)
    {
    case 0:
        moduleI2C = EUSCI_B0_BASE;
        break;
    case 1:
        moduleI2C = EUSCI_B1_BASE;
        break;
    case 2:
        moduleI2C = EUSCI_B2_BASE;
        break;
    case 3:
        moduleI2C = EUSCI_B3_BASE;
        break;
    default:
        moduleI2C = EUSCI_B0_BASE;
        break;
    }
    /* I2C Master Configuration Parameter */
    const eUSCI_I2C_MasterConfig i2cConfig = {
    EUSCI_B_I2C_CLOCKSOURCE_SMCLK,	// SMCLK Clock Source
            12000000,			// SMCLK = 12MHz
            EUSCI_B_I2C_SET_DATA_RATE_400KBPS, // Desired I2C Clock of 400khz
            0,			// No byte counter threshold
            EUSCI_B_I2C_NO_AUTO_STOP   // No Autostop
            };
    /*Inicializar I2C - begin*/
    I2C_disableModule(moduleI2C);
    /* Initializing I2C Master to SMCLK at 400khz with no autostop */
    I2C_initMaster(moduleI2C, &i2cConfig);
}
void DR_i2c_init(uint_fast8_t n_I2C)
{ /*Escolher o modulo*/
    uint32_t moduleI2C;
    switch (n_I2C)
    {
    case 0:
        moduleI2C = EUSCI_B0_BASE;
        break;
    case 1:
        moduleI2C = EUSCI_B1_BASE;
        break;
    case 2:
        moduleI2C = EUSCI_B2_BASE;
        break;
    case 3:
        moduleI2C = EUSCI_B3_BASE;
        break;
    default:
        moduleI2C = EUSCI_B0_BASE;
        break;
    }
    /* Enable I2C Module to start operations */
    I2C_enableModule(moduleI2C);
}
void DR_i2c_read(uint_fast8_t n_I2C, uint8_t slaveAddr, uint8_t memAddr,
                 uint8_t *data)
{
    /* Selecionar I2C */
    EUSCI_B_Type *moduleI2C;
    switch (n_I2C)
    {
    case 0:
        moduleI2C = EUSCI_B0;
        break;
    case 1:
        moduleI2C = EUSCI_B1;
        break;
    case 2:
        moduleI2C = EUSCI_B2;
        break;
    case 3:
        moduleI2C = EUSCI_B3;
        break;
    default:
        moduleI2C = EUSCI_B0;
        break;
    }
    // dr_I2C_read() begin
    /* Read a single byte at memAddr
     * read: S-(slaveAddr+w)-ACK-memAddr-ACK-R-(saddr+r)-ACK-data-NACK-P
     */

    moduleI2C->I2CSA = slaveAddr; /* setup slave address */
    moduleI2C->CTLW0 |= 0x0010; /* enable transmitter */
    moduleI2C->CTLW0 |= 0x0002; /* generate START and send slave address */
    while ((moduleI2C->CTLW0 & 2))
        ; /* wait until slave address is sent */
    moduleI2C->TXBUF = memAddr; /* send memory address to slave */
    while (!(moduleI2C->IFG & 2))
        ; /* wait till it's ready to transmit */
    moduleI2C->CTLW0 &= ~0x0010; /* enable receiver */
    moduleI2C->CTLW0 |= 0x0002; /* generate RESTART and send slave address */
    while (moduleI2C->CTLW0 & 2)
        ; /* wait till restart is finished */
    moduleI2C->CTLW0 |= 0x0004; /* setup to send STOP after the byte is received */
    while (!(moduleI2C->IFG & 1))
        ; /* wait till data is received */
    *data = moduleI2C->RXBUF; /* read the received data */
    while (moduleI2C->CTLW0 & 4)
        ; /* wait until STOP is sent */
    // dr_I2C_read() end
}
void DR_i2c_write(uint_fast8_t n_I2C, uint8_t slaveAddr, uint8_t memAddr,
                  uint8_t data)
{
    /* Selecionar I2C */
    EUSCI_B_Type *moduleI2C;
    switch (n_I2C)
    {
    case 0:
        moduleI2C = EUSCI_B0;
        break;
    case 1:
        moduleI2C = EUSCI_B1;
        break;
    case 2:
        moduleI2C = EUSCI_B2;
        break;
    case 3:
        moduleI2C = EUSCI_B3;
        break;
    default:
        moduleI2C = EUSCI_B0;
        break;
    }
    /* Write a single byte at memAddr
     * write: S-(slaveAddr+w)-ACK-memAddr-ACK-data-ACK-P
     */
    moduleI2C->I2CSA = slaveAddr; /* setup slave address */
    moduleI2C->CTLW0 |= 0x0010; /* enable transmitter */
    moduleI2C->CTLW0 |= 0x0002; /* generate START and send slave address */
    while ((moduleI2C->CTLW0 & 2))
        /*Error Here !*/
        ; /* wait until slave address is sent */
    moduleI2C->TXBUF = memAddr; /* send memory address to slave */
    while (!(moduleI2C->IFG & 2))
        ; /* wait till it's ready to transmit */
    moduleI2C->TXBUF = data; /* send data to slave */
    while (!(moduleI2C->IFG & 2))
        ; /* wait till last transmit is done */
    moduleI2C->CTLW0 |= 0x0004; /* send STOP */
    while (moduleI2C->CTLW0 & 4)
        ; /* wait until STOP is sent */
}
int DR_i2c_readraw(uint_fast8_t n_I2C, uint8_t slaveAddr, uint8_t memAddr,
                   uint8_t byteCount, uint8_t *data)
{
    /* Selecionar I2C */
    EUSCI_B_Type *moduleI2C;
    switch (n_I2C)
    {
    case 0:
        moduleI2C = EUSCI_B0;
        break;
    case 1:
        moduleI2C = EUSCI_B1;
        break;
    case 2:
        moduleI2C = EUSCI_B2;
        break;
    case 3:
        moduleI2C = EUSCI_B3;
        break;
    default:
        moduleI2C = EUSCI_B0;
        break;
    }

    /* Read Raw begin */
    if (byteCount <= 0)
        return -1; /* no read was performed */

    moduleI2C->I2CSA = slaveAddr; /* setup slave address */
    moduleI2C->CTLW0 |= 0x0010; /* enable transmitter */
    moduleI2C->CTLW0 |= 0x0002; /* generate START and send slave address */
    while ((moduleI2C->CTLW0 & 2))
        ; /* wait until slave address is sent */
    moduleI2C->TXBUF = memAddr; /* send memory address to slave */
    while (!(moduleI2C->IFG & 2))
        // Error Here !!!!
        ; /* wait till last transmit is done */
    moduleI2C->CTLW0 &= ~0x0010; /* enable receiver */
    moduleI2C->CTLW0 |= 0x0002; /* generate RESTART and send slave address */
    while (moduleI2C->CTLW0 & 2)
        ; /* wait till RESTART is finished */

    /* receive data one byte at a time */
    do
    {
        if (byteCount == 1) /* when only one byte of data is left */
            moduleI2C->CTLW0 |= 0x0004; /* setup to send STOP after the last byte is received */

        while (!(moduleI2C->IFG & 1))
            // error here !!
            ; /* wait till data is received */
        *data++ = moduleI2C->RXBUF; /* read the received data */
        byteCount--;
    }
    while (byteCount);

    while (moduleI2C->CTLW0 & 4)
        ; /* wait until STOP is sent */

    return 0; /* no error */
}
void DR_pmap_pin()
{/*I2C2_pins: 2.5(SDA) e 3.0(SCL) */
    /* Pmap Code / Avaible for port 2, 3 e 7 */
    PMAP->KEYID = 0x2D52; /* Unlock PMAP */
    /* Escolha dos novos pinos */
    P2MAP->PMAP_REGISTER5 = PMAP_UCB2SDA; /* UCB0SDA, MAP2.5 to ucb2_sda */
    P3MAP->PMAP_REGISTER0 = PMAP_UCB2SCL; /* UCB0SCL, MAP3.0 to ucb2_scl */

    /* atribui��o das fun��es prim�rias */
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2, GPIO_PIN5,
    GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3, GPIO_PIN0,
    GPIO_PRIMARY_MODULE_FUNCTION);
    /* Lock PMAP */
    PMAP->CTL = 1;
    PMAP->KEYID = 0;
}
void DR_pwm_pin()
{ /* PWM : Pin Config */
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN4,
    GPIO_PRIMARY_MODULE_FUNCTION); // PM_TA0.1
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN5,
    GPIO_PRIMARY_MODULE_FUNCTION); // PM_TA0.2
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN6,
    GPIO_PRIMARY_MODULE_FUNCTION); // PM_TA0.3
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN7,
    GPIO_PRIMARY_MODULE_FUNCTION); // PM_TA0.4
    // GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P5, GPIO_PIN6,
    // GPIO_PRIMARY_MODULE_FUNCTION); // PM_TA2.1
    // GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P5, GPIO_PIN7,
    // GPIO_PRIMARY_MODULE_FUNCTION); // PM_TA2.2
}
void DR_pwm_config(const dr_pwm_parameters *pwm_config)
{ /*Configurar modo de operação do timer especificado, cada timer tem 4 canais de pwm*/
    // Selecionando clk_source
    uint_fast16_t clk_source;
    if (pwm_config->fast_mode)
    {
        clk_source = TIMER_A_CLOCKSOURCE_SMCLK;
    }
    else
    {
        clk_source = TIMER_A_CLOCKSOURCE_ACLK;
    }

    // Selecionando o modo de configuração do timer
    uint_fast16_t pwm_carrier;
    if (pwm_config->true_Sawtooth_not_triangular)
    {
        pwm_carrier = TIMER_A_UP_MODE; // set Sawtooth Carrier mode
    }
    else
    {
        pwm_carrier = TIMER_A_UPDOWN_MODE; // set Traingular Carrier mode
    }

    // Selecionar Prescaler
    _pwm_set_Prescaler(pwm_config->timer, pwm_config->timer_Prescaler);
    // Limpar registradores que irão ser configurados
    TIMER_A_CMSIS(pwm_config->timer)->CTL &=
            ~(TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK + TIMER_A_UPDOWN_MODE
                    + TIMER_A_DO_CLEAR + TIMER_A_TAIE_INTERRUPT_ENABLE);

    // Setando as configurações recebidas
    TIMER_A_CMSIS(pwm_config->timer)->CTL |= (clk_source + pwm_carrier
            + TIMER_A_DO_CLEAR); // TACLR: IMER_A_DO_CLEAR
    TIMER_A_CMSIS(pwm_config->timer)->CCR[0] = pwm_config->period_count; // Configurando Periodo PWM
}
void DR_pwm_init(dr_pwm_parameters *pwm_config, uint_fast16_t pwm_init_duty)
{	// Limpando os registradores que serão usados

    TIMER_A_CMSIS(pwm_config->timer)->CCTL[0] &=
            ~(TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE
                    + TIMER_A_OUTPUTMODE_RESET_SET);
    TIMER_A_CMSIS(pwm_config->timer)->CCTL[pwm_config->pwm_channel] |=
            pwm_config->outputmode;
    TIMER_A_CMSIS(pwm_config->timer)->CCR[pwm_config->pwm_channel] =
            pwm_init_duty; /*Como setar o duty do canal 0 ?*/
}
inline uint16_t DR_pwm_getPeriod(dr_pwm_parameters *pwm_config)
{
    return TIMER_A_CMSIS(pwm_config->timer)->CCR[0] = pwm_config->period_count; /* PWM period*/
}
inline void DR_PWM_setDuty(dr_pwm_parameters *pwm_config,
                           uint_fast16_t pwm_duty)
{
    TIMER_A_CMSIS(pwm_config->timer)->CCR[pwm_config->pwm_channel] = pwm_duty;
}
inline uint16_t DR_pwm_getDuty(dr_pwm_parameters *pwm_config)
{ /*Pegar valor do contador do timer do cana de pwm especificado*/
    return TIMER_A_CMSIS(pwm_config->timer)->CCR[pwm_config->pwm_channel];
}
/*PWM functions test*/
float DR_pwm_getfreq(const dr_pwm_parameters *pwm_config)
{ /*Funcionou okay*/
    uint32_t PWM_clk;
    /* Timer CLK*/
    if (pwm_config->fast_mode)
        PWM_clk = CS_getSMCLK();
    else
        PWM_clk = CS_getACLK();
    // Timer Prescaler
    uint16_t PWM_prescaler = pwm_config->timer_Prescaler;
    // Timer Period
    uint16_t PWM_period = TIMER_A_CMSIS(pwm_config->timer)->CCR[0];	// O certo seria mais um n ?

    float pwm_freq = PWM_clk / PWM_prescaler;
    pwm_freq = pwm_freq / PWM_period;  // aqui se tem o periodo do pwm em Hz
    return pwm_freq;
}

float DR_pwm_getDuty_percent(dr_pwm_parameters *pwm_config)
{ /*Funcionou okay*/
    // dependendo do modo tem formas diferentes de calcular
    /* se for no modo reset/set */
    uint16_t PWM_period = TIMER_A_CMSIS(pwm_config->timer)->CCR[0]; // O certo seria mais um n ?
    uint16_t PWM_get_duty =
    TIMER_A_CMSIS(pwm_config->timer)->CCR[pwm_config->pwm_channel];
    float duty_Cycle = (PWM_get_duty * 100) / PWM_period;

    return duty_Cycle;
}

void _pwm_set_Prescaler(uint32_t timer, uint16_t timer_prescaler)
{ /* This functions is only to be used by DR_PWM_config  tentar botar em static para limitar a uso local*/
    TIMER_A_CMSIS(timer)->CTL &= ~TIMER_A_CTL_ID__8;
    TIMER_A_CMSIS(timer)->EX0 &= ~TIMER_A_EX0_IDEX_MASK;

    switch (timer_prescaler)
    {
    case TIMER_A_CLOCKSOURCE_DIVIDER_1:
    case TIMER_A_CLOCKSOURCE_DIVIDER_2:
        TIMER_A_CMSIS(timer)->CTL |= ((timer_prescaler - 1) << 6);
        TIMER_A_CMSIS(timer)->EX0 = TIMER_A_EX0_TAIDEX_0;
        break;
    case TIMER_A_CLOCKSOURCE_DIVIDER_4:
        TIMER_A_CMSIS(timer)->CTL |= TIMER_A_CTL_ID__4;
        TIMER_A_CMSIS(timer)->EX0 = TIMER_A_EX0_TAIDEX_0;
        break;
    case TIMER_A_CLOCKSOURCE_DIVIDER_8:
        TIMER_A_CMSIS(timer)->CTL |= TIMER_A_CTL_ID__8;
        TIMER_A_CMSIS(timer)->EX0 = TIMER_A_EX0_TAIDEX_0;
        break;
    case TIMER_A_CLOCKSOURCE_DIVIDER_3:
    case TIMER_A_CLOCKSOURCE_DIVIDER_5:
    case TIMER_A_CLOCKSOURCE_DIVIDER_6:
    case TIMER_A_CLOCKSOURCE_DIVIDER_7:
        TIMER_A_CMSIS(timer)->CTL |= TIMER_A_CTL_ID__1;
        TIMER_A_CMSIS(timer)->EX0 = (timer_prescaler - 1);
        break;

    case TIMER_A_CLOCKSOURCE_DIVIDER_10:
    case TIMER_A_CLOCKSOURCE_DIVIDER_12:
    case TIMER_A_CLOCKSOURCE_DIVIDER_14:
    case TIMER_A_CLOCKSOURCE_DIVIDER_16:
        TIMER_A_CMSIS(timer)->CTL |= TIMER_A_CTL_ID__2;
        TIMER_A_CMSIS(timer)->EX0 = (timer_prescaler / 2 - 1);
        break;

    case TIMER_A_CLOCKSOURCE_DIVIDER_20:
    case TIMER_A_CLOCKSOURCE_DIVIDER_24:
    case TIMER_A_CLOCKSOURCE_DIVIDER_28:
    case TIMER_A_CLOCKSOURCE_DIVIDER_32:
        TIMER_A_CMSIS(timer)->CTL |= TIMER_A_CTL_ID__4;
        TIMER_A_CMSIS(timer)->EX0 = (timer_prescaler / 4 - 1);
        break;
    case TIMER_A_CLOCKSOURCE_DIVIDER_40:
    case TIMER_A_CLOCKSOURCE_DIVIDER_48:
    case TIMER_A_CLOCKSOURCE_DIVIDER_56:
    case TIMER_A_CLOCKSOURCE_DIVIDER_64:
        TIMER_A_CMSIS(timer)->CTL |= TIMER_A_CTL_ID__8;
        TIMER_A_CMSIS(timer)->EX0 = (timer_prescaler / 8 - 1);
        break;
    }
}
void DR_adc_pin()
{ /* Configuring GPIOs (5.5 A0) */
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN5,
    GPIO_TERTIARY_MODULE_FUNCTION);
}
void DR_adc_config()
{
    /* Initializing ADC (MCLK/1/4) */
    ADC14_enableModule();
    ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_4, 0);
    /* Configuring ADC Memory */
    ADC14_configureSingleSampleMode(ADC_MEM0, true);
    ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS,
    ADC_INPUT_A0,
                                    false);
    /* Configuring Sample Timer */
    ADC14_enableSampleTimer(ADC_MANUAL_ITERATION);
    /* Enabling/Toggling Conversion */
    ADC14_enableConversion();
    ADC14_toggleConversionTrigger();
}
void DR_adc_init()
{
    /* Initializing ADC (MCLK/1/4) */
//    ADC14_enableModule();
//    ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_4, 0);
}
void DR_adc_interrupt()
{
    ADC14_enableInterrupt(ADC_INT0);
    Interrupt_enableInterrupt(INT_ADC14);
}
