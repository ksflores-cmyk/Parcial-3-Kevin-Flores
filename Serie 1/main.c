#include <stdint.h>
#include "stm32l053xx.h"

/*
    Caja fuerte electrónica sobre NUCLEO-L053R8
    -------------------------------------------------

    KEYPAD 4x4
    -------------------------------------------------
    - PB12, PB13, PB14, PB15  -> FILAS (L1..L4)  ***SALIDAS***
        En reposo las dejamos en HIGH.
        Para escanear bajamos UNA fila a LOW.
    - PB8, PB9, PB10, PB11    -> COLUMNAS (C1..C4) ***ENTRADAS con PULL-UP***
        Leemos cuál columna cae a LOW si una tecla está presionada.

    Mapa típico de teclas por [fila][columna]:
        fila0 (PB12):   1   2   3   A
        fila1 (PB13):   4   5   6   B
        fila2 (PB14):   7   8   9   C
        fila3 (PB15):   *   0   #   D

    => keypad_map[fila_idx][col_idx] devuelve el char real.

    Debug:
    - Cada vez que detectamos una tecla nueva, encendemos LED_OPEN (PA15).
    - También mandamos esa tecla por UART2 y actualizamos la LCD.
    - Si no hay tecla, apagamos LED_OPEN.

    Otros periféricos:
    - LCD 16x2 en 4 bits (PA0 RS, PA1 E, PA8/PA10/PA5/PA6 D4..D7)
    - Display 7 segmentos multiplex (PB0..PB7 seg, PC5/PC6/PC8/PC9 dígitos)
    - Stepper PC0..PC3 (ULN2003 + motor unipolar 5V, half-step de 8 fases)
    - LEDs PA11/PA12/PA15
    - Buzzer PA7
    - Timers: SysTick=1ms, TIM21(stepper), TIM22(7seg), TIM2(1Hz lockout)

    while(1) vacío y SIN delay bloqueante.
*/

/*** ------------------ CONFIG STEPPER / LOCK ------------------ ***/
#define LOCK_STEPS_PER_REV   4096u
#define LOCK_STEP_RATE_HZ    800u
#define LOCK_OPEN_STEPS      (LOCK_STEPS_PER_REV/2) //

/*** ------------------ USART2 DEBUG ------------------ ***/
static void USART2_init(void)
{
    RCC->APB1ENR |= (1u<<17);     // clock USART2
    RCC->IOPENR  |= (1u<<0);      // clock GPIOA

    // PA2/PA3 alternate function AF4
    GPIOA->MODER &= ~((3u<<(2*2)) | (3u<<(3*2)));
    GPIOA->MODER |=  ((2u<<(2*2)) | (2u<<(3*2)));
    GPIOA->AFR[0] &= ~((0xFu<<8) | (0xFu<<12));
    GPIOA->AFR[0] |=  ((4u<<8) | (4u<<12));

    USART2->BRR = 0x0683;         // ~9600 baud @16MHz
    USART2->CR1 = (1u<<2) | (1u<<3) | (1u<<0); // RE | TE | UE
}

static void USART2_write(uint8_t ch)
{
    while(!(USART2->ISR & 0x0080)) { }
    USART2->TDR = ch;
}

static void USART2_puts(const char *s)
{
    while(*s) USART2_write((uint8_t)*s++);
    USART2_write('\r');
    USART2_write('\n');
}

/*** ------------------ LEDS & BUZZER ------------------ ***/
#define LED_READY_PIN   11u   // PA11 (cerrado/listo)
#define LED_MOVING_PIN  12u   // PA12 (abriendo/cerrando)
#define LED_OPEN_PIN    15u   // PA15 (abierto / debug keypress)

static inline void LED_READY_ON (void){ GPIOA->BSRR = (1u<<LED_READY_PIN); }
static inline void LED_READY_OFF(void){ GPIOA->BSRR = (1u<<(LED_READY_PIN+16)); }

static inline void LED_MOVING_ON (void){ GPIOA->BSRR = (1u<<LED_MOVING_PIN); }
static inline void LED_MOVING_OFF(void){ GPIOA->BSRR = (1u<<(LED_MOVING_PIN+16)); }

static inline void LED_OPEN_ON (void){ GPIOA->BSRR = (1u<<LED_OPEN_PIN); }
static inline void LED_OPEN_OFF(void){ GPIOA->BSRR = (1u<<(LED_OPEN_PIN+16)); }

static inline void BUZZER_ON (void){ GPIOA->BSRR = (1u<<7); }
static inline void BUZZER_OFF(void){ GPIOA->BSRR = (1u<<(7+16)); }

/*** ------------------ ESTADO GLOBAL DE LA CAJA FUERTE ------------------ ***/
#define LOCK_CODE_LEN 4

// Código correcto inicial: "1234" (ahora modificable en RAM)
static char lock_correct_code[LOCK_CODE_LEN] = { '1','2','3','4' };


// Clave maestra para quitar el bloqueo: "ABCD"
#define OVERRIDE_CODE_LEN 4
static const char override_code[OVERRIDE_CODE_LEN] = { 'A','B','C','D' };


typedef enum {
    LOCK_STATE_CLOSED = 0,
    LOCK_STATE_OPENING,
    LOCK_STATE_OPEN,
    LOCK_STATE_CLOSING,
    LOCK_STATE_BLOCKED
} lock_state_t;
static volatile lock_state_t lock_state = LOCK_STATE_CLOSED;



// Modo de ingreso de PIN (normal o cambio de PIN)
typedef enum {
    PIN_MODE_NORMAL = 0,      // ingresar PIN para abrir
    PIN_MODE_CHANGE_OLD,      // pidiendo PIN actual
    PIN_MODE_CHANGE_NEW       // pidiendo PIN nuevo
} pin_mode_t;
static volatile pin_mode_t pin_mode = PIN_MODE_NORMAL;



// Modo de ajuste de hora (reloj HH:MM)
typedef enum {
    CLOCK_SET_NONE = 0,   // sin ajuste
    CLOCK_SET_HOURS,      // ingresando horas (HH)
    CLOCK_SET_MINUTES     // ingresando minutos (MM)
} clock_set_mode_t;

static volatile clock_set_mode_t clock_set_mode = CLOCK_SET_NONE;
static volatile uint8_t clock_set_digits = 0;  // cuántos dígitos llevamos (0..2)
static volatile uint8_t clock_set_val    = 0;  // valor parcial (0..59)

// Buffer de los 4 dígitos que se van introduciendo en modo ajuste (HHMM)
static volatile uint8_t clock_set_buf[4] = {0,0,0,0};
static volatile uint8_t clock_set_pos    = 0;   // posición siguiente 0..4






static volatile uint8_t  lock_attempts = 0;
static volatile char     lock_entered_code[LOCK_CODE_LEN];
static volatile uint8_t  lock_entered_len = 0;

static volatile uint8_t  lockout_active = 0;
static volatile uint8_t  lockout_seconds_left = 0;

static volatile char     last_key_char = '-';

static volatile uint16_t alarm_ms_left = 0;   // 3s alarma bloqueo
static volatile uint8_t  buzz_mode = 0;       // patrón bip-bip
static volatile uint16_t buzz_timer_ms = 0;

/*** ------------------ DISPLAY 7 SEGMENTOS ------------------ */
// PB0..PB7 segmentos, PC5/PC6/PC8/PC9 dígitos

static volatile uint8_t display_ss = 0xFF; // empieza apagado
static const uint8_t seg_lut[10] =
{
    0b00111111, // 0
    0b00000110, // 1
    0b01011011, // 2
    0b01001111, // 3
    0b01100110, // 4
    0b01101101, // 5
    0b01111101, // 6
    0b00000111, // 7
    0b01111111, // 8
    0b01101111  // 9
};

// Un "underscore" usando solo el segmento inferior (bit 3 = segmento d)
#define SEG_UNDERSCORE 0x08u

static volatile uint8_t seg_digits[4] = {0,0,0,0};

static const uint8_t seg_digit_pins[4] = {5, 6, 8, 9};
#define DIG_TENS_IDX   2
#define DIG_UNITS_IDX  3

// Apagar TODOS los dígitos (ponerlos en HIGH)
static inline void seg_digits_all_off(void)
{
    // PC5, PC6, PC8, PC9 a HIGH => todos OFF
    GPIOC->BSRR = (1u<<5) | (1u<<6) | (1u<<8) | (1u<<9);
}

// Encender SOLO el dígito idx (ponerlo en LOW)
static inline void seg_digit_on(uint8_t idx)
{
    uint8_t pin = seg_digit_pins[idx];
    // LOW usando la mitad alta del BSRR
    GPIOC->BSRR = (1u << (pin + 16));
}

// Escribir patrón de segmentos en PB0..PB7
static inline void seg_write_pattern(uint8_t pattern)
{
    GPIOB->BSRR = (0xFFu << 16);      // limpiar PB0..PB7
    GPIOB->BSRR = (uint32_t)pattern;  // cargar nuevo patrón
}



#ifndef SEG_HAS_DP
#define SEG_HAS_DP 1
#endif
#define SEG_DP_MASK 0x80u  // bit del DP sobre PB7 según tu bus de segmentos

// ——— Reloj en formato 24h ———
static volatile uint8_t clock_hours   = 0;  // 0..23
static volatile uint8_t clock_minutes = 0;  // 0..59
static volatile uint8_t clock_seconds = 0;  // 0..59
static volatile uint8_t colon_blink   = 0;  // 0/1: parpadear ":" cada segundo


//hola


/*** ------------------ LCD NO BLOQUEANTE ------------------ */
#define LCD_RS_PORT GPIOA
#define LCD_E_PORT  GPIOA
#define LCD_D_PORT  GPIOA
#define LCD_RS_PIN  0u
#define LCD_E_PIN   1u
#define LCD_D4_PIN  8u
#define LCD_D5_PIN  10u
#define LCD_D6_PIN  5u
#define LCD_D7_PIN  6u

static inline void LCD_RS(uint8_t v)
{
    if(v) LCD_RS_PORT->ODR |=  (1u<<LCD_RS_PIN);
    else  LCD_RS_PORT->ODR &= ~(1u<<LCD_RS_PIN);
}
static inline void LCD_E(uint8_t v)
{
    if(v) LCD_E_PORT->ODR |=  (1u<<LCD_E_PIN);
    else  LCD_E_PORT->ODR &= ~(1u<<LCD_E_PIN);
}
static inline void lcd_bus_write_nibble(uint8_t n)
{
    // limpio D4..D7
    uint32_t clr = (1u<<LCD_D4_PIN)|(1u<<LCD_D5_PIN)|(1u<<LCD_D6_PIN)|(1u<<LCD_D7_PIN);
    LCD_D_PORT->BSRR = (clr<<16);

    uint32_t set = 0;
    if(n & 0x1) set |= (1u<<LCD_D4_PIN);
    if(n & 0x2) set |= (1u<<LCD_D5_PIN);
    if(n & 0x4) set |= (1u<<LCD_D6_PIN);
    if(n & 0x8) set |= (1u<<LCD_D7_PIN);
    if(set) LCD_D_PORT->BSRR = set;
}

// FSM de la LCD
typedef enum {
    LCD_IDLE,
    LCD_PUT4_HI,
    LCD_PULSE_HI,
    LCD_PUT4_LO,
    LCD_PULSE_LO
} lcd_state_t;

#define LCD_QSIZE 64

static volatile uint8_t    lcd_q[LCD_QSIZE];
static volatile uint8_t    lcd_q_head=0, lcd_q_tail=0;
static volatile lcd_state_t lcd_state = LCD_IDLE;
static volatile uint8_t    lcd_cur_byte=0, lcd_is_data=0;
static volatile uint16_t   lcd_wait_ms=0;

static void lcd_enqueue(uint8_t data, uint8_t is_data)
{
    uint8_t next = (uint8_t)((lcd_q_head+2)&(LCD_QSIZE-1));
    if(next == lcd_q_tail) return;
    lcd_q[lcd_q_head] = is_data;
    lcd_q[(lcd_q_head+1)&(LCD_QSIZE-1)] = data;
    lcd_q_head = next;
}
static inline void lcd_cmd_nb (uint8_t cmd){ lcd_enqueue(cmd, 0); }
static inline void lcd_data_nb(uint8_t ch ){ lcd_enqueue(ch, 1); }
static void lcd_print_nb(const char* s){ while(*s) lcd_data_nb((uint8_t)*s++); }

static void lcd_gpio_init(void)
{
    RCC->IOPENR |= (1u<<0); // GPIOA

    GPIOA->MODER &= ~((3u<<(LCD_RS_PIN*2))|(3u<<(LCD_E_PIN*2))|
                      (3u<<(LCD_D4_PIN*2))|(3u<<(LCD_D5_PIN*2))|
                      (3u<<(LCD_D6_PIN*2))|(3u<<(LCD_D7_PIN*2)));
    GPIOA->MODER |=  ((1u<<(LCD_RS_PIN*2))|(1u<<(LCD_E_PIN*2))|
                      (1u<<(LCD_D4_PIN*2))|(1u<<(LCD_D5_PIN*2))|
                      (1u<<(LCD_D6_PIN*2))|(1u<<(LCD_D7_PIN*2)));
    LCD_RS(0);
    LCD_E(0);
}

static void lcd_init_nb_begin(void)
{
    lcd_cmd_nb(0x33);
    lcd_cmd_nb(0x32);
    lcd_cmd_nb(0x28); // 4 bits, 2 líneas
    lcd_cmd_nb(0x0C); // display ON, cursor OFF
    lcd_cmd_nb(0x06); // auto-increment
    lcd_cmd_nb(0x01); // clear
}

// Muestra en la LCD:
//  Línea 1 → mensaje de estado
//  Línea 2 → "Intentos:X Key:Y"
static void lcd_show_status(const char* line1, char keychar)
{
    lcd_cmd_nb(0x01);                      // clear
    lcd_cmd_nb(0x80); lcd_print_nb(line1); // fila 1

    lcd_cmd_nb(0xC0);                      // fila 2
    lcd_print_nb("Intentos:");
    lcd_data_nb((uint8_t)('0' + lock_attempts));
    lcd_print_nb(" Key:");
    lcd_data_nb((uint8_t)keychar);
}

// Actualiza LEDs según el estado de la cerradura
static inline void lock_update_leds(void)
{
    LED_READY_OFF();
    LED_MOVING_OFF();
    // LED_OPEN no se forza aquí porque también se usa como debug keypress

    switch(lock_state)
    {
    case LOCK_STATE_CLOSED:
        LED_READY_ON();
        break;
    case LOCK_STATE_OPENING:
    case LOCK_STATE_CLOSING:
        LED_MOVING_ON();
        break;
    case LOCK_STATE_OPEN:
        LED_OPEN_ON();
        break;
    case LOCK_STATE_BLOCKED:
        LED_MOVING_ON();
        LED_OPEN_ON();
        break;
    default:
        break;
    }
}

/*** ------------------ STEPPER (cerradura mecánica) ------------------ */
/*
   Reemplazo total de la sección del motor:
   - ULN2003 + motor unipolar (tipo 28BYJ-48, 5V)
   - Bobinas en PC0,PC1,PC2,PC3
   - Secuencia HALF-STEP (8 fases) para más suavidad y torque

   Tabla de 8 fases (half-step):
     idx: 0    1     2    3     4    5     6    7
     pat: A   A+B    B   B+C    C   C+D    D   D+A

   Mapeo pines:
     A -> PC0
     B -> PC1
     C -> PC2
     D -> PC3
*/

static const uint8_t lock_seq[8] = {
    0b0001, // A        -> PC0
    0b0011, // A+B      -> PC0+PC1
    0b0010, // B        -> PC1
    0b0110, // B+C      -> PC1+PC2
    0b0100, // C        -> PC2
    0b1100, // C+D      -> PC2+PC3
    0b1000, // D        -> PC3
    0b1001  // D+A      -> PC3+PC0
};

// escribe el patrón de bobinas en PC0..PC3 usando BSRR
static inline void lock_coils_apply(uint8_t pattern)
{
    // Apagar todas primero (poner PC0..PC3 en LOW)
    GPIOC->BSRR = (0x0Fu << 16);

    uint32_t bsrr_val = 0;
    if (pattern & 0x01) bsrr_val |= (1u<<0); // PC0
    if (pattern & 0x02) bsrr_val |= (1u<<1); // PC1
    if (pattern & 0x04) bsrr_val |= (1u<<2); // PC2
    if (pattern & 0x08) bsrr_val |= (1u<<3); // PC3

    if (bsrr_val) GPIOC->BSRR = bsrr_val;
}

// apaga todas las bobinas (sin torque, menos calor)
static inline void lock_coils_all_off(void)
{
    GPIOC->BSRR = (0x0Fu << 16); // reset PC0..PC3
}

// Estado de posición/objetivo
static volatile int32_t lock_pos_steps    = 0;
static volatile int32_t lock_target_steps = 0;

// índice actual en la secuencia half-step [0..7]
static volatile uint8_t lock_phase_idx    = 0;

// Arrancar/parar TIM21 que hace los pasos
static inline void lock_stepper_start(void)
{
    if (lock_target_steps != lock_pos_steps)
        TIM21->CR1 |= TIM_CR1_CEN;
}
static inline void lock_stepper_stop(void)
{
    TIM21->CR1 &= ~TIM_CR1_CEN;
}

// TIM21: cada update = 1 paso de stepper
void TIM21_IRQHandler(void)
{
    if (TIM21->SR & TIM_SR_UIF)
    {
        TIM21->SR &= ~TIM_SR_UIF;

        if (lock_pos_steps < lock_target_steps)
        {
            // avanzar (sentido "abrir")
            lock_phase_idx++;
            if (lock_phase_idx > 7)
                lock_phase_idx = 0;

            lock_coils_apply(lock_seq[lock_phase_idx]);
            lock_pos_steps++;

            if (lock_pos_steps == lock_target_steps)
            {
                lock_stepper_stop();
                // apaga bobinas para que no caliente
                lock_coils_all_off();

                USART2_puts("Motor posicionado");

                if(lock_pos_steps == 0){
                    lock_state = LOCK_STATE_CLOSED;
                    lock_update_leds();
                    lcd_show_status("Cerrado", last_key_char);
                } else {
                    lock_state = LOCK_STATE_OPEN;
                    lock_update_leds();
                    lcd_show_status("Abierto", last_key_char);
                }
            }
        }
        else if (lock_pos_steps > lock_target_steps)
        {
            // retroceder (sentido "cerrar")
            if (lock_phase_idx == 0)
                lock_phase_idx = 7;
            else
                lock_phase_idx--;

            lock_coils_apply(lock_seq[lock_phase_idx]);
            lock_pos_steps--;

            if (lock_pos_steps == lock_target_steps)
            {
                lock_stepper_stop();
                // apaga bobinas
                lock_coils_all_off();

                USART2_puts("Motor posicionado");

                if(lock_pos_steps == 0){
                    lock_state = LOCK_STATE_CLOSED;
                    lock_update_leds();
                    lcd_show_status("Cerrado", last_key_char);
                } else {
                    lock_state = LOCK_STATE_OPEN;
                    lock_update_leds();
                    lcd_show_status("Abierto", last_key_char);
                }
            }
        }
        else
        {
            lock_stepper_stop();
            lock_coils_all_off();
        }
    }
}

/*** ------------------ KEYPAD 4x4 SCAN ------------------ */
/* Keypad pin mapping según tu hardware:
   R1..R4 -> PB12..PB15 (salidas)
   C1..C4 -> PB8..PB11  (entradas pull-up) */

#define KP_R1   (1u<<12)
#define KP_R2   (1u<<13)
#define KP_R3   (1u<<14)
#define KP_R4   (1u<<15)

#define KP_C1   (1u<<8)
#define KP_C2   (1u<<9)
#define KP_C3   (1u<<10)
#define KP_C4   (1u<<11)

/* Arreglos para recorrer filas/columnas */
static const uint32_t KP_ROWS[4] = { KP_R1, KP_R2, KP_R3, KP_R4 };
static const uint32_t KP_COLS[4] = { KP_C1, KP_C2, KP_C3, KP_C4 };

/* Mapa lógico de teclas [fila][columna] */
static const char keymap[4][4] = {
    {'1','2','3','A'},
    {'4','5','6','B'},
    {'7','8','9','C'},
    {'*','0','#','D'}
};

/* forward declarations (prototipos) */
static inline char keypad_hw_read_once(void);
static void keypad_scan_5ms(void);
static void process_key(char keychar);

/* ========== LECTURA CRUDA DEL KEYPAD ========== */
/* Devuelve la tecla DETECTADA AHORA MISMO (sin debounce).
   '0' si ninguna. */
static inline char keypad_hw_read_once(void)
{
    /* Poner TODAS las filas HIGH (idle) */
    GPIOB->BSRR = KP_R1 | KP_R2 | KP_R3 | KP_R4;

    for (int r = 0; r < 4; r++) {

        /* Bajar SOLO la fila r a LOW */
        GPIOB->BSRR = (KP_ROWS[r] << 16);

        /* pequeño settle */
        __asm volatile ("nop;nop;nop;nop");

        /* Leer columnas PB8..PB11 (pull-up).
           LOW (0) = tecla presionada en esa columna. */
        for (int c = 0; c < 4; c++) {
            if ((GPIOB->IDR & KP_COLS[c]) == 0u) {
                /* restaurar la fila r a HIGH antes de salir */
                GPIOB->BSRR = KP_ROWS[r];
                return keymap[r][c];
            }
        }

        /* restaurar esa fila a HIGH antes de seguir */
        GPIOB->BSRR = KP_ROWS[r];
    }

    return 0; // nada presionado
}

/* ========== SCAN + DEBOUNCE ========== */
#define KP_DEBOUNCE_TICKS  3   // 3 * 5ms = 15ms estable

/* Esta se llama cada ~5ms desde SysTick_Handler.
   Hace debounce y genera un evento de tecla solo en el flanco de PRESIONAR. */
static void keypad_scan_5ms(void)
{
    static char    sample_prev     = 0;
    static uint8_t stable_cnt      = 0;
    static char    stable_key      = 0;
    static uint8_t pressed_latched = 0;

    char k = keypad_hw_read_once();  // lectura cruda instantánea

    // ¿la lectura actual es igual a la anterior?
    if (k == sample_prev) {
        if (stable_cnt < 0xFF) {
            stable_cnt++;
        }
    } else {
        sample_prev = k;
        stable_cnt  = 1;
    }

    // ¿ya se estabilizó la tecla por suficiente tiempo?
    if (stable_cnt >= KP_DEBOUNCE_TICKS) {

        // si cambió la tecla estable, reinicio el latch
        if (k != stable_key) {
            stable_key = k;
            pressed_latched = 0;
        }

        // si hay tecla estable Y todavía no la reportamos -> evento "keydown"
        if (stable_key != 0 && !pressed_latched) {
            pressed_latched = 1;

            // debug visual: prende LED_OPEN mientras hay tecla
            LED_OPEN_ON();

            // manda la tecla a tu lógica del candado
            process_key(stable_key);
        }

        // si ya no hay tecla estable -> soltar latch y apagar LED_OPEN
        if (stable_key == 0) {
            pressed_latched = 0;
            LED_OPEN_OFF();
        }
    }
}

/*** ------------------ LÓGICA DE TECLA (PIN, CERRAR, BLOQUEO) ------------------ */

// Devuelve 1 cuando se ha recibido la secuencia completa "ABCD"
static uint8_t check_override_sequence(char keychar)
{
    static uint8_t idx = 0; // cuántos caracteres correctos llevamos

    if (keychar == override_code[idx])
    {
        // el carácter coincide con el esperado en la posición actual
        idx++;
        if (idx >= OVERRIDE_CODE_LEN)
        {
            // se completó "ABCD"
            idx = 0;
            return 1;
        }
    }
    else
    {
        // no coincide con lo esperado, ver si este char puede ser inicio de la secuencia
        if (keychar == override_code[0])
        {
            // empezamos de nuevo desde 'A'
            idx = 1;
        }
        else
        {
            // cualquier otra cosa rompe la secuencia
            idx = 0;
        }
    }
    return 0;
}




static void process_key(char keychar)
{
    // Debug inmediato al UART
    USART2_write('[');
    USART2_write((uint8_t)keychar);
    USART2_write(']');
    USART2_write('\r');
    USART2_write('\n');

    last_key_char = keychar; // se verá en "Key:X"

    // ====== 1) Clave maestra para quitar bloqueo (ABCD) ======
    if (lockout_active || lock_state == LOCK_STATE_BLOCKED)
    {
        if (check_override_sequence(keychar))
        {
            // --- CANCELAR BLOQUEO ---
            lockout_active        = 0;
            lockout_seconds_left  = 0;
            display_ss            = 0;
            lock_attempts         = 0;

            // Apagamos cualquier alarma / patrón de buzzer
            alarm_ms_left = 0;
            buzz_mode     = 0;
            BUZZER_OFF();

            // Si estaba en estado bloqueado, volver a "cerrado"
            if (lock_state == LOCK_STATE_BLOCKED)
            {
                lock_state = LOCK_STATE_CLOSED;
                lock_update_leds();
            }

            lcd_show_status("Desbloqueado", last_key_char);
            USART2_puts("Override ABCD: bloqueo cancelado");
        }
        else
        {
            lcd_show_status("Bloqueado", last_key_char);
        }
        return;
    }

    // ====== 2) Tecla '#' = cerrar caja si está abierta ======
    if (keychar == '#')
    {
        if (lock_state == LOCK_STATE_OPEN)
        {
            lock_state = LOCK_STATE_CLOSING;
            lock_update_leds();

            lock_target_steps = 0;          // volver a cerrado
            lock_stepper_start();

            lcd_show_status("Cerrando...", last_key_char);
            USART2_puts("Cerrando caja");
        }
        else
        {
            // si ya estaba cerrada/no corresponde
            lcd_show_status("Cerrado", last_key_char);
        }
        return;
    }

    // ========= ESTABA CERRADA → PIN, cambio PIN, ajuste de hora =========
    if (lock_state == LOCK_STATE_CLOSED)
    {
        // --- 2.1) Tecla 'D' = entrar/salir de ajuste de hora ---
        if (keychar == 'D')
        {
            // No permitir ajuste de hora si estamos cambiando PIN
            if (pin_mode != PIN_MODE_NORMAL)
            {
                lcd_show_status("Fin Cambio PIN", last_key_char);
                USART2_puts("No se puede ajustar hora mientras cambia PIN");
                return;
            }

            if (clock_set_mode == CLOCK_SET_NONE)
            {
                clock_set_mode   = CLOCK_SET_HOURS;
                clock_set_digits = 0;
                clock_set_val    = 0;
                clock_set_pos    = 0;
                clock_set_buf[0] = clock_set_buf[1] = clock_set_buf[2] = clock_set_buf[3] = 0;
                lcd_show_status("Set Hora", last_key_char);
                USART2_puts("Ajuste hora: ingrese HH (0-23)");
            }
            else
            {
                // Cancelar ajuste de hora
                clock_set_mode   = CLOCK_SET_NONE;
                clock_set_digits = 0;
                clock_set_val    = 0;
                clock_set_pos    = 0;
                lcd_show_status("Ingrese PIN", last_key_char);
                USART2_puts("Ajuste de hora cancelado");
            }
            return;
        }

        // --- 2.2) Tecla '*' = entrar/salir de cambio de PIN ---
        if (keychar == '*')
        {
            // No permitir cambio de PIN si estamos ajustando hora
            if (clock_set_mode != CLOCK_SET_NONE)
            {
                lcd_show_status("Fin ajuste hora", last_key_char);
                USART2_puts("Termine ajuste de hora antes de cambiar PIN");
                return;
            }

            if (pin_mode == PIN_MODE_NORMAL)
            {
                pin_mode = PIN_MODE_CHANGE_OLD;
                lock_entered_len = 0;
                lcd_show_status("PIN actual", last_key_char);
                USART2_puts("Modo cambio PIN: ingrese PIN actual");
            }
            else
            {
                // '*' nuevamente -> cancelar cambio de PIN
                pin_mode = PIN_MODE_NORMAL;
                lock_entered_len = 0;
                lcd_show_status("Ingrese PIN", last_key_char);
                USART2_puts("Cambio PIN cancelado");
            }
            return;
        }

        // --- 2.3) Si es dígito, según el modo activo ---
        if (keychar >= '0' && keychar <= '9')
        {
            uint8_t i;
            uint8_t d = (uint8_t)(keychar - '0');

            // ----- MODO AJUSTE DE HORA: HORAS (HH) -----
            if (clock_set_mode == CLOCK_SET_HOURS)
            {
                if (clock_set_digits < 2 && clock_set_pos < 4)
                {
                    clock_set_buf[clock_set_pos] = d;      // guarda primer dígito en pos 0, luego 1
                    clock_set_pos++;
                    clock_set_val = (uint8_t)(clock_set_val*10u + d);
                    clock_set_digits++;
                }

                if (clock_set_digits >= 2)
                {
                    if (clock_set_val <= 23u)
                    {
                        clock_hours = clock_set_val;

                        // Pasar a ajuste de minutos -> ahora se llenarán pos 2 y 3
                        clock_set_mode   = CLOCK_SET_MINUTES;
                        clock_set_digits = 0;
                        clock_set_val    = 0;
                        lcd_show_status("Set Minutos", last_key_char);
                        USART2_puts("Horas ajustadas, ingrese MM (0-59)");
                    }
                    else
                    {
                        // Hora inválida
                        clock_set_mode   = CLOCK_SET_NONE;
                        clock_set_digits = 0;
                        clock_set_val    = 0;
                        clock_set_pos    = 0;
                        lcd_show_status("Hora invalida", last_key_char);
                        USART2_puts("Hora invalida (0-23)");
                    }
                }
                else
                {
                    lcd_show_status("Set Hora", last_key_char);
                }
                return;
            }

            // ----- MODO AJUSTE DE HORA: MINUTOS (MM) -----
            if (clock_set_mode == CLOCK_SET_MINUTES)
            {
                if (clock_set_digits < 2 && clock_set_pos < 4)
                {
                    clock_set_buf[clock_set_pos] = d;      // guarda en pos 2 y 3
                    clock_set_pos++;
                    clock_set_val = (uint8_t)(clock_set_val*10u + d);
                    clock_set_digits++;
                }

                if (clock_set_digits >= 2)
                {
                    if (clock_set_val <= 59u)
                    {
                        clock_minutes = clock_set_val;
                        clock_seconds = 0;

                        clock_set_mode   = CLOCK_SET_NONE;
                        clock_set_digits = 0;
                        clock_set_val    = 0;
                        clock_set_pos    = 0;

                        // bip-bip éxito (si no hay alarma activa)
                        if (alarm_ms_left == 0)
                        {
                            buzz_mode = 1;
                            buzz_timer_ms = 100;
                            BUZZER_ON();
                        }

                        lcd_show_status("Hora ajustada", last_key_char);
                        USART2_puts("Hora ajustada HH:MM");
                    }
                    else
                    {
                        clock_set_mode   = CLOCK_SET_NONE;
                        clock_set_digits = 0;
                        clock_set_val    = 0;
                        clock_set_pos    = 0;
                        lcd_show_status("Minuto invalido", last_key_char);
                        USART2_puts("Minuto invalido (0-59)");
                    }
                }
                else
                {
                    lcd_show_status("Set Minutos", last_key_char);
                }
                return;
            }

            // ----- MODO CAMBIO: verificar PIN actual -----
            if (pin_mode == PIN_MODE_CHANGE_OLD)
            {
                if (lock_entered_len < LOCK_CODE_LEN) {
                    lock_entered_code[lock_entered_len++] = keychar;
                }

                if (lock_entered_len >= LOCK_CODE_LEN)
                {
                    uint8_t ok = 1u;
                    for (i = 0; i < LOCK_CODE_LEN; i++) {
                        if (lock_entered_code[i] != lock_correct_code[i]) {
                            ok = 0u;
                            break;
                        }
                    }

                    lock_entered_len = 0;

                    if (ok)
                    {
                        // PIN actual correcto -> pedir nuevo PIN
                        pin_mode = PIN_MODE_CHANGE_NEW;
                        lcd_show_status("Nuevo PIN", last_key_char);
                        USART2_puts("PIN actual correcto, ingrese nuevo PIN");
                    }
                    else
                    {
                        // contamos como intento fallido normal
                        lock_attempts++;
                        USART2_puts("PIN actual incorrecto");

                        if (lock_attempts >= 3)
                        {
                            // >>>>> ENTRAMOS A BLOQUEO <<<<<
                            lock_state = LOCK_STATE_BLOCKED;
                            lock_update_leds();

                            lockout_seconds_left = 15;
                            display_ss            = lockout_seconds_left;
                            lockout_active        = 1;

                            // alarma 3s
                            alarm_ms_left = 3000;
                            BUZZER_ON();

                            lcd_show_status("Bloqueado", last_key_char);
                            USART2_puts("Bloqueado 15s");
                            pin_mode = PIN_MODE_NORMAL;
                        }
                        else
                        {
                            lcd_show_status("Pin Incorrecto", last_key_char);
                            // para volver a intentar cambio de PIN, presiona '*' de nuevo
                            pin_mode = PIN_MODE_NORMAL;
                        }
                    }
                }
                else
                {
                    lcd_show_status("PIN actual", last_key_char);
                }
                return;
            }

            // ----- MODO CAMBIO: guardar nuevo PIN -----
            if (pin_mode == PIN_MODE_CHANGE_NEW)
            {
                if (lock_entered_len < LOCK_CODE_LEN) {
                    lock_entered_code[lock_entered_len++] = keychar;
                }

                if (lock_entered_len >= LOCK_CODE_LEN)
                {
                    // Copiar nuevo PIN a la variable global
                    for (i = 0; i < LOCK_CODE_LEN; i++) {
                        lock_correct_code[i] = lock_entered_code[i];
                    }

                    lock_entered_len = 0;
                    pin_mode         = PIN_MODE_NORMAL;
                    lock_attempts    = 0;

                    // bip-bip éxito (si no hay alarma activa)
                    if (alarm_ms_left == 0) {
                        buzz_mode = 1;
                        buzz_timer_ms = 100;
                        BUZZER_ON();
                    }

                    lcd_show_status("PIN cambiado", last_key_char);
                    USART2_puts("Nuevo PIN guardado");
                }
                else
                {
                    lcd_show_status("Nuevo PIN", last_key_char);
                }
                return;
            }

            // ----- MODO NORMAL: ingreso de PIN para abrir -----
            if (pin_mode == PIN_MODE_NORMAL)
            {
                if (lock_entered_len < LOCK_CODE_LEN) {
                    lock_entered_code[lock_entered_len++] = keychar;
                }

                if (lock_entered_len >= LOCK_CODE_LEN)
                {
                    uint8_t ok = 1u;
                    for (i = 0; i < LOCK_CODE_LEN; i++) {
                        if (lock_entered_code[i] != lock_correct_code[i]) {
                            ok = 0u;
                            break;
                        }
                    }

                    // reset del buffer para el próximo intento
                    lock_entered_len = 0;

                    if (ok)
                    {
                        // ***** PIN CORRECTO *****
                        lock_attempts = 0;

                        lock_state = LOCK_STATE_OPENING;
                        lock_update_leds();

                        lock_target_steps = (int32_t)LOCK_OPEN_STEPS;
                        lock_stepper_start();

                        // bip-bip éxito (si no hay alarma activa)
                        if (alarm_ms_left == 0) {
                            buzz_mode = 1;
                            buzz_timer_ms = 100;
                            BUZZER_ON();
                        }

                        lcd_show_status("Abriendo...", last_key_char);
                        USART2_puts("PIN correcto, abriendo");
                    }
                    else
                    {
                        // ***** PIN INCORRECTO *****
                        lock_attempts++;
                        USART2_puts("PIN incorrecto");

                        if (lock_attempts >= 3)
                        {
                            // >>>>> ENTRAMOS A BLOQUEO <<<<<
                            lock_state = LOCK_STATE_BLOCKED;
                            lock_update_leds();

                            lockout_seconds_left = 15;      // 15 segundos de castigo
                            display_ss            = lockout_seconds_left;
                            lockout_active        = 1;

                            // alarma 3s
                            alarm_ms_left = 3000;
                            BUZZER_ON();

                            lcd_show_status("Bloqueado", last_key_char);
                            USART2_puts("Bloqueado 15s");
                        }
                        else
                        {
                            // todavía le quedan intentos
                            lcd_show_status("Pin Incorrecto", last_key_char);
                        }
                    }
                }
                else
                {
                    // todavía no son 4 dígitos
                    lcd_show_status("Ingrese PIN", last_key_char);
                }
                return;
            }
        }

        // cualquier otra tecla (A/B/C) estando cerrado:
        lcd_show_status("Ingrese PIN", last_key_char);
        return;
    }

    // ========= Si la caja YA está abierta (LOCK_STATE_OPEN) =========
    if (lock_state == LOCK_STATE_OPEN)
    {
        lcd_show_status("Abierto", last_key_char);
        return;
    }

    // ========= Si está abriendo / cerrando =========
    lcd_show_status("Espere", last_key_char);
}






/*** ------------------ SysTick_Handler ------------------ */
/*
    SysTick cada 1ms:
      - FSM LCD
      - Buzzer FSM
      - Escaneo keypad (edge simple)
      - LED_OPEN (PA15) ON mientras haya tecla presionada
*/
void SysTick_Handler(void)
{
    // ---------- LCD FSM ----------
    if(lcd_wait_ms){
        lcd_wait_ms--;
    } else {
        switch(lcd_state)
        {
        case LCD_IDLE:
            if(lcd_q_tail != lcd_q_head){
                lcd_is_data = lcd_q[lcd_q_tail];
                lcd_q_tail = (uint8_t)((lcd_q_tail+1)&(LCD_QSIZE-1));
                lcd_cur_byte = lcd_q[lcd_q_tail];
                lcd_q_tail = (uint8_t)((lcd_q_tail+1)&(LCD_QSIZE-1));
                LCD_RS(lcd_is_data);
                lcd_state = LCD_PUT4_HI;
            }
            break;
        case LCD_PUT4_HI:
        {
            uint8_t hi = (lcd_cur_byte>>4)&0x0F;
            lcd_bus_write_nibble(hi);
            LCD_E(1);
            lcd_wait_ms = 1;
            lcd_state = LCD_PULSE_HI;
        } break;
        case LCD_PULSE_HI:
            LCD_E(0);
            lcd_wait_ms = 1;
            lcd_state = LCD_PUT4_LO;
            break;
        case LCD_PUT4_LO:
        {
            uint8_t lo = lcd_cur_byte & 0x0F;
            lcd_bus_write_nibble(lo);
            LCD_E(1);
            lcd_wait_ms = 1;
            lcd_state = LCD_PULSE_LO;
        } break;
        case LCD_PULSE_LO:
            LCD_E(0);
            if(!lcd_is_data && (lcd_cur_byte==0x01u || lcd_cur_byte==0x02u)){
                lcd_wait_ms = 2;
            }
            lcd_state = LCD_IDLE;
            break;
        default:
            lcd_state = LCD_IDLE;
            break;
        }
    }

    // ---------- BUZZER FSM ----------
    if(alarm_ms_left){
        alarm_ms_left--;
        BUZZER_ON();
        if(alarm_ms_left==0){
            if(buzz_mode==0){
                BUZZER_OFF();
            }
        }
    } else {
        if(buzz_mode){
            if(buzz_timer_ms){
                buzz_timer_ms--;
            } else {
                switch(buzz_mode)
                {
                case 1:
                    BUZZER_OFF();
                    buzz_mode = 2;
                    buzz_timer_ms = 100; // 100ms OFF
                    break;
                case 2:
                    BUZZER_ON();
                    buzz_mode = 3;
                    buzz_timer_ms = 100; // 100ms ON
                    break;
                case 3:
                    BUZZER_OFF();
                    buzz_mode = 4;
                    buzz_timer_ms = 100; // 100ms OFF final
                    break;
                case 4:
                default:
                    buzz_mode = 0;
                    BUZZER_OFF();
                    break;
                }
            }
        }
    }

    // ---------- KEYPAD SCAN ----------
    {
        static uint8_t kp_div = 0;
        kp_div++;
        if (kp_div >= 5) {   // cada ~5ms
            kp_div = 0;
            keypad_scan_5ms();
        }
    }
}

/*** ------------------ TIM22_IRQHandler: multiplex 7seg ------------------ */
void TIM22_IRQHandler(void)
{
    if (TIM22->SR & TIM_SR_UIF)
    {
        TIM22->SR &= ~TIM_SR_UIF;

        // 1) Si hay BLOQUEO ACTIVO => se mantiene el contador (lógica original)
        if (lockout_active)
        {
            static uint8_t which = DIG_TENS_IDX; // alterna entre decenas y unidades

            // Apagamos todos
            seg_digits_all_off();

            uint8_t val;
            uint8_t blank = 0;

            if (which == DIG_TENS_IDX)
            {
                val = (display_ss / 10) % 10;
                if (val == 0)
                    blank = 1; // sin cero a la izquierda
            }
            else // unidades
            {
                val = display_ss % 10;
            }

            seg_write_pattern(blank ? 0x00 : seg_lut[val]);
            seg_digit_on(which);

            which = (which == DIG_TENS_IDX) ? DIG_UNITS_IDX : DIG_TENS_IDX;
            return;
        }

        // 2) Si NO hay bloqueo => reloj / modo ajuste
        static uint8_t which4 = 0; // 0..3 -> Hten, Hun, Mten, Mun

        // Apagamos todos antes de cambiar
        seg_digits_all_off();

        uint8_t pattern = 0;

        // --- MODO AJUSTE DE HORA: mostrar dígitos escritos y "_" parpadeando ---
        if (clock_set_mode != CLOCK_SET_NONE)
        {
            if (which4 < clock_set_pos)
            {
                // Este dígito ya fue ingresado -> mostrarlo
                uint8_t d = clock_set_buf[which4];
                if (d < 10u)
                    pattern = seg_lut[d];
                else
                    pattern = 0x00;
            }
            else
            {
                // Este dígito aún no se ingresa -> mostrar "_" parpadeante
                if (colon_blink)
                    pattern = SEG_UNDERSCORE;  // solo la línea de abajo
                else
                    pattern = 0x00;            // apagado
            }
        }
        else
        {
            // --- MODO NORMAL: mostrar HH:MM con ":" parpadeando ---
            uint8_t val = 0;
            uint8_t dp_on = 0;

            switch (which4)
            {
            case 0: // decena de hora
                val   = (uint8_t)(clock_hours / 10u);
                dp_on = 0;
                break;
            case 1: // unidad de hora
                val   = (uint8_t)(clock_hours % 10u);
                dp_on = colon_blink ? 1u : 0u;   // DPON para la "mitad" izquierda del ":"
                break;
            case 2: // decena de minuto
                val   = (uint8_t)(clock_minutes / 10u);
                dp_on = colon_blink ? 1u : 0u;   // DPON para la "mitad" derecha del ":"
                break;
            default: // 3 -> unidad de minuto
                val   = (uint8_t)(clock_minutes % 10u);
                dp_on = 0;
                break;
            }

            pattern = seg_lut[val];
#if SEG_HAS_DP
            if (dp_on)
                pattern |= SEG_DP_MASK;
#endif
        }

        // Enviar patrón y encender SOLO ese dígito
        seg_write_pattern(pattern);
        seg_digit_on(which4);

        // Siguiente dígito
        which4 = (uint8_t)((which4 + 1u) & 0x03u);
    }
}



/*** ------------------ TIM2_IRQHandler: 1 Hz lockout countdown ------------------ */
void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_UIF)
    {
        TIM2->SR &= ~TIM_SR_UIF;

        // ——— 1) Contador de bloqueo (lógica existente) ———
        if (lockout_active)
        {
            if (lockout_seconds_left > 0)
            {
                lockout_seconds_left--;
                display_ss = lockout_seconds_left;
            }

            if (lockout_seconds_left == 0)
            {
                lockout_active        = 0;
                lock_attempts         = 0;
                display_ss            = 0; // "00"

                if (lock_state == LOCK_STATE_BLOCKED){
                    lock_state = LOCK_STATE_CLOSED;
                    lock_update_leds();
                }

                lcd_show_status("Ingrese PIN", last_key_char);
                USART2_puts("Bloqueo finalizado");
            }
        }

        // ——— 2) Tick de reloj 24h (siempre corre, esté o no en bloqueo) ———
        clock_seconds++;
        if (clock_seconds >= 60)
        {
            clock_seconds = 0;
            clock_minutes++;
            if (clock_minutes >= 60)
            {
                clock_minutes = 0;
                clock_hours++;
                if (clock_hours >= 24)
                    clock_hours = 0;
            }
        }

        // Parpadeo del "dos puntos" (cada segundo)
        colon_blink ^= 1u;
    }
}




/*** ------------------ main() ------------------ */
int main(void)
{
    // HSI16 como SYSCLK
    RCC->CR   |= (1u<<0);  // HSION=1
    RCC->CFGR |= (1u<<0);  // SW=01 -> HSI16

    // Habilitar clocks GPIO A/B/C
    RCC->IOPENR |= (1u<<0) | (1u<<1) | (1u<<2);

    // UART debug
    USART2_init();
    USART2_puts("Caja Fuerte Inicializada");
    USART2_puts("Ingrese PIN");

    // ---------- STEPPER: PC0..PC3 como salida push-pull ----------
    GPIOC->MODER &= ~((3u<<(0*2))|(3u<<(1*2))|(3u<<(2*2))|(3u<<(3*2)));
    GPIOC->MODER |=  ((1u<<(0*2))|(1u<<(1*2))|(1u<<(2*2))|(1u<<(3*2)));

    // opcional: asegurar push-pull, low-speed, sin pull
    GPIOC->OTYPER  &= ~(0x0Fu);                  // PC0..PC3 push-pull
    GPIOC->OSPEEDR &= ~((3u<<(0*2))|
                        (3u<<(1*2))|
                        (3u<<(2*2))|
                        (3u<<(3*2)));            // low speed
    GPIOC->PUPDR   &= ~((3u<<(0*2))|
                        (3u<<(1*2))|
                        (3u<<(2*2))|
                        (3u<<(3*2)));            // no pull

    // Fase inicial del motor: arrancamos con todo apagado
    lock_phase_idx = 0;
    lock_coils_all_off();

    // PC4 y PC7 salidas en alto (válvulas deshabilitadas / relés inactivos)
    GPIOC->MODER &= ~((3u<<(4*2)) | (3u<<(7*2)));
    GPIOC->MODER |=  ((1u<<(4*2)) | (1u<<(7*2)));
    GPIOC->BSRR = (1u<<4);
    GPIOC->BSRR = (1u<<7);

    // 7-seg: PB0..PB7 salida (segmentos)
    GPIOB->MODER &= ~((3u<<(0*2))|(3u<<(1*2))|(3u<<(2*2))|(3u<<(3*2))|
                      (3u<<(4*2))|(3u<<(5*2))|(3u<<(6*2))|(3u<<(7*2)));
    GPIOB->MODER |=  ((1u<<(0*2))|(1u<<(1*2))|(1u<<(2*2))|(1u<<(3*2))|
                      (1u<<(4*2))|(1u<<(5*2))|(1u<<(6*2))|(1u<<(7*2)));

    // Dígitos PC5/PC6/PC8/PC9 salida
    GPIOC->MODER &= ~((3u<<(5*2))|(3u<<(6*2))|(3u<<(8*2))|(3u<<(9*2)));
    GPIOC->MODER |=  ((1u<<(5*2))|(1u<<(6*2))|(1u<<(8*2))|(1u<<(9*2)));
    lockout_active = 0;
    lockout_seconds_left = 0;
    display_ss            = 0;

    // ---------- KEYPAD ----------
    // Filas PB12..PB15 = salida
    GPIOB->MODER &= ~(0xFFu << 24);   // limpia PB12..PB15
    GPIOB->MODER |=  (0x55u << 24);   // 01 para cada pin -> salida
    // arrancar todas las filas HIGH
    GPIOB->BSRR = KP_R1 | KP_R2 | KP_R3 | KP_R4;

    // Columnas PB8..PB11 = entrada con pull-up
    GPIOB->MODER &= ~(0xFFu << 16);   // PB8..PB11 input
    GPIOB->PUPDR &= ~(0xFFu << 16);
    GPIOB->PUPDR |=  (0x55u << 16);   // pull-up interno en cada columna

    // LEDs PA11/PA12/PA15
    GPIOA->MODER &= ~((3u<<(LED_READY_PIN*2)) | (3u<<(LED_MOVING_PIN*2)) | (3u<<(LED_OPEN_PIN*2)));
    GPIOA->MODER |=  ((1u<<(LED_READY_PIN*2)) | (1u<<(LED_MOVING_PIN*2)) | (1u<<(LED_OPEN_PIN*2)));
    LED_READY_OFF();
    LED_MOVING_OFF();
    LED_OPEN_OFF();
    lock_state = LOCK_STATE_CLOSED;
    lock_update_leds();

    // Buzzer PA7 salida
    GPIOA->MODER &= ~(3u<<(7*2));
    GPIOA->MODER |=  (1u<<(7*2));
    BUZZER_OFF();

    // LCD GPIO + SysTick 1ms
    lcd_gpio_init();
    SysTick->LOAD  = (16000u - 1u);  // 16MHz -> 1ms
    SysTick->VAL   = 0;
    SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
                     SysTick_CTRL_TICKINT_Msk   |
                     SysTick_CTRL_ENABLE_Msk;

    lcd_init_nb_begin();
    lcd_show_status("Ingrese PIN", last_key_char);

    // TIM21: stepper (cada update = un paso half-step)
    RCC->APB2ENR |= (1u<<2);          // TIM21 clock
    TIM21->PSC = 160u - 1u;           // 16MHz/160 = 100kHz base
    TIM21->ARR = (100000u/LOCK_STEP_RATE_HZ) - 1u; // ~400 Hz stepping
    TIM21->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM21_IRQn);
    // NOTA: no lo encendemos aquí. Se enciende con lock_stepper_start().

    // TIM22: display 7 seg multiplex rápido (~2kHz)
    RCC->APB2ENR |= (1u<<5);          // TIM22 clock
    TIM22->PSC = 160u - 1u;           // 16MHz/160 = 100kHz
    TIM22->ARR = 50u - 1u;            // 100kHz/50 = 2kHz
    TIM22->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM22_IRQn);
    TIM22->CR1 |= TIM_CR1_CEN;

    // TIM2: 1 Hz (lockout countdown)
    RCC->APB1ENR |= (1u<<0);          // TIM2 clock
    TIM2->PSC = 15999u;               // 16MHz/(15999+1)=1000 Hz
    TIM2->ARR = 999u;                 // 1000/(999+1)=1 Hz
    TIM2->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM2_IRQn);
    TIM2->CR1 |= TIM_CR1_CEN;

    __enable_irq(); // habilitar interrupciones globales



    while (1)
    {
        // loop vacío, todo por interrupciones
    }
}
