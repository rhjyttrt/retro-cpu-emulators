#ifndef F_CPU
#define F_CPU 20000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stddef.h>

#define BUS_PORT            PORTA
#define BUS_PIN             PINA
#define BUS_DDR             DDRA

#define CM_RAM_PORT         PORTB
#define CM_RAM_DDR          DDRB

#define CTRL_PORT           PORTC
#define CTRL_DDR            DDRC
#define PHI1_PIN            0
#define PHI2_PIN            1
#define SYNC_PIN            2
#define CM_ROM_PIN          3
#define LATCH_A1_PIN        4
#define LATCH_A2_PIN        5
#define ROM_A0_PIN          6
#define LATCH_A3_PIN        7

#define SYS_PORT            PORTD
#define SYS_PIN             PIND
#define SYS_DDR             DDRD
#define SRAM_LATCH_LOW_PIN  0
#define SRAM_LATCH_HIGH_PIN 1
#define RESET_PIN           2
#define TEST_PIN            3
#define SRAM_WE_PIN         4
#define SRAM_OE_PIN         5
#define SRAM_A8_PIN         6

#define TEST_IS_LOW         (!(SYS_PIN & (1 << TEST_PIN)))
#define RESET_IS_HIGH       (SYS_PIN & (1 << RESET_PIN))

static uint16_t pc = 0x000;
static uint16_t stack[3] = {0, 0, 0};
static uint8_t  reg[16];
static uint8_t  acc = 0;
static uint8_t  carry = 0;
static uint8_t  cm_bank = 1;
static uint8_t  src_reg = 0;

#define CLOCK_PHI1() do { \
    CTRL_PORT |= (1 << PHI1_PIN); \
    __builtin_avr_delay_cycles(7); \
    CTRL_PORT &= ~(1 << PHI1_PIN); \
    __builtin_avr_delay_cycles(2); \
} while(0)

#define CLOCK_PHI2() do { \
    CTRL_PORT |= (1 << PHI2_PIN); \
    __builtin_avr_delay_cycles(7); \
    CTRL_PORT &= ~(1 << PHI2_PIN); \
    __builtin_avr_delay_cycles(4); \
} while(0)

#define CLOCK_PULSE() do { \
    CLOCK_PHI1(); \
    CLOCK_PHI2(); \
} while(0)

static inline void strobe_ctrl_latch(uint8_t pin) __attribute__((always_inline));
static inline void strobe_ctrl_latch(uint8_t pin) {
    asm volatile("" ::: "memory");
    CTRL_PORT |= (1 << pin);
    asm volatile("nop; nop;");
    CTRL_PORT &= ~(1 << pin);
    asm volatile("" ::: "memory");
}

static inline void strobe_sys_latch(uint8_t pin) __attribute__((always_inline));
static inline void strobe_sys_latch(uint8_t pin) {
    asm volatile("" ::: "memory");
    SYS_PORT |= (1 << pin);
    asm volatile("nop; nop;");
    SYS_PORT &= ~(1 << pin);
    asm volatile("" ::: "memory");
}

uint8_t fetch_instruction_nibbles(uint16_t target_addr) {
    uint8_t opcode = 0;

    CTRL_PORT |= (1 << SYNC_PIN); 
    CTRL_PORT &= ~(1 << CM_ROM_PIN); 
    
    BUS_PORT = (BUS_PORT & 0xF0) | (target_addr & 0x0F);
    BUS_DDR  = (BUS_DDR & 0xF0)  | 0x0F; 
    CLOCK_PHI1(); strobe_ctrl_latch(LATCH_A1_PIN); CLOCK_PHI2();

    CTRL_PORT &= ~(1 << SYNC_PIN); 
    BUS_PORT = (BUS_PORT & 0xF0) | ((target_addr >> 4) & 0x0F);
    CLOCK_PHI1(); strobe_ctrl_latch(LATCH_A2_PIN); CLOCK_PHI2();

    BUS_PORT = (BUS_PORT & 0xF0) | ((target_addr >> 8) & 0x0F);
    CLOCK_PHI1(); strobe_ctrl_latch(LATCH_A3_PIN); CLOCK_PHI2();

    BUS_PORT &= 0xF0; BUS_DDR &= 0xF0; 
    CTRL_PORT &= ~(1 << ROM_A0_PIN);
    CLOCK_PHI1();
    CTRL_PORT |= (1 << PHI2_PIN);
    __builtin_avr_delay_cycles(3);
    opcode = (BUS_PIN & 0x0F) << 4; 
    __builtin_avr_delay_cycles(2);
    CTRL_PORT &= ~(1 << PHI2_PIN);
    __builtin_avr_delay_cycles(4);

    CTRL_PORT |= (1 << ROM_A0_PIN);
    CLOCK_PHI1();
    CTRL_PORT |= (1 << PHI2_PIN);
    __builtin_avr_delay_cycles(3);
    opcode |= (BUS_PIN & 0x0F);
    __builtin_avr_delay_cycles(2);
    CTRL_PORT &= ~(1 << PHI2_PIN);
    __builtin_avr_delay_cycles(4);

    CTRL_PORT |= (1 << CM_ROM_PIN);
    return opcode;
}

static inline void execute_x_phases(uint8_t is_ram_io, uint8_t is_ram_write, uint8_t is_status, uint8_t status_idx, uint8_t ram_data_out, uint8_t *ram_data_in) __attribute__((always_inline));
static inline void execute_x_phases(uint8_t is_ram_io, uint8_t is_ram_write, uint8_t is_status, uint8_t status_idx, uint8_t ram_data_out, uint8_t *ram_data_in) {
    if (is_ram_io) {
        CM_RAM_PORT = (CM_RAM_PORT & 0xF0) | (cm_bank & 0x0F); 
        if (is_status) {
            SYS_PORT |= (1 << SRAM_A8_PIN); 
            BUS_PORT = (BUS_PORT & 0xF0) | (status_idx & 0x03);
            BUS_DDR  = (BUS_DDR & 0xF0)  | 0x0F;
            asm volatile("nop;");
            strobe_sys_latch(SRAM_LATCH_LOW_PIN);
        } else {
            SYS_PORT &= ~(1 << SRAM_A8_PIN); 
        }
    }
    CLOCK_PULSE();

    if (is_ram_io) {
        if (is_ram_write) {
            BUS_PORT = (BUS_PORT & 0xF0) | (ram_data_out & 0x0F);
            BUS_DDR  = (BUS_DDR & 0xF0)  | 0x0F;
            SYS_PORT &= ~(1 << SRAM_WE_PIN); 
        } else {
            BUS_PORT &= 0xF0; BUS_DDR &= 0xF0; 
            asm volatile("nop;"); 
        }
    }
    CLOCK_PULSE();

    if (is_ram_io) {
        if (is_ram_write) {
            SYS_PORT |= (1 << SRAM_WE_PIN); 
            CLOCK_PULSE();
        } else {
            SYS_PORT &= ~(1 << SRAM_OE_PIN); 
            CLOCK_PHI1();
            CTRL_PORT |= (1 << PHI2_PIN);
            __builtin_avr_delay_cycles(3);
            if (ram_data_in) *ram_data_in = BUS_PIN & 0x0F;
            __builtin_avr_delay_cycles(2);
            CTRL_PORT &= ~(1 << PHI2_PIN);
            __builtin_avr_delay_cycles(4);
            SYS_PORT |= (1 << SRAM_OE_PIN);  
        }
    } else {
        CLOCK_PULSE();
    }

    if (is_ram_io && is_status) {
        BUS_PORT = (BUS_PORT & 0xF0) | (src_reg & 0x0F);
        BUS_DDR  = (BUS_DDR & 0xF0)  | 0x0F;
        asm volatile("nop;");
        strobe_sys_latch(SRAM_LATCH_LOW_PIN);
    }

    CM_RAM_PORT &= 0xF0; 
    SYS_PORT &= ~(1 << SRAM_A8_PIN); 
    BUS_PORT &= 0xF0; BUS_DDR &= 0xF0;
}

uint8_t fetch_byte_2() {
    execute_x_phases(0, 0, 0, 0, 0, NULL);
    uint8_t b2 = fetch_instruction_nibbles(pc);
    pc = (pc + 1) & 0x0FFF;
    execute_x_phases(0, 0, 0, 0, 0, NULL);
    return b2;
}

void decode_and_execute() {
    if (RESET_IS_HIGH) {
        pc = 0; acc = 0; carry = 0; cm_bank = 1; src_reg = 0;
        stack[0] = 0; stack[1] = 0; stack[2] = 0;
        for (uint8_t i = 0; i < 16; i++) reg[i] = 0;
        CLOCK_PULSE();
        return;
    }

    uint8_t opcode = fetch_instruction_nibbles(pc);
    pc = (pc + 1) & 0x0FFF;

    uint8_t opa = (opcode >> 4) & 0x0F;
    uint8_t opr = opcode & 0x0F;
    uint8_t temp, res, ram_read_val = 0;

    switch (opa) {
        case 0x0:
            execute_x_phases(0, 0, 0, 0, 0, NULL);
            break;

        case 0x1: {
            uint8_t target_low = fetch_byte_2();
            temp = 0;
            if ((opr & 0x8) && TEST_IS_LOW) temp = 1;
            if ((opr & 0x4) && carry)       temp = 1;
            if ((opr & 0x2) && (acc == 0))  temp = 1;
            
            if (opr & 0x1) temp = !temp;
            
            if (temp) {
                pc = ((pc - 1) & 0x0F00) | target_low;
            }
            break;
        }

        case 0x2:
            if ((opr & 0x01) == 0) {
                uint8_t pair = opr & 0x0E;
                uint8_t data = fetch_byte_2();
                reg[pair]     = (data >> 4) & 0x0F;
                reg[pair + 1] = data & 0x0F;
            } else {
                uint8_t pair = opr & 0x0E;
                src_reg = (reg[pair] << 4) | reg[pair + 1];
                
                BUS_PORT = (BUS_PORT & 0xF0) | (src_reg & 0x0F);
                BUS_DDR  = (BUS_DDR & 0xF0)  | 0x0F;
                asm volatile("nop;");
                strobe_sys_latch(SRAM_LATCH_LOW_PIN);
                
                BUS_PORT = (BUS_PORT & 0xF0) | ((src_reg >> 4) & 0x0F);
                asm volatile("nop;");
                strobe_sys_latch(SRAM_LATCH_HIGH_PIN);
                
                execute_x_phases(0, 0, 0, 0, 0, NULL);
            }
            break;

        case 0x3:
            if ((opr & 0x01) == 0) {
                uint16_t rom_addr = ((pc - 1) & 0x0F00) | (reg[0] << 4) | reg[1];
                execute_x_phases(0, 0, 0, 0, 0, NULL);
                res = fetch_instruction_nibbles(rom_addr);
                execute_x_phases(0, 0, 0, 0, 0, NULL);
                reg[opr & 0x0E]     = (res >> 4) & 0x0F;
                reg[(opr & 0x0E)+1] = res & 0x0F;
            } else {
                uint8_t pair = opr & 0x0E;
                pc = ((pc - 1) & 0x0F00) | (reg[pair] << 4) | reg[pair + 1];
                execute_x_phases(0, 0, 0, 0, 0, NULL);
            }
            break;

        case 0x4: {
            uint8_t target_low = fetch_byte_2();
            pc = ((opr << 8) | target_low) & 0x0FFF;
            break;
        }

        case 0x5: {
            uint8_t target_low = fetch_byte_2();
            stack[2] = stack[1];
            stack[1] = stack[0];
            stack[0] = pc;
            pc = ((opr << 8) | target_low) & 0x0FFF;
            break;
        }

        case 0x6:
            reg[opr] = (reg[opr] + 1) & 0x0F;
            execute_x_phases(0, 0, 0, 0, 0, NULL);
            break;

        case 0x7: {
            uint8_t target_low = fetch_byte_2();
            reg[opr] = (reg[opr] + 1) & 0x0F;
            if (reg[opr] != 0) {
                pc = ((pc - 1) & 0x0F00) | target_low;
            }
            break;
        }

        case 0x8:
            res = acc + reg[opr] + carry;
            carry = (res > 15) ? 1 : 0;
            acc = res & 0x0F;
            execute_x_phases(0, 0, 0, 0, 0, NULL);
            break;

        case 0x9:
            res = acc + (~reg[opr] & 0x0F) + carry;
            carry = (res > 15) ? 1 : 0;
            acc = res & 0x0F;
            execute_x_phases(0, 0, 0, 0, 0, NULL);
            break;

        case 0xA:
            acc = reg[opr];
            execute_x_phases(0, 0, 0, 0, 0, NULL);
            break;

        case 0xB:
            temp = acc; acc = reg[opr]; reg[opr] = temp;
            execute_x_phases(0, 0, 0, 0, 0, NULL);
            break;

        case 0xC:
            acc = opr;
            pc = stack[0];
            stack[0] = stack[1];
            stack[1] = stack[2];
            execute_x_phases(0, 0, 0, 0, 0, NULL);
            break;

        case 0xD:
            acc = opr;
            execute_x_phases(0, 0, 0, 0, 0, NULL);
            break;

        case 0xE:
            if (opr == 0x0) {
                execute_x_phases(1, 1, 0, 0, acc, NULL);
            } else if (opr == 0x1 || opr == 0x3) {
                PORTA = (PORTA & 0x0F) | ((acc & 0x0F) << 4);
                DDRA |= 0xF0; 
                execute_x_phases(0, 0, 0, 0, 0, NULL);
            } else if (opr == 0x2) {
                PORTA = (PORTA & 0x0F) | ((acc & 0x0F) << 4);
                DDRA |= 0xF0; 
                CTRL_PORT &= ~(1 << CM_ROM_PIN);
                execute_x_phases(0, 0, 0, 0, 0, NULL);
                CTRL_PORT |= (1 << CM_ROM_PIN);
            } else if (opr == 0x4) {
                execute_x_phases(1, 0, 0, 0, 0, &ram_read_val);
                acc = ram_read_val;
            } else if (opr == 0x5) {
                PORTA &= 0x0F; 
                DDRA  &= 0x0F; 
                CTRL_PORT &= ~(1 << CM_ROM_PIN);
                CLOCK_PULSE();
                CLOCK_PULSE();
                
                CLOCK_PHI1();
                CTRL_PORT |= (1 << PHI2_PIN);
                __builtin_avr_delay_cycles(3);
                acc = (PINA >> 4) & 0x0F; 
                __builtin_avr_delay_cycles(2);
                CTRL_PORT &= ~(1 << PHI2_PIN);
                __builtin_avr_delay_cycles(4);

                CTRL_PORT |= (1 << CM_ROM_PIN);
                BUS_PORT &= 0xF0; BUS_DDR &= 0xF0;
            } else if (opr == 0x6) {
                execute_x_phases(1, 0, 0, 0, 0, &ram_read_val);
                res = acc + ram_read_val + carry;
                carry = (res > 15) ? 1 : 0;
                acc = res & 0x0F;
            } else if (opr == 0x7) {
                execute_x_phases(1, 0, 0, 0, 0, &ram_read_val);
                res = acc + (~ram_read_val & 0x0F) + carry;
                carry = (res > 15) ? 1 : 0;
                acc = res & 0x0F;
            } else if ((opr & 0x0C) == 0x08) {
                execute_x_phases(1, 1, 1, opr & 0x03, acc, NULL);
            } else if ((opr & 0x0C) == 0x0C) {
                execute_x_phases(1, 0, 1, opr & 0x03, 0, &ram_read_val);
                acc = ram_read_val;
            } else {
                execute_x_phases(0, 0, 0, 0, 0, NULL);
            }
            break;

        case 0xF:
            switch (opr) {
                case 0x0: acc = 0; carry = 0; break;
                case 0x1: carry = 0; break;
                case 0x2: res = acc + 1; carry = (res > 15) ? 1 : 0; acc = res & 0x0F; break;
                case 0x3: carry = !carry; break;
                case 0x4: acc = ~acc & 0x0F; break;
                case 0x5: temp = carry; carry = (acc & 0x08) ? 1 : 0; acc = ((acc << 1) | temp) & 0x0F; break;
                case 0x6: temp = carry; carry = (acc & 0x01) ? 1 : 0; acc = ((acc >> 1) | (temp << 3)) & 0x0F; break;
                case 0x7: acc = carry; carry = 0; break;
                case 0x8: res = acc + 0x0F; carry = (res > 15) ? 1 : 0; acc = res & 0x0F; break;
                case 0x9: acc = carry ? 10 : 9; carry = 0; break;
                case 0xA: carry = 1; break;
                case 0xB:
                    if (carry || acc > 9) {
                        res = acc + 6;
                        carry = carry || (res > 15);
                        acc = res & 0x0F;
                    }
                    break;
                case 0xC:
                    if (acc == 0x00)      acc = 0;
                    else if (acc == 0x01) acc = 1;
                    else if (acc == 0x02) acc = 2;
                    else if (acc == 0x04) acc = 3;
                    else if (acc == 0x08) acc = 4;
                    else                  acc = 15;
                    break;
                case 0xD:
                    switch (acc & 0x07) {
                        case 0: cm_bank = 0x01; break;
                        case 1: cm_bank = 0x02; break;
                        case 2: cm_bank = 0x04; break;
                        case 4: cm_bank = 0x08; break;
                        default: break;
                    }
                    break;
                default:
                    break;
            }
            execute_x_phases(0, 0, 0, 0, 0, NULL);
            break;
    }
}

void setup() {
    cli();

#if defined(MCUCSR)
    uint8_t mcu_status = MCUCSR | (1 << JTD);
    asm volatile("" ::: "memory");
    MCUCSR = mcu_status;
    MCUCSR = mcu_status;
    asm volatile("" ::: "memory");
#elif defined(MCUCR)
    uint8_t mcu_status = MCUCR | (1 << JTD);
    asm volatile("" ::: "memory");
    MCUCR = mcu_status;
    MCUCR = mcu_status;
    asm volatile("" ::: "memory");
#endif

#if defined(TIMSK)
    TIMSK = 0x00;
#endif

    BUS_DDR = 0x0F; 
    CM_RAM_DDR |= 0x0F;
    CTRL_DDR |= (1<<PHI1_PIN) | (1<<PHI2_PIN) | (1<<SYNC_PIN) | 
                (1<<CM_ROM_PIN) | (1<<LATCH_A1_PIN) | (1<<LATCH_A2_PIN) | 
                (1<<ROM_A0_PIN) | (1<<LATCH_A3_PIN);

    SYS_DDR |= (1<<SRAM_LATCH_LOW_PIN) | (1<<SRAM_LATCH_HIGH_PIN) | 
               (1<<SRAM_WE_PIN) | (1<<SRAM_OE_PIN) | (1<<SRAM_A8_PIN);
    SYS_DDR &= ~((1<<RESET_PIN) | (1<<TEST_PIN));

    CTRL_PORT |= (1 << CM_ROM_PIN); 
    SYS_PORT  |= (1 << SRAM_WE_PIN) | (1 << SRAM_OE_PIN); 
    SYS_PORT  |= (1 << TEST_PIN); 
}

void loop() {
    decode_and_execute();
}
