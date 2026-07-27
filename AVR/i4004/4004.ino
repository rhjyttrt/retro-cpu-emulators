#ifndef F_CPU
#define F_CPU 16000000UL
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


static uint16_t pc = 0x000;           // 12-bit program counter
static uint16_t stack[3] = {0, 0, 0};// 3-level hardware call stack
static uint8_t  reg[16];             // 16 4-bit index registers
static uint8_t  acc = 0;             // 4-bit accumulator
static uint8_t  carry = 0;           // 1-bit carry flag
static uint8_t  cm_bank = 1;         // active cm-ram bank bitmask (1, 2, 4, 8)
static uint8_t  src_reg = 0;         // latched 8-bit ram address from src


#define CLOCK_PHI1() do { \
    CTRL_PORT |= (1 << PHI1_PIN);             /* pulse high */ \
    __builtin_avr_delay_cycles(4);            /* total high ~ 375 ns */ \
    CTRL_PORT &= ~(1 << PHI1_PIN);            /* pulse low */ \
    __builtin_avr_delay_cycles(1);            /* inter-phase gap ~ 187.5 ns */ \
} while(0)

#define CLOCK_PHI2() do { \
    CTRL_PORT |= (1 << PHI2_PIN);             /* pulse high */ \
    __builtin_avr_delay_cycles(4);            /* total high ~ 375 ns */ \
    CTRL_PORT &= ~(1 << PHI2_PIN);            /* pulse low */ \
    __builtin_avr_delay_cycles(5);            /* inter-cycle gap ~ 437.5 ns */ \
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

    // --- phase a1: output address bits 0..3 & enable eeprom (/ce = low) ---
    CTRL_PORT |= (1 << SYNC_PIN); 
    CTRL_PORT &= ~(1 << CM_ROM_PIN); 
    
    BUS_PORT = (BUS_PORT & 0xF0) | (target_addr & 0x0F);
    BUS_DDR  = (BUS_DDR & 0xF0)  | 0x0F; 
    CLOCK_PHI1(); strobe_ctrl_latch(LATCH_A1_PIN); CLOCK_PHI2();

    // --- phase a2: output address bits 4..7 & drop sync ---
    CTRL_PORT &= ~(1 << SYNC_PIN); 
    BUS_PORT = (BUS_PORT & 0xF0) | ((target_addr >> 4) & 0x0F);
    CLOCK_PHI1(); strobe_ctrl_latch(LATCH_A2_PIN); CLOCK_PHI2();

    // --- phase a3: output address bits 8..11 ---
    BUS_PORT = (BUS_PORT & 0xF0) | ((target_addr >> 8) & 0x0F);
    CLOCK_PHI1(); strobe_ctrl_latch(LATCH_A3_PIN); CLOCK_PHI2();

    // --- phase m1: read upper nibble (rom_a0 = 0) ---
    BUS_PORT &= 0xF0; BUS_DDR &= 0xF0; 
    CTRL_PORT &= ~(1 << ROM_A0_PIN);
    CLOCK_PHI1();
    asm volatile("nop; nop; nop;");
    opcode = (BUS_PIN & 0x0F) << 4; 
    CLOCK_PHI2();

    // --- phase m2: read lower nibble (rom_a0 = 1) ---
    CTRL_PORT |= (1 << ROM_A0_PIN);
    CLOCK_PHI1();
    asm volatile("nop; nop; nop;");
    opcode |= (BUS_PIN & 0x0F);
    CLOCK_PHI2();

    CTRL_PORT |= (1 << CM_ROM_PIN); // disable eeprom output (/ce = high)
    return opcode;
}

static inline void execute_x_phases(uint8_t is_ram_io, uint8_t is_ram_write, uint8_t is_status, uint8_t status_idx, uint8_t ram_data_out, uint8_t *ram_data_in) __attribute__((always_inline));
static inline void execute_x_phases(uint8_t is_ram_io, uint8_t is_ram_write, uint8_t is_status, uint8_t status_idx, uint8_t ram_data_out, uint8_t *ram_data_in) {
    // --- phase x1: bank select & sram a8 space selection ---
    if (is_ram_io) {
        CM_RAM_PORT = cm_bank; 
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

    // --- phase x2: ram address / data output ---
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

    // --- phase x3: sram transfer & data sampling ---
    if (is_ram_io) {
        if (is_ram_write) {
            SYS_PORT |= (1 << SRAM_WE_PIN); 
        } else {
            SYS_PORT &= ~(1 << SRAM_OE_PIN); 
            asm volatile("nop; nop;");
            if (ram_data_in) *ram_data_in = BUS_PIN & 0x0F;
            SYS_PORT |= (1 << SRAM_OE_PIN);  
            asm volatile("nop;");
        }
    }
    CLOCK_PULSE();

    // restore lower sram address latch back to src_reg if status operation modified it
    if (is_ram_io && is_status) {
        BUS_PORT = (BUS_PORT & 0xF0) | (src_reg & 0x0F);
        BUS_DDR  = (BUS_DDR & 0xF0)  | 0x0F;
        asm volatile("nop;");
        strobe_sys_latch(SRAM_LATCH_LOW_PIN);
    }

    CM_RAM_PORT = 0x00; 
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
        case 0x0: // nop
            execute_x_phases(0, 0, 0, 0, 0, NULL);
            break;

        case 0x1: { // jcn
            uint8_t target_low = fetch_byte_2();
            temp = 0;
            if ((opr & 0x1) && TEST_IS_LOW) temp = 1;
            if ((opr & 0x2) && carry) temp = 1;
            if ((opr & 0x4) && (acc == 0)) temp = 1;
            if (opr & 0x8) temp = !temp;
            
            if (temp) {
                pc = (pc & 0x0F00) | target_low;
            }
            break;
        }

        case 0x2: // fim / src
            if ((opr & 0x01) == 0) { // fim
                uint8_t pair = opr & 0x0E;
                uint8_t data = fetch_byte_2();
                reg[pair]     = (data >> 4) & 0x0F;
                reg[pair + 1] = data & 0x0F;
            } else { // src
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

        case 0x3: // fin / jin
            if ((opr & 0x01) == 0) { // fin
                uint16_t rom_addr = ((pc - 1) & 0x0F00) | (reg[0] << 4) | reg[1];
                execute_x_phases(0, 0, 0, 0, 0, NULL);
                res = fetch_instruction_nibbles(rom_addr);
                execute_x_phases(0, 0, 0, 0, 0, NULL);
                reg[opr & 0x0E]     = (res >> 4) & 0x0F;
                reg[(opr & 0x0E)+1] = res & 0x0F;
            } else { // jin
                uint8_t pair = opr & 0x0E;
                pc = ((pc - 1) & 0x0F00) | (reg[pair] << 4) | reg[pair + 1];
                execute_x_phases(0, 0, 0, 0, 0, NULL);
            }
            break;

        case 0x4: { // jun
            uint8_t target_low = fetch_byte_2();
            pc = ((opr << 8) | target_low) & 0x0FFF;
            break;
        }

        case 0x5: { // jms
            uint8_t target_low = fetch_byte_2();
            stack[2] = stack[1];
            stack[1] = stack[0];
            stack[0] = pc;
            pc = ((opr << 8) | target_low) & 0x0FFF;
            break;
        }

        case 0x6: // inc
            reg[opr] = (reg[opr] + 1) & 0x0F;
            execute_x_phases(0, 0, 0, 0, 0, NULL);
            break;

        case 0x7: { // isz
            uint8_t target_low = fetch_byte_2();
            reg[opr] = (reg[opr] + 1) & 0x0F;
            if (reg[opr] != 0) {
                pc = (pc & 0x0F00) | target_low;
            }
            break;
        }

        case 0x8: // add
            res = acc + reg[opr] + carry;
            carry = (res > 15) ? 1 : 0;
            acc = res & 0x0F;
            execute_x_phases(0, 0, 0, 0, 0, NULL);
            break;

        case 0x9: // sub
            res = acc + (~reg[opr] & 0x0F) + carry;
            carry = (res > 15) ? 1 : 0;
            acc = res & 0x0F;
            execute_x_phases(0, 0, 0, 0, 0, NULL);
            break;

        case 0xA: // ld
            acc = reg[opr];
            execute_x_phases(0, 0, 0, 0, 0, NULL);
            break;

        case 0xB: // xch
            temp = acc; acc = reg[opr]; reg[opr] = temp;
            execute_x_phases(0, 0, 0, 0, 0, NULL);
            break;

        case 0xC: // bbl
            acc = opr;
            pc = stack[0];
            stack[0] = stack[1];
            stack[1] = stack[2];
            execute_x_phases(0, 0, 0, 0, 0, NULL);
            break;

        case 0xD: // ldm
            acc = opr;
            execute_x_phases(0, 0, 0, 0, 0, NULL);
            break;

        case 0xE: // i/o and ram commands
            if (opr == 0x0) { // wrm
                execute_x_phases(1, 1, 0, 0, acc, NULL);
            } else if (opr == 0x1 || opr == 0x2 || opr == 0x3) { // wmp / wrr / wpm
                PORTA = (PORTA & 0x0F) | ((acc & 0x0F) << 4);
                DDRA |= 0xF0; 
                execute_x_phases(0, 0, 0, 0, 0, NULL);
            } else if (opr == 0x4) { // rdm
                execute_x_phases(1, 0, 0, 0, 0, &ram_read_val);
                acc = ram_read_val;
            } else if (opr == 0x5) { // rdr (hardware synchronized)
                PORTA &= 0x0F; 
                DDRA  &= 0x0F; 
                CLOCK_PULSE(); // x1 phase pulse
                CLOCK_PULSE(); // x2 phase pulse
                asm volatile("nop; nop;"); 
                acc = (PINA >> 4) & 0x0F; // sample during x2/x3 after clocking
                CLOCK_PULSE(); // x3 phase pulse
                BUS_PORT &= 0xF0; BUS_DDR &= 0xF0;
            } else if (opr == 0x6) { // adm
                execute_x_phases(1, 0, 0, 0, 0, &ram_read_val);
                res = acc + ram_read_val + carry;
                carry = (res > 15) ? 1 : 0;
                acc = res & 0x0F;
            } else if (opr == 0x7) { // sbm
                execute_x_phases(1, 0, 0, 0, 0, &ram_read_val);
                res = acc + (~ram_read_val & 0x0F) + carry;
                carry = (res > 15) ? 1 : 0;
                acc = res & 0x0F;
            } else if ((opr & 0x0C) == 0x08) { // wr0..wr3
                execute_x_phases(1, 1, 1, opr & 0x03, acc, NULL);
            } else if ((opr & 0x0C) == 0x0C) { // rd0..rd3
                execute_x_phases(1, 0, 1, opr & 0x03, 0, &ram_read_val);
                acc = ram_read_val;
            } else {
                execute_x_phases(0, 0, 0, 0, 0, NULL);
            }
            break;

        case 0xF: // alu accumulator group
            switch (opr) {
                case 0x0: acc = 0; carry = 0; break; // clb
                case 0x1: carry = 0; break;          // clc
                case 0x2: res = acc + 1; carry = (res > 15) ? 1 : 0; acc = res & 0x0F; break; // iac
                case 0x3: carry = !carry; break;     // cmc
                case 0x4: acc = ~acc & 0x0F; break;  // cma
                case 0x5: temp = carry; carry = (acc & 0x08) ? 1 : 0; acc = ((acc << 1) | temp) & 0x0F; break; // ral
                case 0x6: temp = carry; carry = (acc & 0x01) ? 1 : 0; acc = ((acc >> 1) | (temp << 3)) & 0x0F; break; // rar
                case 0x7: acc = carry; carry = 0; break; // tcc
                case 0x8: res = acc + 0x0F; carry = (res > 15) ? 1 : 0; acc = res & 0x0F; break; // dac
                case 0x9: acc = carry ? 10 : 9; carry = 0; break; // tcs
                case 0xA: carry = 1; break;          // stc
                case 0xB:                            // daa
                    if (carry || acc > 9) {
                        res = acc + 6;
                        if (res > 15) carry = 1;
                        acc = res & 0x0F;
                    }
                    break;
                case 0xC:                            // kbp
                    if (acc == 0x00)      acc = 0;
                    else if (acc == 0x01) acc = 1;
                    else if (acc == 0x02) acc = 2;
                    else if (acc == 0x04) acc = 3;
                    else if (acc == 0x08) acc = 4;
                    else                  acc = 15;
                    break;
                case 0xD:                            // dcl
                    switch (acc & 0x07) {
                        case 0: cm_bank = 0x01; break;
                        case 1: cm_bank = 0x02; break;
                        case 2: cm_bank = 0x04; break;
                        case 4: cm_bank = 0x08; break;
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
    cli(); // disable interrupts immediately

    // 1. disable jtag immediately (guaranteed 2-cycle write)
    uint8_t mcu_status = MCUCSR | (1 << JTD);
    MCUCSR = mcu_status;
    MCUCSR = mcu_status;

    // 2. disable timer interrupts
    TIMSK = 0x00;

    // 3. configure port directions
    BUS_DDR = 0x0F; 
    CM_RAM_DDR |= 0x0F;
    CTRL_DDR |= (1<<PHI1_PIN) | (1<<PHI2_PIN) | (1<<SYNC_PIN) | 
                (1<<CM_ROM_PIN) | (1<<LATCH_A1_PIN) | (1<<LATCH_A2_PIN) | 
                (1<<ROM_A0_PIN) | (1<<LATCH_A3_PIN);

    SYS_DDR |= (1<<SRAM_LATCH_LOW_PIN) | (1<<SRAM_LATCH_HIGH_PIN) | 
               (1<<SRAM_WE_PIN) | (1<<SRAM_OE_PIN) | (1<<SRAM_A8_PIN);
    SYS_DDR &= ~((1<<RESET_PIN) | (1<<TEST_PIN));

    // 4. default line states
    CTRL_PORT |= (1 << CM_ROM_PIN); 
    SYS_PORT  |= (1 << SRAM_WE_PIN) | (1 << SRAM_OE_PIN); 
    SYS_PORT  |= (1 << TEST_PIN); 
}

void loop() {
    decode_and_execute();
}
