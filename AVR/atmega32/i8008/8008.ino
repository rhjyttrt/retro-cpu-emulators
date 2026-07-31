#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

#pragma GCC optimize ("O3")

#define STATE_TW   0x00
#define STATE_TH   0x01
#define STATE_T1   0x02
#define STATE_T1I  0x03
#define STATE_T2   0x04
#define STATE_T5   0x05
#define STATE_T3   0x06
#define STATE_T4   0x07

#define SYNC_HIGH  0x08
#define SYNC_LOW   0x00

#define CC_PCI     0x00
#define CC_PCC     0x40
#define CC_PCR     0x80
#define CC_PCW     0xC0

register uint8_t regA asm("r2");
register uint8_t regB asm("r3");
register uint8_t regC asm("r4");
register uint8_t regD asm("r5");
register uint8_t regE asm("r6");
register uint8_t regH asm("r7");
register uint8_t regL asm("r8");

uint8_t flag_C = 0;
uint8_t flag_Z = 0;
uint8_t flag_S = 0;
uint8_t flag_P = 0;

uint16_t address_stack[8];
uint8_t  stack_ptr = 0;

#define PC (address_stack[stack_ptr])

__attribute__((always_inline)) inline void push_stack(uint16_t target_addr, uint16_t return_addr) {
    address_stack[stack_ptr] = return_addr & 0x3FFF;
    stack_ptr = (stack_ptr + 1) & 0x07;
    PC = target_addr & 0x3FFF;
}

__attribute__((always_inline)) inline void pop_stack(void) {
    address_stack[stack_ptr] = 0x0000;
    stack_ptr = (stack_ptr == 0) ? 7 : (stack_ptr - 1);
}

__attribute__((always_inline)) inline uint8_t get_parity(uint8_t val) {
    val ^= val >> 4;
    val ^= val >> 2;
    val ^= val >> 1;
    return (val & 1) ? 0 : 1;
}

__attribute__((always_inline)) inline void update_ZSP(uint8_t res) {
    flag_Z = (res == 0) ? 1 : 0;
    flag_S = (res & 0x80) ? 1 : 0;
    flag_P = get_parity(res);
}

void setup_hardware_clocks(void) {
    DDRD |= (1 << PD4) | (1 << PD5);

    TCCR1A = (1 << COM1A1) | (0 << COM1A0) |
             (1 << COM1B1) | (1 << COM1B0) |
             (1 << WGM11)  | (0 << WGM10);

    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);

    ICR1  = 19;
    OCR1A = 6;
    OCR1B = 12;
}

__attribute__((always_inline)) inline void sync_to_phi2_fall(void) {
    while (TCNT1 >= 18);
    while (TCNT1 < 18);
}

__attribute__((always_inline)) inline void bus_T1(uint16_t addr, uint8_t state_code) {
    sync_to_phi2_fall();
    PORTA = (uint8_t)(addr & 0xFF);
    DDRA  = 0xFF;
    PORTB = state_code | SYNC_HIGH;
    asm volatile("nop\n nop\n nop\n nop\n");
    PORTB = state_code | SYNC_LOW;
}

__attribute__((always_inline)) inline void bus_T2(uint8_t high_byte, uint8_t cc_bits) {
    sync_to_phi2_fall();
    PORTA = (high_byte & 0x3F) | cc_bits;
    PORTB = STATE_T2 | SYNC_HIGH;

    asm volatile("nop\n nop\n nop\n nop\n");

    if (!(PIND & (1 << PD2))) {
        while (!(PIND & (1 << PD2))) {
            sync_to_phi2_fall();
            PORTB = STATE_TW | SYNC_HIGH;
            asm volatile("nop\n nop\n nop\n nop\n");
            PORTB = STATE_TW | SYNC_LOW;
        }
    } else {
        PORTB = STATE_T2 | SYNC_LOW;
    }
}

__attribute__((always_inline)) inline uint8_t bus_T3_read(void) {
    DDRA  = 0x00;
    PORTA = 0x00;
    sync_to_phi2_fall();
    PORTB = STATE_T3 | SYNC_HIGH;

    asm volatile("nop\n nop\n nop\n nop\n nop\n nop\n");
    uint8_t val = PINA;
    PORTB = STATE_T3 | SYNC_LOW;
    return val;
}

__attribute__((always_inline)) inline void bus_T3_write(uint8_t val) {
    sync_to_phi2_fall();
    DDRA  = 0xFF;
    PORTA = val;
    PORTB = STATE_T3 | SYNC_HIGH;
    asm volatile("nop\n nop\n nop\n nop\n");
    PORTB = STATE_T3 | SYNC_LOW;
    asm volatile("nop\n nop\n");
    PORTA = 0x00;
    DDRA  = 0x00;
}

__attribute__((always_inline)) inline void bus_T4(void) {
    sync_to_phi2_fall();
    PORTB = STATE_T4 | SYNC_HIGH;
    asm volatile("nop\n nop\n nop\n nop\n");
    PORTB = STATE_T4 | SYNC_LOW;
}

__attribute__((always_inline)) inline void bus_T5(void) {
    sync_to_phi2_fall();
    PORTB = STATE_T5 | SYNC_HIGH;
    asm volatile("nop\n nop\n nop\n nop\n");
    PORTB = STATE_T5 | SYNC_LOW;
}

__attribute__((always_inline)) inline uint8_t fetch_memory_byte(uint16_t addr, uint8_t cc_type, uint8_t t1_code) {
    bus_T1(addr, t1_code);
    bus_T2((uint8_t)(addr >> 8), cc_type);
    return bus_T3_read();
}

__attribute__((always_inline)) inline void write_memory_byte(uint16_t addr, uint8_t val) {
    bus_T1(addr, STATE_T1);
    bus_T2((uint8_t)(addr >> 8), CC_PCW);
    bus_T3_write(val);
}

__attribute__((always_inline)) inline uint8_t get_reg_val(uint8_t index) {
    switch (index & 0x07) {
        case 0: return regA;
        case 1: return regB;
        case 2: return regC;
        case 3: return regD;
        case 4: return regE;
        case 5: return regH;
        case 6: return regL;
        case 7: { 
            uint16_t hl = ((((uint16_t)regH & 0x3F) << 8) | regL) & 0x3FFF;
            uint8_t val = fetch_memory_byte(hl, CC_PCR, STATE_T1);
            bus_T4();
            bus_T5();
            return val;
        }
    }
    return 0;
}

__attribute__((always_inline)) inline void set_reg_val(uint8_t index, uint8_t val) {
    switch (index & 0x07) {
        case 0: regA = val; break;
        case 1: regB = val; break;
        case 2: regC = val; break;
        case 3: regD = val; break;
        case 4: regE = val; break;
        case 5: regH = val; break;
        case 6: regL = val; break;
        case 7: { 
            uint16_t hl = ((((uint16_t)regH & 0x3F) << 8) | regL) & 0x3FFF;
            write_memory_byte(hl, val);
            bus_T4();
            bus_T5();
            break;
        }
    }
}

__attribute__((always_inline)) inline void execute_ALU(uint8_t op, uint8_t operand) {
    switch (op & 0x07) {
        case 0: {
            uint16_t res = (uint16_t)regA + (uint16_t)operand;
            flag_C = (res > 0xFF) ? 1 : 0;
            regA = res & 0xFF;
            update_ZSP(regA);
            break;
        }
        case 1: {
            uint16_t res = (uint16_t)regA + (uint16_t)operand + flag_C;
            flag_C = (res > 0xFF) ? 1 : 0;
            regA = res & 0xFF;
            update_ZSP(regA);
            break;
        }
        case 2: {
            flag_C = (regA < operand) ? 1 : 0;
            regA = (regA - operand) & 0xFF;
            update_ZSP(regA);
            break;
        }
        case 3: {
            uint16_t temp_a = regA;
            uint16_t temp_sub = (uint16_t)operand + flag_C;
            flag_C = (temp_a < temp_sub) ? 1 : 0;
            regA = (temp_a - temp_sub) & 0xFF;
            update_ZSP(regA);
            break;
        }
        case 4: {
            regA &= operand;
            flag_C = 0;
            update_ZSP(regA);
            break;
        }
        case 5: {
            regA ^= operand;
            flag_C = 0;
            update_ZSP(regA);
            break;
        }
        case 6: {
            regA |= operand;
            flag_C = 0;
            update_ZSP(regA);
            break;
        }
        case 7: {
            flag_C = (regA < operand) ? 1 : 0;
            uint8_t temp_sub = (regA - operand) & 0xFF;
            update_ZSP(temp_sub);
            break;
        }
    }
}

__attribute__((always_inline)) inline uint8_t check_condition(uint8_t opcode) {
    uint8_t cond = (opcode >> 3) & 0x07;
    switch (cond) {
        case 0: return (flag_C == 0);
        case 4: return (flag_C == 1);
        case 1: return (flag_Z == 0);
        case 5: return (flag_Z == 1);
        case 2: return (flag_S == 0);
        case 6: return (flag_S == 1);
        case 3: return (flag_P == 0);
        case 7: return (flag_P == 1);
    }
    return 0;
}

void run_emulator_cycle(void) {
    uint8_t is_interrupt = 0;
    uint8_t t1_state_code = STATE_T1;

    if (PIND & (1 << PD3)) {
        is_interrupt = 1;
        t1_state_code = STATE_T1I; 
    }

    uint16_t saved_pc = PC;
    uint8_t opcode = fetch_memory_byte(PC, CC_PCI, t1_state_code);
    bus_T4();
    bus_T5();

    if (!is_interrupt) {
        PC = (PC + 1) & 0x3FFF;
    }

    if (opcode == 0x00 || opcode == 0x01 || opcode == 0x38 || 
        opcode == 0x39 || opcode == 0x3F || opcode == 0xFF) {
        while (!(PIND & (1 << PD3))) {
            sync_to_phi2_fall();
            PORTB = STATE_TH | SYNC_HIGH;
            asm volatile("nop\n nop\n nop\n nop\n");
            PORTB = STATE_TH | SYNC_LOW;
        }
        return;
    }

    if ((opcode & 0xC0) == 0xC0) {
        uint8_t dest = (opcode >> 3) & 0x07;
        uint8_t src  = opcode & 0x07;
        if (dest != src) { 
            set_reg_val(dest, get_reg_val(src));
        }
        return;
    }

    if ((opcode & 0xC0) == 0x80) {
        uint8_t op  = (opcode >> 3) & 0x07;
        uint8_t src = opcode & 0x07;
        execute_ALU(op, get_reg_val(src));
        return;
    }

    if ((opcode & 0xC0) == 0x00) {
        uint8_t sub_op = opcode & 0x07;

        if (sub_op == 0x00 || sub_op == 0x01) {
            uint8_t reg_idx = (opcode >> 3) & 0x07;
            if (reg_idx == 0) return;

            uint8_t reg_val = get_reg_val(reg_idx);
            uint8_t val = (sub_op == 0x00) ? (reg_val + 1) : (reg_val - 1);

            set_reg_val(reg_idx, val);
            update_ZSP(val);
            return;
        }

        if (sub_op == 0x02) {
            uint8_t rot_type = (opcode >> 3) & 0x07;
            if (rot_type < 4) {
                uint8_t old_carry = flag_C ? 1 : 0;

                if (rot_type == 0) {
                    flag_C = (regA & 0x80) ? 1 : 0;
                    regA = ((regA << 1) | flag_C) & 0xFF;
                } else if (rot_type == 1) {
                    flag_C = (regA & 0x01) ? 1 : 0;
                    regA = (regA >> 1) | (flag_C << 7);
                } else if (rot_type == 2) {
                    uint8_t bit7 = (regA & 0x80) ? 1 : 0;
                    regA = ((regA << 1) | old_carry) & 0xFF;
                    flag_C = bit7;
                } else if (rot_type == 3) {
                    uint8_t bit0 = regA & 0x01;
                    regA = (regA >> 1) | (old_carry << 7);
                    flag_C = bit0;
                }
            }
            return;
        }

        if (sub_op == 0x03) {
            if (check_condition(opcode)) {
                pop_stack();
            }
            return;
        }

        if (sub_op == 0x04) {
            uint8_t imm_val = fetch_memory_byte(is_interrupt ? saved_pc : PC, CC_PCR, is_interrupt ? STATE_T1I : STATE_T1);
            if (is_interrupt) {
                PC = saved_pc;
            } else {
                PC = (PC + 1) & 0x3FFF;
            }

            uint8_t op = (opcode >> 3) & 0x07;
            execute_ALU(op, imm_val);
            return;
        }

        if (sub_op == 0x05) {
            uint16_t target = (opcode & 0x38); 
            push_stack(target, is_interrupt ? saved_pc : PC);
            return;
        }

        if (sub_op == 0x06) {
            uint8_t imm_val = fetch_memory_byte(is_interrupt ? saved_pc : PC, CC_PCR, is_interrupt ? STATE_T1I : STATE_T1);
            uint8_t dest = (opcode >> 3) & 0x07;
            set_reg_val(dest, imm_val);

            if (is_interrupt) {
                PC = saved_pc;
            } else {
                PC = (PC + 1) & 0x3FFF;
            }
            return;
        }

        if (sub_op == 0x07) {
            pop_stack();
            return;
        }
    }

    if ((opcode & 0xC0) == 0x40) {
        if ((opcode & 0x07) == 0x00) {
            uint8_t t1_type = is_interrupt ? STATE_T1I : STATE_T1;
            uint16_t operand_pc = is_interrupt ? saved_pc : PC;
            uint8_t low_addr  = fetch_memory_byte(operand_pc, CC_PCR, t1_type);
            uint8_t high_addr = fetch_memory_byte((operand_pc + 1) & 0x3FFF, CC_PCR, t1_type);
            uint16_t target   = (((uint16_t)high_addr & 0x3F) << 8) | low_addr;

            if (check_condition(opcode)) {
                PC = target;
            } else {
                PC = is_interrupt ? saved_pc : ((operand_pc + 2) & 0x3FFF);
            }
            return;
        }

        if ((opcode & 0x07) == 0x02) {
            uint8_t t1_type = is_interrupt ? STATE_T1I : STATE_T1;
            uint16_t operand_pc = is_interrupt ? saved_pc : PC;
            uint8_t low_addr  = fetch_memory_byte(operand_pc, CC_PCR, t1_type);
            uint8_t high_addr = fetch_memory_byte((operand_pc + 1) & 0x3FFF, CC_PCR, t1_type);
            uint16_t target   = (((uint16_t)high_addr & 0x3F) << 8) | low_addr;
            uint16_t return_pc = (operand_pc + 2) & 0x3FFF;

            if (check_condition(opcode)) {
                push_stack(target, return_pc);
                if (is_interrupt) {
                    address_stack[(stack_ptr == 0) ? 7 : (stack_ptr - 1)] = saved_pc;
                }
            } else {
                PC = is_interrupt ? saved_pc : return_pc;
            }
            return;
        }

        if ((opcode & 0x07) == 0x04) {
            uint8_t t1_type = is_interrupt ? STATE_T1I : STATE_T1;
            uint16_t operand_pc = is_interrupt ? saved_pc : PC;
            uint8_t low_addr  = fetch_memory_byte(operand_pc, CC_PCR, t1_type);
            uint8_t high_addr = fetch_memory_byte((operand_pc + 1) & 0x3FFF, CC_PCR, t1_type);
            PC = (((uint16_t)high_addr & 0x3F) << 8) | low_addr;
            return;
        }

        if ((opcode & 0x07) == 0x06) {
            uint8_t t1_type = is_interrupt ? STATE_T1I : STATE_T1;
            uint16_t operand_pc = is_interrupt ? saved_pc : PC;
            uint8_t low_addr  = fetch_memory_byte(operand_pc, CC_PCR, t1_type);
            uint8_t high_addr = fetch_memory_byte((operand_pc + 1) & 0x3FFF, CC_PCR, t1_type);
            uint16_t target   = (((uint16_t)high_addr & 0x3F) << 8) | low_addr;
            uint16_t return_pc = (operand_pc + 2) & 0x3FFF;

            push_stack(target, return_pc);
            if (is_interrupt) {
                address_stack[(stack_ptr == 0) ? 7 : (stack_ptr - 1)] = saved_pc;
            }
            return;
        }

        if ((opcode & 0x01) == 0x01) {
            uint8_t port_num = (opcode >> 1) & 0x1F;

            bus_T1(regA, STATE_T1);
            bus_T2(port_num, CC_PCC);

            if ((opcode & 0x30) == 0x00) {
                regA = bus_T3_read();
            } else {
                bus_T3_write(regA);
            }
            bus_T4();
            bus_T5();
            return;
        }
    }
}

int main(void) {
    DDRB |= (1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB3); 
    DDRD &= ~((1 << PD2) | (1 << PD3));                        
    PORTD |= (1 << PD2);                                       

    regA = regB = regC = regD = regE = regH = regL = 0;
    flag_C = flag_Z = flag_S = flag_P = 0;
    
    for (uint8_t i = 0; i < 8; i++) {
        address_stack[i] = 0x0000;
    }
    stack_ptr = 0;

    setup_hardware_clocks();
    cli();
    TCNT1 = 0;

    while (1) {
        run_emulator_cycle();
    }

    return 0;
}
