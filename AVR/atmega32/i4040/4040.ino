#include <Arduino.h>
#include <avr/io.h>
#include <stdint.h>
#include <stdbool.h>

#ifndef F_CPU
#define F_CPU 20000000UL
#endif

#define INT_PIN   PD0
#define PHI1_PIN  PD2
#define PHI2_PIN  PD3
#define TEST_PIN  PD4
#define SYNC_PIN  PA4

typedef struct {
    uint8_t  acc;
    uint8_t  carry;
    uint16_t pc;
    uint8_t  index_reg[24];
    uint8_t  reg_bank;
    uint8_t  saved_reg_bank;
    uint16_t stack[8];
    uint8_t  sp;
    uint8_t  rom_bank;
    uint8_t  ram_bank;
    
    bool     in_second_cycle;
    uint8_t  first_opr;
    uint8_t  first_opa;
    uint16_t first_page;
    bool     interrupt_enable;
    bool     in_interrupt;
    bool     halted;
} i4040_hardware_t;

static i4040_hardware_t cpu;

static inline void clock_phase_out_cm(uint8_t bus_nibble, bool sync, bool assert_cm_ram, bool assert_cm_rom) {
    uint8_t cm_ram_mask = assert_cm_ram ? (1 << cpu.ram_bank) : 0;
    uint8_t cm_rom_mask = assert_cm_rom ? ((cpu.rom_bank == 0) ? (1 << PA5) : (1 << PA6)) : 0;

    uint8_t pa_out    = (PORTA & 0x80) | (bus_nibble & 0x0F) | cm_rom_mask;
    if (sync) pa_out |= (1 << SYNC_PIN);

    uint8_t ddra_out  = DDRA | 0x0F;
    uint8_t portc_on  = PORTC | cm_ram_mask;
    uint8_t portc_off = PORTC & ~cm_ram_mask;
    uint8_t pa_off    = pa_out & ~((1 << PA5) | (1 << PA6));

    PORTD |= (1 << PHI1_PIN);
    asm volatile("nop\n nop\n nop\n nop\n nop\n nop\n");

    PORTD &= ~(1 << PHI1_PIN);
    DDRA = ddra_out;
    PORTA = pa_out;
    asm volatile("nop\n");

    PORTD |= (1 << PHI2_PIN);
    PORTC = portc_on;
    asm volatile("nop\n nop\n nop\n nop\n nop\n");

    PORTD &= ~(1 << PHI2_PIN);
    PORTC = portc_off;
    PORTA = pa_off;
    asm volatile("nop\n nop\n");
}

static inline uint8_t clock_phase_in_cm(bool assert_cm_ram, bool assert_cm_rom) {
    uint8_t cm_ram_mask = assert_cm_ram ? (1 << cpu.ram_bank) : 0;
    uint8_t cm_rom_mask = assert_cm_rom ? ((cpu.rom_bank == 0) ? (1 << PA5) : (1 << PA6)) : 0;

    uint8_t ddra_in   = DDRA & ~0x0F;
    uint8_t portc_on  = PORTC | cm_ram_mask;
    uint8_t portc_off = PORTC & ~cm_ram_mask;
    uint8_t pa_in     = (PORTA & 0x80) | cm_rom_mask;
    uint8_t pa_off    = pa_in & ~((1 << PA5) | (1 << PA6));
    uint8_t sample;

    PORTD |= (1 << PHI1_PIN);
    asm volatile("nop\n nop\n nop\n nop\n nop\n nop\n");

    PORTD &= ~(1 << PHI1_PIN);
    DDRA = ddra_in;
    PORTA = pa_in;
    asm volatile("nop\n");

    PORTD |= (1 << PHI2_PIN);
    PORTC = portc_on;
    asm volatile("nop\n");
    sample = PINA & 0x0F;
    asm volatile("nop\n nop\n nop\n");

    PORTD &= ~(1 << PHI2_PIN);
    PORTC = portc_off;
    PORTA = pa_off;
    asm volatile("nop\n nop\n");

    return sample;
}

static inline uint8_t get_reg_idx(uint8_t reg) {
    reg &= 0x0F;
    if (cpu.reg_bank == 1 && reg < 8) return 16 + reg;
    return reg;
}

static inline bool get_test_pin(void) {
    return (PIND & (1 << TEST_PIN)) ? true : false;
}

void i4040_hardware_step(void) {
    if (cpu.interrupt_enable && !cpu.in_interrupt && !cpu.in_second_cycle && !(PIND & (1 << INT_PIN))) {
        cpu.halted = false;
        cpu.in_interrupt = true;
        cpu.interrupt_enable = false;
        
        cpu.saved_reg_bank = cpu.reg_bank;
        cpu.reg_bank = 1;

        cpu.stack[cpu.sp & 0x07] = cpu.pc & 0x0FFF;
        cpu.sp = (cpu.sp + 1) & 0x07;
        cpu.pc = 0x003;
    }

    if (cpu.halted) {
        for (uint8_t i = 0; i < 8; i++) {
            clock_phase_out_cm(0x0, (i == 0), false, false);
        }
        return;
    }

    uint16_t bus_addr = cpu.pc;
    
    if (cpu.in_second_cycle) {
        uint8_t r0 = cpu.index_reg[get_reg_idx(0)];
        uint8_t r1 = cpu.index_reg[get_reg_idx(1)];
        if (cpu.first_opr == 0x3) {
            bus_addr = ((r0 << 4) | r1) & 0x0FFF;
        } else if (cpu.first_opr == 0x0 && cpu.first_opa == 0x0E) {
            bus_addr = (cpu.first_page | (r0 << 4) | r1) & 0x0FFF;
        }
    }

    clock_phase_out_cm((bus_addr) & 0x0F, true, false, false);
    clock_phase_out_cm((bus_addr >> 4) & 0x0F, false, false, false);
    clock_phase_out_cm((bus_addr >> 8) & 0x0F, false, false, false);

    uint8_t opr = clock_phase_in_cm(false, true);
    uint8_t opa = clock_phase_in_cm(false, true);

    // Send OPA command modifier during X1 only on cycle 1 for RAM/IO group (0xE)
    uint8_t x1_nibble = (!cpu.in_second_cycle && opr == 0xE) ? opa : 0x0;
    clock_phase_out_cm(x1_nibble, false, false, false);

    if (cpu.in_second_cycle) {
        uint8_t prev_opr = cpu.first_opr;
        uint8_t prev_opa = cpu.first_opa;
        uint8_t data_byte = (opr << 4) | opa;

        switch (prev_opr) {
            case 0x0:
                if (prev_opa == 0x0E) {
                    cpu.acc = opa;
                }
                break;

            case 0x1: {
                bool cond = false;
                if (prev_opa & 0x01) cond |= (get_test_pin() == false);
                if (prev_opa & 0x02) cond |= (cpu.carry == 1);
                if (prev_opa & 0x04) cond |= (cpu.acc == 0);
                if (prev_opa & 0x08) cond = !cond;

                if (cond) cpu.pc = (cpu.first_page | data_byte) & 0x0FFF;
                else      cpu.pc = (cpu.pc + 1) & 0x0FFF;
                break;
            }
            case 0x2:
                if ((prev_opa & 0x01) == 0) {
                    uint8_t pair = prev_opa & 0x0E;
                    cpu.index_reg[get_reg_idx(pair)]     = (data_byte >> 4) & 0x0F;
                    cpu.index_reg[get_reg_idx(pair + 1)] = data_byte & 0x0F;
                    cpu.pc = (cpu.pc + 1) & 0x0FFF;
                }
                break;

            case 0x3:
                if ((prev_opa & 0x01) == 0) {
                    uint8_t pair = prev_opa & 0x0E;
                    cpu.index_reg[get_reg_idx(pair)]     = opr;
                    cpu.index_reg[get_reg_idx(pair + 1)] = opa;
                }
                break;

            case 0x4:
                cpu.pc = (((uint16_t)(prev_opa & 0x0F) << 8) | data_byte) & 0x0FFF;
                break;

            case 0x5:
                cpu.stack[cpu.sp & 0x07] = (cpu.pc + 1) & 0x0FFF;
                cpu.sp = (cpu.sp + 1) & 0x07;
                cpu.pc = (((uint16_t)(prev_opa & 0x0F) << 8) | data_byte) & 0x0FFF;
                break;

            case 0x7: {
                uint8_t idx = get_reg_idx(prev_opa);
                cpu.index_reg[idx] = (cpu.index_reg[idx] + 1) & 0x0F;
                if (cpu.index_reg[idx] != 0) {
                    cpu.pc = (cpu.first_page | data_byte) & 0x0FFF;
                } else {
                    cpu.pc = (cpu.pc + 1) & 0x0FFF;
                }
                break;
            }
        }
        clock_phase_out_cm(0x0, false, false, false);
        clock_phase_out_cm(0x0, false, false, false);
        cpu.in_second_cycle = false;
    } else {
        bool is_two_cycle = false;

        switch (opr) {
            case 0x0:
                switch (opa) {
                    case 0x0: break;
                    case 0x1: cpu.halted = true; break;
                    case 0x2: {
                        cpu.sp = (cpu.sp - 1) & 0x07;
                        cpu.pc = cpu.stack[cpu.sp] & 0x0FFF;
                        cpu.reg_bank = cpu.saved_reg_bank;
                        cpu.in_interrupt = false;
                        cpu.interrupt_enable = true;
                        clock_phase_out_cm(0x0, false, false, false);
                        clock_phase_out_cm(0x0, false, false, false);
                        return;
                    }
                    case 0x3: cpu.acc = cpu.ram_bank; break;
                    case 0x4: cpu.acc |= cpu.index_reg[get_reg_idx(4)]; break;
                    case 0x5: cpu.acc |= cpu.index_reg[get_reg_idx(5)]; break;
                    case 0x6: cpu.acc &= cpu.index_reg[get_reg_idx(6)]; break;
                    case 0x7: cpu.acc &= cpu.index_reg[get_reg_idx(7)]; break;
                    case 0x8: cpu.rom_bank = 0; break;
                    case 0x9: cpu.rom_bank = 1; break;
                    case 0x0A: cpu.reg_bank = 0; break;
                    case 0x0B: cpu.reg_bank = 1; break;
                    case 0x0C: cpu.interrupt_enable = true; break;  // EIN
                    case 0x0D: cpu.interrupt_enable = false; break; // DIN
                    case 0x0E: is_two_cycle = true; break;
                }
                break;

            case 0x1: is_two_cycle = true; break;

            case 0x2:
                if (opa & 0x01) {
                    uint8_t pair = opa & 0x0E;
                    uint8_t low_nib  = cpu.index_reg[get_reg_idx(pair + 1)];
                    uint8_t high_nib = cpu.index_reg[get_reg_idx(pair)];
                    
                    clock_phase_out_cm(high_nib, false, true, false);
                    clock_phase_out_cm(low_nib, false, true, false);
                    cpu.pc = (cpu.pc + 1) & 0x0FFF;
                    return;
                } else {
                    is_two_cycle = true;
                }
                break;

            case 0x3:
                if (opa & 0x01) {
                    uint8_t pair = opa & 0x0E;
                    uint16_t addr = ((uint16_t)cpu.index_reg[get_reg_idx(pair)] << 4) | 
                                    cpu.index_reg[get_reg_idx(pair + 1)];
                    cpu.pc = ((cpu.pc & 0xF00) | addr) & 0x0FFF;
                    clock_phase_out_cm(0x0, false, false, false);
                    clock_phase_out_cm(0x0, false, false, false);
                    return;
                } else {
                    is_two_cycle = true;
                }
                break;

            case 0x4: is_two_cycle = true; break;
            case 0x5: is_two_cycle = true; break;

            case 0x6: {
                uint8_t idx = get_reg_idx(opa);
                cpu.index_reg[idx] = (cpu.index_reg[idx] + 1) & 0x0F;
                break;
            }

            case 0x7: is_two_cycle = true; break;

            case 0x8: {
                uint8_t val = cpu.index_reg[get_reg_idx(opa)];
                uint16_t sum = cpu.acc + val + cpu.carry;
                cpu.carry = (sum > 15) ? 1 : 0;
                cpu.acc = sum & 0x0F;
                break;
            }

            case 0x9: {
                uint8_t val = cpu.index_reg[get_reg_idx(opa)];
                uint16_t sum = cpu.acc + (~val & 0x0F) + cpu.carry;
                cpu.carry = (sum > 15) ? 1 : 0;
                cpu.acc = sum & 0x0F;
                break;
            }

            case 0xA: cpu.acc = cpu.index_reg[get_reg_idx(opa)]; break;

            case 0xB: {
                uint8_t idx = get_reg_idx(opa);
                uint8_t tmp = cpu.acc;
                cpu.acc = cpu.index_reg[idx];
                cpu.index_reg[idx] = tmp;
                break;
            }

            case 0xC: {
                cpu.sp = (cpu.sp - 1) & 0x07;
                cpu.pc = cpu.stack[cpu.sp] & 0x0FFF;
                cpu.acc = opa & 0x0F;
                clock_phase_out_cm(0x0, false, false, false);
                clock_phase_out_cm(0x0, false, false, false);
                return;
            }

            case 0xD: cpu.acc = opa & 0x0F; break;

            case 0xE:
                if (opa <= 0x7) {
                    bool is_rom_io = (opa == 0x02);
                    clock_phase_out_cm(cpu.acc, false, !is_rom_io, is_rom_io);
                    clock_phase_out_cm(cpu.acc, false, false, false);
                } else {
                    bool is_rom_io = (opa == 0x0A);
                    uint8_t rval = clock_phase_in_cm(!is_rom_io, is_rom_io);
                    
                    if (opa == 0x08) {
                        uint16_t sum = cpu.acc + (~rval & 0x0F) + cpu.carry;
                        cpu.carry = (sum > 15) ? 1 : 0;
                        cpu.acc = sum & 0x0F;
                    } else if (opa == 0x0B) {
                        uint16_t sum = cpu.acc + rval + cpu.carry;
                        cpu.carry = (sum > 15) ? 1 : 0;
                        cpu.acc = sum & 0x0F;
                    } else {
                        cpu.acc = rval;
                    }
                    clock_phase_in_cm(false, false);
                }
                cpu.pc = (cpu.pc + 1) & 0x0FFF;
                return;

            case 0xF:
                switch (opa) {
                    case 0x0: cpu.acc = 0; cpu.carry = 0; break;
                    case 0x1: cpu.carry = 0; break;
                    case 0x2: cpu.acc++; cpu.carry = (cpu.acc > 15) ? 1 : 0; cpu.acc &= 0x0F; break;
                    case 0x3: cpu.carry ^= 1; break;
                    case 0x4: cpu.acc = (~cpu.acc) & 0x0F; break;
                    case 0x5: {
                        uint8_t new_c = (cpu.acc & 0x08) ? 1 : 0;
                        cpu.acc = ((cpu.acc << 1) | cpu.carry) & 0x0F;
                        cpu.carry = new_c;
                        break;
                    }
                    case 0x6: {
                        uint8_t new_c = cpu.acc & 0x01;
                        cpu.acc = ((cpu.acc >> 1) | (cpu.carry << 3)) & 0x0F;
                        cpu.carry = new_c;
                        break;
                    }
                    case 0x7:
                        cpu.acc = cpu.carry & 0x01;
                        cpu.carry = 0;
                        break;
                    case 0x8:
                        if (cpu.acc == 0) { cpu.acc = 15; cpu.carry = 0; }
                        else { cpu.acc--; cpu.carry = 1; }
                        break;
                    case 0x9: cpu.acc = (cpu.carry == 0) ? 9 : 10; cpu.carry = 0; break;
                    case 0x0A: cpu.carry = 1; break;
                    case 0x0B: {
                        if (cpu.acc > 9 || cpu.carry) {
                            uint8_t sum = cpu.acc + 6;
                            if (sum > 15) cpu.carry = 1;
                            cpu.acc = sum & 0x0F;
                        }
                        break;
                    }
                    case 0x0C:
                        if (cpu.acc == 1) cpu.acc = 1;
                        else if (cpu.acc == 2) cpu.acc = 2;
                        else if (cpu.acc == 4) cpu.acc = 3;
                        else if (cpu.acc == 8) cpu.acc = 4;
                        else if (cpu.acc == 0) cpu.acc = 0;
                        else cpu.acc = 15;
                        break;
                    case 0x0D: cpu.ram_bank = cpu.acc & 0x07; break;
                    case 0x0F:
                        cpu.acc = (!cpu.carry) & 0x01;
                        cpu.carry = 0;
                        break;
                }
                break;
        }

        if (is_two_cycle) {
            cpu.in_second_cycle = true;
            cpu.first_opr = opr;
            cpu.first_opa = opa;
            cpu.first_page = cpu.pc & 0xF00;
            cpu.pc = (cpu.pc + 1) & 0x0FFF;
        } else {
            cpu.pc = (cpu.pc + 1) & 0x0FFF;
        }

        clock_phase_out_cm(0x0, false, false, false);
        clock_phase_out_cm(0x0, false, false, false);
    }
}

void setup(void) {
    DDRD |= (1 << PHI1_PIN) | (1 << PHI2_PIN);
    DDRD &= ~((1 << TEST_PIN) | (1 << INT_PIN));
    PORTD |= (1 << TEST_PIN) | (1 << INT_PIN);

    DDRA |= (1 << SYNC_PIN) | (1 << PA5) | (1 << PA6);
    DDRC = 0xFF;

    PORTD &= ~((1 << PHI1_PIN) | (1 << PHI2_PIN));
    PORTA &= ~(1 << SYNC_PIN);

    cpu.acc = 0; cpu.carry = 0; cpu.pc = 0;
    cpu.rom_bank = 0; cpu.ram_bank = 0; cpu.reg_bank = 0;
    cpu.sp = 0; cpu.in_second_cycle = false;
    cpu.interrupt_enable = false; cpu.in_interrupt = false;
    cpu.halted = false;
}

void loop(void) {
    while (1) {
        i4040_hardware_step();
    }
}
