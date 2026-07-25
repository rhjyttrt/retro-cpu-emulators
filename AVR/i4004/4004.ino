#define F_CPU 20000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdbool.h>

// --- ATMEGA328P PIN DEFINITIONS ---
#define BUS_PORT PORTD
#define BUS_PIN   PIND
#define BUS_DDR   DDRD
#define BUS_MASK  0x0F

#define CTRL_PORT PORTB
#define CTRL_PIN  PINB
#define CTRL_DDR  DDRB

#define PIN_CLK1  PB0   // Driven internally
#define PIN_CLK2  PB1   // Driven internally
#define PIN_SYNC  PB2
#define PIN_RESET PB3
#define PIN_TEST  PB4

#define CM_PORT PORTC
#define CM_DDR  DDRC

#define PIN_CM_ROM  PC0
#define PIN_CM_RAM0 PC1
#define PIN_CM_RAM1 PC2
#define PIN_CM_RAM2 PC3
#define PIN_CM_RAM3 PC4

// --- 4004 ARCHITECTURAL STATE ---
static uint16_t pc = 0;           
static uint16_t stack[3] = {0};   
static uint8_t  R[16] = {0};      
static uint8_t  acc = 0;         
static uint8_t  cy = 0;           
static uint8_t  cm_ram_bank = 0;  

static bool is_second_cycle = false;
static uint8_t opr1 = 0, opa1 = 0;

// --- FIXED TWO-PHASE CLOCK & BUS LATCH GENERATOR ---
static inline void phase_clk1(void) {
    CTRL_PORT = (CTRL_PORT & ~(1 << PIN_CLK2)) | (1 << PIN_CLK1);
    _delay_us(0.40);
    CTRL_PORT &= ~((1 << PIN_CLK1) | (1 << PIN_CLK2));
    _delay_us(0.25);
}

static inline uint8_t phase_clk2_and_read(bool sample_bus) {
    CTRL_PORT = (CTRL_PORT & ~(1 << PIN_CLK1)) | (1 << PIN_CLK2);
    _delay_us(0.30);
    
    uint8_t data = 0;
    if (sample_bus) {
        data = BUS_PIN & BUS_MASK;
    }
    
    _delay_us(0.10);
    CTRL_PORT &= ~((1 << PIN_CLK1) | (1 << PIN_CLK2));
    _delay_us(0.25);
    
    return data;
}

static inline void sync_clk_phase(void) {
    phase_clk1();
    phase_clk2_and_read(false);
}

static inline uint8_t sync_clk_phase_read(void) {
    phase_clk1();
    return phase_clk2_and_read(true);
}

// --- STACK HELPERS ---
static inline void push_stack(uint16_t ret_pc) {
    stack[2] = stack[1];
    stack[1] = stack[0];
    stack[0] = ret_pc;
}

static inline uint16_t pop_stack(void) {
    uint16_t ret = stack[0];
    stack[0] = stack[1];
    stack[1] = stack[2];
    return ret;
}

static inline bool is_two_cycle_op(uint8_t opr, uint8_t opa) {
    if (opr == 0x1 || opr == 0x4 || opr == 0x5 || opr == 0x7) return true;
    if (opr == 0x2 && ((opa & 1) == 0)) return true; 
    if (opr == 0x3 && ((opa & 1) == 0)) return true; 
    return false;
}

// --- HARDWARE INITIALIZATION ---
void i4004_init(void) {
    BUS_DDR &= ~BUS_MASK;
    BUS_PORT &= ~BUS_MASK;

    CTRL_DDR |= (1 << PIN_CLK1) | (1 << PIN_CLK2) | (1 << PIN_SYNC);
    CTRL_DDR &= ~((1 << PIN_RESET) | (1 << PIN_TEST));
    CTRL_PORT &= ~((1 << PIN_CLK1) | (1 << PIN_CLK2) | (1 << PIN_SYNC));
    CTRL_PORT |= (1 << PIN_TEST); // Active-low TEST line internal pull-up

    CM_DDR |= (1 << PIN_CM_ROM) | (1 << PIN_CM_RAM0) | (1 << PIN_CM_RAM1) | (1 << PIN_CM_RAM2) | (1 << PIN_CM_RAM3);
    CM_PORT &= ~0x1F;
}

// --- SINGLE 8-PHASE INSTRUCTION CYCLE ---
void i4004_step_cycle(void) {
    uint8_t fetched_opr = 0, fetched_opa = 0;

    // ACTIVE RESET CHECK
    if (CTRL_PIN & (1 << PIN_RESET)) {
        pc = 0;
        stack[0] = stack[1] = stack[2] = 0;
        acc = 0;
        cy = 0;
        cm_ram_bank = 0;
        is_second_cycle = false;
        CM_PORT &= ~0x1F;
        BUS_DDR &= ~BUS_MASK;
        BUS_PORT &= ~BUS_MASK;
        return; // Suppress cycle execution during active reset
    }

    uint8_t current_bank = cm_ram_bank & 0x03;

    uint16_t addr_out = pc;
    if (is_second_cycle && opr1 == 0x3 && ((opa1 & 1) == 0)) { 
        // FIN: Derives page index from where FIN instruction resides (pc - 1)
        uint16_t fin_page = ((pc - 1) & 0xFFF) & 0xF00;
        addr_out = fin_page | (R[0] << 4) | R[1];
    }

    // PHASE A1
    CTRL_PORT |= (1 << PIN_SYNC);
    BUS_DDR |= BUS_MASK;
    BUS_PORT = (BUS_PORT & ~BUS_MASK) | (addr_out & 0x0F);
    sync_clk_phase();

    // PHASE A2
    CTRL_PORT &= ~(1 << PIN_SYNC);
    BUS_PORT = (BUS_PORT & ~BUS_MASK) | ((addr_out >> 4) & 0x0F);
    sync_clk_phase();

    // PHASE A3
    BUS_PORT = (BUS_PORT & ~BUS_MASK) | ((addr_out >> 8) & 0x0F);
    sync_clk_phase();

    // PHASE M1 (FETCH OPR)
    BUS_DDR &= ~BUS_MASK;
    BUS_PORT &= ~BUS_MASK;
    fetched_opr = sync_clk_phase_read();

    // PHASE M2 (FETCH OPA)
    fetched_opa = sync_clk_phase_read();

    if (!(is_second_cycle && opr1 == 0x3 && ((opa1 & 1) == 0))) {
        pc = (pc + 1) & 0xFFF;
    }

    // PHASE X1
    uint8_t opr, opa;
    if (!is_second_cycle) {
        opr1 = fetched_opr;
        opa1 = fetched_opa;
        opr = opr1;
        opa = opa1;
    } else {
        opr = opr1;
        opa = opa1;
    }
    sync_clk_phase();

    // PHASE X2
    uint8_t ram_data = 0;
    if (opr == 0x2 && ((opa & 1) != 0)) { // SRC
        uint8_t pair = opa >> 1;
        BUS_DDR |= BUS_MASK;
        BUS_PORT = (BUS_PORT & ~BUS_MASK) | (R[pair * 2 + 1] & 0x0F);
        sync_clk_phase();
    } 
    else if (opr == 0xE) {
        if (opa == 0x2 || opa == 0xA) {
            CM_PORT |= (1 << PIN_CM_ROM);
        } else {
            CM_PORT |= (1 << (PIN_CM_RAM0 + current_bank));
        }

        if (opa <= 0x7) { // Writes
            BUS_DDR |= BUS_MASK;
            BUS_PORT = (BUS_PORT & ~BUS_MASK) | (acc & 0x0F);
            sync_clk_phase();
        } else { // Reads (opa >= 0x8)
            BUS_DDR &= ~BUS_MASK;
            BUS_PORT &= ~BUS_MASK; // Disable pull-ups for true Hi-Z input
            ram_data = sync_clk_phase_read(); // Sample bus in Phase X2
        }
    } else {
        sync_clk_phase();
    }

    // PHASE X3
    if (opr == 0x2 && ((opa & 1) != 0)) { // SRC
        BUS_PORT = (BUS_PORT & ~BUS_MASK) | (R[(opa >> 1) * 2] & 0x0F);
        sync_clk_phase();
    } 
    else if (opr == 0xE && opa >= 0x8) { 
        sync_clk_phase(); // Clock phase; execution uses data sampled in X2
        
        if (opa == 0x8) { // SBM
            uint8_t diff = acc + (~ram_data & 0x0F) + cy;
            acc = diff & 0x0F;
            cy = (diff > 15) ? 1 : 0;
        } else if (opa == 0x9 || opa == 0xA || opa >= 0xC) {
            acc = ram_data;
        } else if (opa == 0xB) { // ADM
            uint8_t sum = acc + ram_data + cy;
            acc = sum & 0x0F;
            cy = (sum > 15) ? 1 : 0;
        }
    } else {
        sync_clk_phase();
    }

    // EXECUTION UNIT
    if (!is_second_cycle) {
        if (is_two_cycle_op(opr, opa)) {
            is_second_cycle = true;
        } else {
            switch (opr) {
                case 0x0: break; // NOP
                case 0x3: 
                    if ((opa & 1) != 0) { // JIN
                        uint8_t p = opa >> 1;
                        pc = (pc & 0xF00) | (R[p * 2] << 4) | R[p * 2 + 1];
                    }
                    break;
                case 0x6: R[opa] = (R[opa] + 1) & 0x0F; break; // INC
                case 0x8: { uint8_t s = acc + R[opa] + cy; acc = s & 0x0F; cy = (s > 15) ? 1 : 0; break; } // ADD
                case 0x9: { uint8_t d = acc + (~R[opa] & 0x0F) + cy; acc = d & 0x0F; cy = (d > 15) ? 1 : 0; break; } // SUB
                case 0xA: acc = R[opa] & 0x0F; break; // LD
                case 0xB: { uint8_t t = acc; acc = R[opa]; R[opa] = t; break; } // XCH
                case 0xC: acc = opa & 0x0F; pc = pop_stack(); break; // BBL
                case 0xD: acc = opa & 0x0F; break; // LDM
                case 0xF:
                    switch (opa) {
                        case 0x0: acc = 0; cy = 0; break; // CLB
                        case 0x1: cy = 0; break; // CLC
                        case 0x2: acc = (acc + 1) & 0x0F; cy = (acc == 0) ? 1 : 0; break; // IAC
                        case 0x3: cy = !cy; break; // CMC
                        case 0x4: acc = (~acc) & 0x0F; break; // CMA
                        case 0x5: { uint8_t t = (acc << 1) | cy; acc = t & 0x0F; cy = (t >> 4) & 1; break; } // RAL
                        case 0x6: { uint8_t t = (cy << 4) | acc; acc = (t >> 1) & 0x0F; cy = t & 1; break; } // RAR
                        case 0x7: acc = cy; cy = 0; break; // TCC
                        case 0x8: cy = (acc != 0) ? 1 : 0; acc = (acc - 1) & 0x0F; break; // DAC
                        case 0x9: acc = cy ? 10 : 9; cy = 0; break; // TCS
                        case 0xA: cy = 1; break; // STC
                        case 0xB: if (acc > 9 || cy) { acc = (acc + 6) & 0x0F; cy = 1; } break; // DAA
                        case 0xC: // KBP
                            if (acc == 0) acc = 0;
                            else if (acc == 1) acc = 1;
                            else if (acc == 2) acc = 2;
                            else if (acc == 4) acc = 3;
                            else if (acc == 8) acc = 4;
                            else acc = 15;
                            break;
                        case 0xD: // DCL (1-hot bank decoding)
                            if (acc & 0x01) cm_ram_bank = 1;
                            else if (acc & 0x02) cm_ram_bank = 2;
                            else if (acc & 0x04) cm_ram_bank = 3;
                            else cm_ram_bank = 0; 
                            break;
                    }
                    break;
            }
        }
    } else {
        uint8_t data8 = (fetched_opr << 4) | fetched_opa;
        switch (opr) {
            case 0x1: { // JCN
                bool c1 = (opa & 0x8) != 0;
                bool c2 = (opa & 0x4) && (acc == 0);
                bool c3 = (opa & 0x2) && (cy == 1);
                bool c4 = (opa & 0x1) && !(CTRL_PIN & (1 << PIN_TEST));
                bool cond = c2 || c3 || c4;
                if (c1) cond = !cond;
                if (cond) pc = (pc & 0xF00) | data8;
                break;
            }
            case 0x2: // FIM
                R[(opa >> 1) * 2]     = fetched_opr;
                R[(opa >> 1) * 2 + 1] = fetched_opa;
                break;
            case 0x3: // FIN
                R[(opa >> 1) * 2]     = fetched_opr;
                R[(opa >> 1) * 2 + 1] = fetched_opa;
                break;
            case 0x4: pc = ((opa & 0x0F) << 8) | data8; break; // JUN
            case 0x5: push_stack(pc); pc = ((opa & 0x0F) << 8) | data8; break; // JMS
            case 0x7: // ISZ
                R[opa] = (R[opa] + 1) & 0x0F;
                if (R[opa] != 0) pc = (pc & 0xF00) | data8;
                break;
        }
        is_second_cycle = false;
    }

    BUS_DDR &= ~BUS_MASK;
    BUS_PORT &= ~BUS_MASK; // Clear port latch to maintain high-impedance mode
    CM_PORT &= ~0x1F;
}

int main(void) {
    i4004_init();

    while (1) {
        i4004_step_cycle();
    }
    return 0;
}