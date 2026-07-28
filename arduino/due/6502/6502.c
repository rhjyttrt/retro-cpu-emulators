#include "6502.h"

#if defined(__arm__) && defined(__SAM3X8E__)
#include <Arduino.h>
#include "sam.h"

#ifndef DWT
typedef struct {
    volatile uint32_t CTRL;
    volatile uint32_t CYCCNT;
} DWT_Type;
#define DWT ((DWT_Type *) 0xE0001000UL)
#define DWT_CTRL_CYCCNTENA_Msk (1UL << 0)
#endif

#ifndef CoreDebug
typedef struct {
    volatile uint32_t DEMCR;
} CoreDebug_Type;
#define CoreDebug ((CoreDebug_Type *) 0xE000EDFCUL)
#define CoreDebug_DEMCR_TRCENA_Msk (1UL << 24)
#endif
#endif

CPU6502 cpu;

static int cycle_counter = 0;
static bool irq_line_asserted = false;
static bool nmi_pending = false;
static bool reset_pending = false;
static bool last_nmi_state = true;
static bool last_so_state = true;

#define FLAG_C (1 << 0)
#define FLAG_Z (1 << 1)
#define FLAG_I (1 << 2)
#define FLAG_D (1 << 3)
#define FLAG_B (1 << 4)
#define FLAG_U (1 << 5)
#define FLAG_V (1 << 6)
#define FLAG_N (1 << 7)

#define PORTC_ADDR_MASK 0x0007F3FEUL

#define PHI2_HIGH()   (PIOD->PIO_SODR = (1UL << 0))
#define PHI2_LOW()    (PIOD->PIO_CODR = (1UL << 0))

#define RW_READ()     (PIOD->PIO_SODR = (1UL << 1))
#define RW_WRITE()    (PIOD->PIO_CODR = (1UL << 1))

#define SYNC_HIGH()   (PIOD->PIO_SODR = (1UL << 2))
#define SYNC_LOW()    (PIOD->PIO_CODR = (1UL << 2))

#define PHI1_HIGH()   (PIOD->PIO_SODR = (1UL << 3))
#define PHI1_LOW()    (PIOD->PIO_CODR = (1UL << 3))

#define READ_PIN_RDY()  (PIOD->PIO_PDSR & (1UL << 4))
#define READ_PIN_IRQ()  (!(PIOD->PIO_PDSR & (1UL << 5)))
#define READ_PIN_NMI()  (!(PIOD->PIO_PDSR & (1UL << 6)))
#define READ_PIN_SO()   (!(PIOD->PIO_PDSR & (1UL << 7)))
#define READ_PIN_RES()  (!(PIOD->PIO_PDSR & (1UL << 8)))

static inline void init_dwt(void) {
#if defined(__arm__) && defined(__SAM3X8E__)
    if (!(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
#endif
}

static inline void delay_250ns(void) {
#if defined(__arm__) && defined(__SAM3X8E__)
    uint32_t start = DWT->CYCCNT;
    while ((uint32_t)(DWT->CYCCNT - start) < 11);
#else
    for (volatile int i = 0; i < 4; i++) { __asm__("nop"); }
#endif
}

static inline void sample_inputs_per_cycle(void) {
#if defined(__arm__) && defined(__SAM3X8E__)
    bool curr_so = READ_PIN_SO();
    if (curr_so && !last_so_state) {
        cpu.status |= FLAG_V;
    }
    last_so_state = curr_so;

    bool curr_nmi = READ_PIN_NMI();
    if (curr_nmi && !last_nmi_state) {
        nmi_pending = true;
    }
    last_nmi_state = curr_nmi;

    irq_line_asserted = READ_PIN_IRQ();
#endif
}

static inline void set_data_bus_direction(bool output_mode) {
#if defined(__arm__) && defined(__SAM3X8E__)
    if (output_mode) {
        PIOA->PIO_OER = 0xFFUL;
    } else {
        PIOA->PIO_ODR = 0xFFUL;
    }
#endif
}

static inline void drive_address_bus(uint16_t addr) {
#if defined(__arm__) && defined(__SAM3X8E__)
    uint32_t val = ((addr & 0x01FF) << 1)   | 
                   ((addr & 0x0600) << 8)   | 
                   ((addr & 0xF800) << 1);

    PIOC->PIO_ODSR = (PIOC->PIO_ODSR & ~PORTC_ADDR_MASK) | val;
#endif
}

static inline uint8_t read_byte(uint16_t addr, bool is_sync) {
#if defined(__arm__) && defined(__SAM3X8E__)
    while (!READ_PIN_RDY()) {}

    cycle_counter++;

    set_data_bus_direction(false);

    PHI2_LOW();
    PHI1_HIGH();
    drive_address_bus(addr);
    RW_READ();
    
    cpu.sync = is_sync;
    if (is_sync) SYNC_HIGH(); else SYNC_LOW();
    delay_250ns();

    PHI1_LOW();
    PHI2_HIGH();
    delay_250ns();
    
    uint8_t data = (uint8_t)(PIOA->PIO_PDSR & 0xFFUL);
    sample_inputs_per_cycle();

    PHI2_LOW();
    return data;
#else
    (void)addr; (void)is_sync; return 0xEA;
#endif
}

static inline void write_byte(uint16_t addr, uint8_t data) {
#if defined(__arm__) && defined(__SAM3X8E__)
    cycle_counter++;

    PHI2_LOW();
    PHI1_HIGH();
    drive_address_bus(addr);
    RW_WRITE();
    cpu.sync = false;
    SYNC_LOW();
    delay_250ns();

    PIOA->PIO_ODSR = (PIOA->PIO_ODSR & ~0xFFUL) | data;
    set_data_bus_direction(true);

    PHI1_LOW();
    PHI2_HIGH();
    delay_250ns();

    sample_inputs_per_cycle();

    PHI2_LOW();

    __asm__ volatile ("nop\n\tnop\n\tnop\n\tnop\n\t");

    set_data_bus_direction(false);
#else
    (void)addr; (void)data;
#endif
}

static inline uint16_t read_word(uint16_t addr) { 
    uint8_t lo = read_byte(addr, false);
    uint8_t hi = read_byte((uint16_t)(addr + 1), false);
    return lo | (hi << 8); 
}

static inline void push_byte(uint8_t val) { write_byte(0x0100 + cpu.sp--, val); }
static inline uint8_t pop_byte(void) { return read_byte(0x0100 + ++cpu.sp, false); }
static inline void push_word(uint16_t val) { push_byte(val >> 8); push_byte(val & 0xFF); }
static inline uint16_t pop_word(void) { uint16_t l = pop_byte(); uint16_t h = pop_byte(); return (h << 8) | l; }

static inline void set_flag(uint8_t flag, bool cond) {
    if (cond) cpu.status |= flag; else cpu.status &= ~flag;
    cpu.status |= FLAG_U;
}

static inline void update_zn(uint8_t val) {
    set_flag(FLAG_Z, val == 0);
    set_flag(FLAG_N, val & 0x80);
}

static inline uint16_t addr_imm(void) { return cpu.pc++; }
static inline uint16_t addr_zp(void)  { return read_byte(cpu.pc++, false); }

static inline uint16_t addr_zpx(void) { 
    uint8_t zp = read_byte(cpu.pc++, false);
    read_byte(zp, false);
    return (zp + cpu.x) & 0xFF; 
}

static inline uint16_t addr_zpy(void) { 
    uint8_t zp = read_byte(cpu.pc++, false);
    read_byte(zp, false);
    return (zp + cpu.y) & 0xFF; 
}

static inline uint16_t addr_abs(void) { uint16_t a = read_word(cpu.pc); cpu.pc += 2; return a; }

static inline uint16_t addr_abx(void) {
    uint16_t base = read_word(cpu.pc);
    uint16_t addr = base + cpu.x;
    if (((base ^ addr) & 0xFF00) != 0) {
        read_byte((base & 0xFF00) | (addr & 0x00FF), false);
    }
    cpu.pc += 2; return addr;
}

static inline uint16_t addr_aby(void) {
    uint16_t base = read_word(cpu.pc);
    uint16_t addr = base + cpu.y;
    if (((base ^ addr) & 0xFF00) != 0) {
        read_byte((base & 0xFF00) | (addr & 0x00FF), false);
    }
    cpu.pc += 2; return addr;
}

static inline uint16_t addr_abx_store(void) {
    uint16_t base = read_word(cpu.pc);
    uint16_t addr = base + cpu.x;
    read_byte((base & 0xFF00) | (addr & 0x00FF), false);
    cpu.pc += 2; return addr;
}

static inline uint16_t addr_aby_store(void) {
    uint16_t base = read_word(cpu.pc);
    uint16_t addr = base + cpu.y;
    read_byte((base & 0xFF00) | (addr & 0x00FF), false);
    cpu.pc += 2; return addr;
}

static inline uint16_t addr_ind(void) {
    uint16_t ptr = read_word(cpu.pc); cpu.pc += 2;
    uint16_t ptr_hi = ((ptr & 0x00FF) == 0x00FF) ? (ptr & 0xFF00) : (ptr + 1);
    uint8_t lo = read_byte(ptr, false);
    uint8_t hi = read_byte(ptr_hi, false);
    return lo | (hi << 8);
}

static inline uint16_t addr_izx(void) {
    uint8_t z = read_byte(cpu.pc++, false);
    read_byte(z, false);
    uint8_t ptr = (z + cpu.x) & 0xFF;
    uint8_t lo = read_byte(ptr, false);
    uint8_t hi = read_byte((uint8_t)(ptr + 1), false);
    return lo | (hi << 8);
}

static inline uint16_t addr_izy(void) {
    uint8_t z = read_byte(cpu.pc++, false);
    uint8_t lo = read_byte(z, false);
    uint8_t hi = read_byte((uint8_t)(z + 1), false);
    uint16_t base = lo | (hi << 8);
    uint16_t addr = base + cpu.y;
    if (((base ^ addr) & 0xFF00) != 0) {
        read_byte((base & 0xFF00) | (addr & 0x00FF), false);
    }
    return addr;
}

static inline uint16_t addr_izy_store(void) {
    uint8_t z = read_byte(cpu.pc++, false);
    uint8_t lo = read_byte(z, false);
    uint8_t hi = read_byte((uint8_t)(z + 1), false);
    uint16_t base = lo | (hi << 8);
    uint16_t addr = base + cpu.y;
    read_byte((base & 0xFF00) | (addr & 0x00FF), false);
    return addr;
}

static void do_adc(uint8_t val) {
    uint8_t c = (cpu.status & FLAG_C) ? 1 : 0;
    if (cpu.status & FLAG_D) {
        uint16_t low = (cpu.a & 0x0F) + (val & 0x0F) + c;
        if (low > 9) low += 6;
        uint16_t high = (cpu.a >> 4) + (val >> 4) + (low > 15 ? 1 : 0);
        if (high > 9) high += 6;
        
        uint16_t bin = cpu.a + val + c;
        set_flag(FLAG_C, high > 15);
        set_flag(FLAG_Z, (bin & 0xFF) == 0);
        set_flag(FLAG_N, (bin & 0x80) != 0);
        set_flag(FLAG_V, (~(cpu.a ^ val) & (cpu.a ^ bin) & 0x80) != 0);
        cpu.a = ((high & 0x0F) << 4) | (low & 0x0F);
    } else {
        uint16_t sum = cpu.a + val + c;
        set_flag(FLAG_C, sum > 0xFF);
        set_flag(FLAG_V, (~(cpu.a ^ val) & (cpu.a ^ sum) & 0x80) != 0);
        cpu.a = (uint8_t)sum;
        update_zn(cpu.a);
    }
}

static void do_sbc(uint8_t val) {
    if (cpu.status & FLAG_D) {
        uint8_t c = (cpu.status & FLAG_C) ? 0 : 1;
        int16_t bin_diff = (int16_t)cpu.a - (int16_t)val - (int16_t)c;
        
        int16_t low = (cpu.a & 0x0F) - (val & 0x0F) - c;
        if (low < 0) low -= 6;
        
        int16_t high = (cpu.a >> 4) - (val >> 4) - (low < 0 ? 1 : 0);
        if (high < 0) high -= 6;
        
        set_flag(FLAG_C, bin_diff >= 0);
        set_flag(FLAG_V, ((cpu.a ^ val) & (cpu.a ^ bin_diff) & 0x80) != 0);
        update_zn((uint8_t)bin_diff);
        cpu.a = (((uint8_t)high & 0x0F) << 4) | ((uint8_t)low & 0x0F);
    } else {
        do_adc(~val);
    }
}

static inline void do_cmp(uint8_t reg, uint8_t val) {
    set_flag(FLAG_C, reg >= val);
    update_zn(reg - val);
}

static inline void do_branch(bool cond) {
    int8_t offset = (int8_t)read_byte(cpu.pc++, false);
    if (cond) {
        read_byte(cpu.pc, false);
        uint16_t target = cpu.pc + offset;
        if ((cpu.pc ^ target) & 0xFF00) {
            read_byte((cpu.pc & 0xFF00) | (target & 0x00FF), false);
        }
        cpu.pc = target;
    }
}

void cpu_reset(void) {
    init_dwt();

#if defined(__arm__) && defined(__SAM3X8E__)
    PMC->PMC_PCER0 = (1u << ID_PIOA) | (1u << ID_PIOC) | (1u << ID_PIOD);

    PIOA->PIO_PER = 0xFFUL;              
    PIOC->PIO_PER = PORTC_ADDR_MASK;     
    PIOD->PIO_PER = 0x1FFUL;             

    PIOC->PIO_OER = PORTC_ADDR_MASK;     
    PIOD->PIO_OER = 0x0FUL;              

    PIOA->PIO_OWER = 0xFFUL;             
    PIOC->PIO_OWER = PORTC_ADDR_MASK;    

    PIOD->PIO_ODR = 0x1F0UL;             
    PIOD->PIO_PUER = 0x1F0UL;

    set_data_bus_direction(false);
#endif

    cpu.a = cpu.x = cpu.y = 0;
    cpu.sp = 0xFF;
    cpu.status = FLAG_U | FLAG_I;
    cpu.rdy = true;
    cpu.sync = false;
    
    nmi_pending = false;
    reset_pending = false;
    irq_line_asserted = false;

    read_byte(0x0000, false);
    read_byte(0x0001, false);
    read_byte(0x0100 + cpu.sp, false);
    read_byte(0x0100 + ((cpu.sp - 1) & 0xFF), false);
    read_byte(0x0100 + ((cpu.sp - 2) & 0xFF), false);

    cpu.pc = read_word(0xFFFC);

#if defined(__arm__) && defined(__SAM3X8E__)
    last_nmi_state = READ_PIN_NMI();
    last_so_state = READ_PIN_SO();
#endif
}

int cpu_nmi(void) {
    read_byte(cpu.pc, false);
    read_byte(cpu.pc, false);
    push_word(cpu.pc);
    push_byte((cpu.status & ~FLAG_B) | FLAG_U);
    cpu.status |= FLAG_I;
    cpu.pc = read_word(0xFFFA);
    return cycle_counter;
}

int cpu_irq(void) {
    if (!(cpu.status & FLAG_I)) {
        read_byte(cpu.pc, false);
        read_byte(cpu.pc, false);
        push_word(cpu.pc);
        push_byte((cpu.status & ~FLAG_B) | FLAG_U);
        cpu.status |= FLAG_I;
        cpu.pc = read_word(0xFFFE);
        return cycle_counter;
    }
    return 0;
}

int cpu_step(void) {
    cycle_counter = 0;

#if defined(__arm__) && defined(__SAM3X8E__)
    if (READ_PIN_RES()) {
        reset_pending = true;
        return 0;
    }

    if (reset_pending) {
        reset_pending = false;
        cpu_reset();
        return cycle_counter;
    }
#else
    if (reset_pending) {
        reset_pending = false;
        cpu_reset();
        return cycle_counter;
    }
#endif

    if (nmi_pending) {
        nmi_pending = false;
        return cpu_nmi();
    }
    if (irq_line_asserted && !(cpu.status & FLAG_I)) {
        return cpu_irq();
    }

    uint8_t opcode = read_byte(cpu.pc++, true);
    uint16_t addr = 0;
    uint8_t val = 0;

    switch (opcode) {
        case 0xA9: cpu.a = read_byte(addr_imm(), false); update_zn(cpu.a); break; 
        case 0xA5: cpu.a = read_byte(addr_zp(), false);  update_zn(cpu.a); break;
        case 0xB5: cpu.a = read_byte(addr_zpx(), false); update_zn(cpu.a); break;
        case 0xAD: cpu.a = read_byte(addr_abs(), false); update_zn(cpu.a); break;
        case 0xBD: cpu.a = read_byte(addr_abx(), false); update_zn(cpu.a); break;
        case 0xB9: cpu.a = read_byte(addr_aby(), false); update_zn(cpu.a); break;
        case 0xA1: cpu.a = read_byte(addr_izx(), false); update_zn(cpu.a); break;
        case 0xB1: cpu.a = read_byte(addr_izy(), false); update_zn(cpu.a); break;

        case 0xA2: cpu.x = read_byte(addr_imm(), false); update_zn(cpu.x); break; 
        case 0xA6: cpu.x = read_byte(addr_zp(), false);  update_zn(cpu.x); break;
        case 0xB6: cpu.x = read_byte(addr_zpy(), false); update_zn(cpu.x); break;
        case 0xAE: cpu.x = read_byte(addr_abs(), false); update_zn(cpu.x); break;
        case 0xBE: cpu.x = read_byte(addr_aby(), false); update_zn(cpu.x); break;

        case 0xA0: cpu.y = read_byte(addr_imm(), false); update_zn(cpu.y); break; 
        case 0xA4: cpu.y = read_byte(addr_zp(), false);  update_zn(cpu.y); break;
        case 0xB4: cpu.y = read_byte(addr_zpx(), false); update_zn(cpu.y); break;
        case 0xAC: cpu.y = read_byte(addr_abs(), false); update_zn(cpu.y); break;
        case 0xBC: cpu.y = read_byte(addr_abx(), false); update_zn(cpu.y); break;

        case 0x85: write_byte(addr_zp(), cpu.a);  break; 
        case 0x95: write_byte(addr_zpx(), cpu.a); break;
        case 0x8D: write_byte(addr_abs(), cpu.a); break;
        case 0x9D: write_byte(addr_abx_store(), cpu.a); break;
        case 0x99: write_byte(addr_aby_store(), cpu.a); break;
        case 0x81: write_byte(addr_izx(), cpu.a); break;
        case 0x91: write_byte(addr_izy_store(), cpu.a); break;

        case 0x86: write_byte(addr_zp(), cpu.x);  break; 
        case 0x96: write_byte(addr_zpy(), cpu.x); break;
        case 0x8E: write_byte(addr_abs(), cpu.x); break;

        case 0x84: write_byte(addr_zp(), cpu.y);  break; 
        case 0x94: write_byte(addr_zpx(), cpu.y); break;
        case 0x8C: write_byte(addr_abs(), cpu.y); break;

        case 0x69: do_adc(read_byte(addr_imm(), false)); break; 
        case 0x65: do_adc(read_byte(addr_zp(), false));  break;
        case 0x75: do_adc(read_byte(addr_zpx(), false)); break;
        case 0x6D: do_adc(read_byte(addr_abs(), false)); break;
        case 0x7D: do_adc(read_byte(addr_abx(), false)); break;
        case 0x79: do_adc(read_byte(addr_aby(), false)); break;
        case 0x61: do_adc(read_byte(addr_izx(), false)); break;
        case 0x71: do_adc(read_byte(addr_izy(), false)); break;

        case 0xE9: case 0xEB: do_sbc(read_byte(addr_imm(), false)); break; 
        case 0xE5: do_sbc(read_byte(addr_zp(), false));  break;
        case 0xF5: do_sbc(read_byte(addr_zpx(), false)); break;
        case 0xED: do_sbc(read_byte(addr_abs(), false)); break;
        case 0xFD: do_sbc(read_byte(addr_abx(), false)); break;
        case 0xF9: do_sbc(read_byte(addr_aby(), false)); break;
        case 0xE1: do_sbc(read_byte(addr_izx(), false)); break;
        case 0xF1: do_sbc(read_byte(addr_izy(), false)); break;

        case 0x29: cpu.a &= read_byte(addr_imm(), false); update_zn(cpu.a); break; 
        case 0x25: cpu.a &= read_byte(addr_zp(), false);  update_zn(cpu.a); break;
        case 0x35: cpu.a &= read_byte(addr_zpx(), false); update_zn(cpu.a); break;
        case 0x2D: cpu.a &= read_byte(addr_abs(), false); update_zn(cpu.a); break;
        case 0x3D: cpu.a &= read_byte(addr_abx(), false); update_zn(cpu.a); break;
        case 0x39: cpu.a &= read_byte(addr_aby(), false); update_zn(cpu.a); break;
        case 0x21: cpu.a &= read_byte(addr_izx(), false); update_zn(cpu.a); break;
        case 0x31: cpu.a &= read_byte(addr_izy(), false); update_zn(cpu.a); break;

        case 0x09: cpu.a |= read_byte(addr_imm(), false); update_zn(cpu.a); break; 
        case 0x05: cpu.a |= read_byte(addr_zp(), false);  update_zn(cpu.a); break;
        case 0x15: cpu.a |= read_byte(addr_zpx(), false); update_zn(cpu.a); break;
        case 0x0D: cpu.a |= read_byte(addr_abs(), false); update_zn(cpu.a); break;
        case 0x1D: cpu.a |= read_byte(addr_abx(), false); update_zn(cpu.a); break;
        case 0x19: cpu.a |= read_byte(addr_aby(), false); update_zn(cpu.a); break;
        case 0x01: cpu.a |= read_byte(addr_izx(), false); update_zn(cpu.a); break;
        case 0x11: cpu.a |= read_byte(addr_izy(), false); update_zn(cpu.a); break;

        case 0x49: cpu.a ^= read_byte(addr_imm(), false); update_zn(cpu.a); break; 
        case 0x45: cpu.a ^= read_byte(addr_zp(), false);  update_zn(cpu.a); break;
        case 0x55: cpu.a ^= read_byte(addr_zpx(), false); update_zn(cpu.a); break;
        case 0x4D: cpu.a ^= read_byte(addr_abs(), false); update_zn(cpu.a); break;
        case 0x5D: cpu.a ^= read_byte(addr_abx(), false); update_zn(cpu.a); break;
        case 0x59: cpu.a ^= read_byte(addr_aby(), false); update_zn(cpu.a); break;
        case 0x41: cpu.a ^= read_byte(addr_izx(), false); update_zn(cpu.a); break;
        case 0x51: cpu.a ^= read_byte(addr_izy(), false); update_zn(cpu.a); break;

        case 0xC9: do_cmp(cpu.a, read_byte(addr_imm(), false)); break; 
        case 0xC5: do_cmp(cpu.a, read_byte(addr_zp(), false));  break;
        case 0xD5: do_cmp(cpu.a, read_byte(addr_zpx(), false)); break;
        case 0xCD: do_cmp(cpu.a, read_byte(addr_abs(), false)); break;
        case 0xDD: do_cmp(cpu.a, read_byte(addr_abx(), false)); break;
        case 0xD9: do_cmp(cpu.a, read_byte(addr_aby(), false)); break;
        case 0xC1: do_cmp(cpu.a, read_byte(addr_izx(), false)); break;
        case 0xD1: do_cmp(cpu.a, read_byte(addr_izy(), false)); break;

        case 0xE0: do_cmp(cpu.x, read_byte(addr_imm(), false)); break; 
        case 0xE4: do_cmp(cpu.x, read_byte(addr_zp(), false));  break;
        case 0xEC: do_cmp(cpu.x, read_byte(addr_abs(), false)); break;

        case 0xC0: do_cmp(cpu.y, read_byte(addr_imm(), false)); break; 
        case 0xC4: do_cmp(cpu.y, read_byte(addr_zp(), false));  break;
        case 0xCC: do_cmp(cpu.y, read_byte(addr_abs(), false)); break;

        case 0x0A: read_byte(cpu.pc, false); set_flag(FLAG_C, cpu.a & 0x80); cpu.a <<= 1; update_zn(cpu.a); break; 
        case 0x06: addr = addr_zp();  val = read_byte(addr, false); write_byte(addr, val); set_flag(FLAG_C, val & 0x80); val <<= 1; write_byte(addr, val); update_zn(val); break;
        case 0x16: addr = addr_zpx(); val = read_byte(addr, false); write_byte(addr, val); set_flag(FLAG_C, val & 0x80); val <<= 1; write_byte(addr, val); update_zn(val); break;
        case 0x0E: addr = addr_abs(); val = read_byte(addr, false); write_byte(addr, val); set_flag(FLAG_C, val & 0x80); val <<= 1; write_byte(addr, val); update_zn(val); break;
        case 0x1E: addr = addr_abx_store(); val = read_byte(addr, false); write_byte(addr, val); set_flag(FLAG_C, val & 0x80); val <<= 1; write_byte(addr, val); update_zn(val); break;

        case 0x4A: read_byte(cpu.pc, false); set_flag(FLAG_C, cpu.a & 0x01); cpu.a >>= 1; update_zn(cpu.a); break; 
        case 0x46: addr = addr_zp();  val = read_byte(addr, false); write_byte(addr, val); set_flag(FLAG_C, val & 0x01); val >>= 1; write_byte(addr, val); update_zn(val); break;
        case 0x56: addr = addr_zpx(); val = read_byte(addr, false); write_byte(addr, val); set_flag(FLAG_C, val & 0x01); val >>= 1; write_byte(addr, val); update_zn(val); break;
        case 0x4E: addr = addr_abs(); val = read_byte(addr, false); write_byte(addr, val); set_flag(FLAG_C, val & 0x01); val >>= 1; write_byte(addr, val); update_zn(val); break;
        case 0x5E: addr = addr_abx_store(); val = read_byte(addr, false); write_byte(addr, val); set_flag(FLAG_C, val & 0x01); val >>= 1; write_byte(addr, val); update_zn(val); break;

        case 0x2A: read_byte(cpu.pc, false); { uint8_t c = (cpu.status & FLAG_C) ? 1 : 0; set_flag(FLAG_C, cpu.a & 0x80); cpu.a = (cpu.a << 1) | c; update_zn(cpu.a); break; }
        case 0x26: case 0x36: case 0x2E: case 0x3E: { 
            addr = (opcode == 0x26) ? addr_zp() : (opcode == 0x36) ? addr_zpx() : (opcode == 0x2E) ? addr_abs() : addr_abx_store();
            val = read_byte(addr, false); write_byte(addr, val);
            uint8_t c = (cpu.status & FLAG_C) ? 1 : 0;
            set_flag(FLAG_C, val & 0x80); val = (val << 1) | c; write_byte(addr, val); update_zn(val);
            break;
        }

        case 0x6A: read_byte(cpu.pc, false); { uint8_t c = (cpu.status & FLAG_C) ? 0x80 : 0; set_flag(FLAG_C, cpu.a & 0x01); cpu.a = (cpu.a >> 1) | c; update_zn(cpu.a); break; }
        case 0x66: case 0x76: case 0x6E: case 0x7E: { 
            addr = (opcode == 0x66) ? addr_zp() : (opcode == 0x76) ? addr_zpx() : (opcode == 0x6E) ? addr_abs() : addr_abx_store();
            val = read_byte(addr, false); write_byte(addr, val);
            uint8_t c = (cpu.status & FLAG_C) ? 0x80 : 0;
            set_flag(FLAG_C, val & 0x01); val = (val >> 1) | c; update_zn(val);
            break;
        }

        case 0xE6: addr = addr_zp();  val = read_byte(addr, false); write_byte(addr, val); val++; write_byte(addr, val); update_zn(val); break; 
        case 0xF6: addr = addr_zpx(); val = read_byte(addr, false); write_byte(addr, val); val++; write_byte(addr, val); update_zn(val); break;
        case 0xEE: addr = addr_abs(); val = read_byte(addr, false); write_byte(addr, val); val++; write_byte(addr, val); update_zn(val); break;
        case 0xFE: addr = addr_abx_store(); val = read_byte(addr, false); write_byte(addr, val); val++; write_byte(addr, val); update_zn(val); break;

        case 0xC6: addr = addr_zp();  val = read_byte(addr, false); write_byte(addr, val); val--; write_byte(addr, val); update_zn(val); break; 
        case 0xD6: addr = addr_zpx(); val = read_byte(addr, false); write_byte(addr, val); val--; write_byte(addr, val); update_zn(val); break;
        case 0xCE: addr = addr_abs(); val = read_byte(addr, false); write_byte(addr, val); val--; write_byte(addr, val); update_zn(val); break;
        case 0xDE: addr = addr_abx_store(); val = read_byte(addr, false); write_byte(addr, val); val--; write_byte(addr, val); update_zn(val); break;

        case 0xE8: read_byte(cpu.pc, false); cpu.x++; update_zn(cpu.x); break; 
        case 0xC8: read_byte(cpu.pc, false); cpu.y++; update_zn(cpu.y); break; 
        case 0xCA: read_byte(cpu.pc, false); cpu.x--; update_zn(cpu.x); break; 
        case 0x88: read_byte(cpu.pc, false); cpu.y--; update_zn(cpu.y); break; 

        case 0xAA: read_byte(cpu.pc, false); cpu.x = cpu.a; update_zn(cpu.x); break; 
        case 0x8A: read_byte(cpu.pc, false); cpu.a = cpu.x; update_zn(cpu.a); break; 
        case 0xA8: read_byte(cpu.pc, false); cpu.y = cpu.a; update_zn(cpu.y); break; 
        case 0x98: read_byte(cpu.pc, false); cpu.a = cpu.y; update_zn(cpu.a); break; 
        case 0xBA: read_byte(cpu.pc, false); cpu.x = cpu.sp; update_zn(cpu.x); break; 
        case 0x9A: read_byte(cpu.pc, false); cpu.sp = cpu.x; break;                   

        case 0x48: read_byte(cpu.pc, false); push_byte(cpu.a); break; 
        case 0x08: read_byte(cpu.pc, false); push_byte(cpu.status | FLAG_B | FLAG_U); break; 
        case 0x68: read_byte(cpu.pc, false); read_byte(0x0100 + cpu.sp, false); cpu.a = pop_byte(); update_zn(cpu.a); break;     
        case 0x28: read_byte(cpu.pc, false); read_byte(0x0100 + cpu.sp, false); cpu.status = (pop_byte() & ~FLAG_B) | FLAG_U; break; 

        case 0x10: do_branch(!(cpu.status & FLAG_N)); break; 
        case 0x30: do_branch((cpu.status & FLAG_N));  break; 
        case 0x50: do_branch(!(cpu.status & FLAG_V)); break; 
        case 0x70: do_branch((cpu.status & FLAG_V));  break; 
        case 0x90: do_branch(!(cpu.status & FLAG_C)); break; 
        case 0xB0: do_branch((cpu.status & FLAG_C));  break; 
        case 0xD0: do_branch(!(cpu.status & FLAG_Z)); break; 
        case 0xF0: do_branch((cpu.status & FLAG_Z));  break; 

        case 0x4C: cpu.pc = addr_abs(); break; 
        case 0x6C: cpu.pc = addr_ind(); break; 
        case 0x20: { 
            uint8_t target_lo = read_byte(cpu.pc++, false);
            read_byte(0x0100 + cpu.sp, false);
            push_word(cpu.pc);
            uint8_t target_hi = read_byte(cpu.pc, false);
            cpu.pc = target_lo | (target_hi << 8);
            break; 
        } 
        case 0x60: 
            read_byte(cpu.pc, false);
            read_byte(0x0100 + cpu.sp, false);
            cpu.pc = pop_word();
            read_byte(cpu.pc++, false);
            break; 
        case 0x40: 
            read_byte(cpu.pc, false);
            read_byte(0x0100 + cpu.sp, false);
            cpu.status = (pop_byte() & ~FLAG_B) | FLAG_U;
            cpu.pc = pop_word();
            break; 

        case 0x18: read_byte(cpu.pc, false); set_flag(FLAG_C, false); break; 
        case 0x38: read_byte(cpu.pc, false); set_flag(FLAG_C, true);  break; 
        case 0x58: read_byte(cpu.pc, false); set_flag(FLAG_I, false); break; 
        case 0x78: read_byte(cpu.pc, false); set_flag(FLAG_I, true);  break; 
        case 0xB8: read_byte(cpu.pc, false); set_flag(FLAG_V, false); break; 
        case 0xD8: read_byte(cpu.pc, false); set_flag(FLAG_D, false); break; 
        case 0xF8: read_byte(cpu.pc, false); set_flag(FLAG_D, true);  break; 

        case 0xEA: case 0x1A: case 0x3A: case 0x5A: case 0x7A: case 0xDA: case 0xFA: 
            read_byte(cpu.pc, false); 
            break; 

        case 0x00: 
            read_byte(cpu.pc++, false);
            push_word(cpu.pc);
            push_byte(cpu.status | FLAG_B | FLAG_U);
            cpu.status |= FLAG_I;
            cpu.pc = read_word(0xFFFE);
            break; 
            
        case 0x24: case 0x2C: { 
            val = read_byte((opcode == 0x24) ? addr_zp() : addr_abs(), false);
            set_flag(FLAG_Z, (cpu.a & val) == 0); set_flag(FLAG_N, val & 0x80); set_flag(FLAG_V, val & 0x40); break;
        }

        case 0xA7: val = read_byte(addr_zp(), false);  cpu.a = val; cpu.x = val; update_zn(val); break; 
        case 0xB7: val = read_byte(addr_zpy(), false); cpu.a = val; cpu.x = val; update_zn(val); break;
        case 0xAF: val = read_byte(addr_abs(), false); cpu.a = val; cpu.x = val; update_zn(val); break;
        case 0xBF: val = read_byte(addr_aby(), false); cpu.a = val; cpu.x = val; update_zn(val); break;
        case 0xA3: val = read_byte(addr_izx(), false); cpu.a = val; cpu.x = val; update_zn(val); break;
        case 0xB3: val = read_byte(addr_izy(), false); cpu.a = val; cpu.x = val; update_zn(val); break;

        case 0x87: write_byte(addr_zp(), cpu.a & cpu.x); break; 
        case 0x97: write_byte(addr_zpy(), cpu.a & cpu.x); break;
        case 0x8F: write_byte(addr_abs(), cpu.a & cpu.x); break;
        case 0x83: write_byte(addr_izx(), cpu.a & cpu.x); break;

        case 0xC7: addr = addr_zp();  val = read_byte(addr, false); write_byte(addr, val); val--; write_byte(addr, val); do_cmp(cpu.a, val); break; 
        case 0xD7: addr = addr_zpx(); val = read_byte(addr, false); write_byte(addr, val); val--; write_byte(addr, val); do_cmp(cpu.a, val); break;
        case 0xCF: addr = addr_abs(); val = read_byte(addr, false); write_byte(addr, val); val--; write_byte(addr, val); do_cmp(cpu.a, val); break;
        case 0xDF: addr = addr_abx_store(); val = read_byte(addr, false); write_byte(addr, val); val--; write_byte(addr, val); do_cmp(cpu.a, val); break;
        case 0xDB: addr = addr_aby_store(); val = read_byte(addr, false); write_byte(addr, val); val--; write_byte(addr, val); do_cmp(cpu.a, val); break;
        case 0xC3: addr = addr_izx(); val = read_byte(addr, false); write_byte(addr, val); val--; write_byte(addr, val); do_cmp(cpu.a, val); break;
        case 0xD3: addr = addr_izy_store(); val = read_byte(addr, false); write_byte(addr, val); val--; write_byte(addr, val); do_cmp(cpu.a, val); break;

        case 0xE7: addr = addr_zp();  val = read_byte(addr, false); write_byte(addr, val); val++; write_byte(addr, val); do_sbc(val); break; 
        case 0xF7: addr = addr_zpx(); val = read_byte(addr, false); write_byte(addr, val); val++; write_byte(addr, val); do_sbc(val); break;
        case 0xEF: addr = addr_abs(); val = read_byte(addr, false); write_byte(addr, val); val++; write_byte(addr, val); do_sbc(val); break;
        case 0xFF: addr = addr_abx_store(); val = read_byte(addr, false); write_byte(addr, val); val++; write_byte(addr, val); do_sbc(val); break;
        case 0xFB: addr = addr_aby_store(); val = read_byte(addr, false); write_byte(addr, val); val++; write_byte(addr, val); do_sbc(val); break;
        case 0xE3: addr = addr_izx(); val = read_byte(addr, false); write_byte(addr, val); val++; write_byte(addr, val); do_sbc(val); break;
        case 0xF3: addr = addr_izy_store(); val = read_byte(addr, false); write_byte(addr, val); val++; write_byte(addr, val); do_sbc(val); break;

        case 0x07: addr = addr_zp();  val = read_byte(addr, false); write_byte(addr, val); set_flag(FLAG_C, val & 0x80); val <<= 1; write_byte(addr, val); cpu.a |= val; update_zn(cpu.a); break; 
        case 0x17: addr = addr_zpx(); val = read_byte(addr, false); write_byte(addr, val); set_flag(FLAG_C, val & 0x80); val <<= 1; write_byte(addr, val); cpu.a |= val; update_zn(cpu.a); break;
        case 0x0F: addr = addr_abs(); val = read_byte(addr, false); write_byte(addr, val); set_flag(FLAG_C, val & 0x80); val <<= 1; write_byte(addr, val); cpu.a |= val; update_zn(cpu.a); break;
        case 0x1F: addr = addr_abx_store(); val = read_byte(addr, false); write_byte(addr, val); set_flag(FLAG_C, val & 0x80); val <<= 1; write_byte(addr, val); cpu.a |= val; update_zn(cpu.a); break;
        case 0x1B: addr = addr_aby_store(); val = read_byte(addr, false); write_byte(addr, val); set_flag(FLAG_C, val & 0x80); val <<= 1; write_byte(addr, val); cpu.a |= val; update_zn(cpu.a); break;
        case 0x03: addr = addr_izx(); val = read_byte(addr, false); write_byte(addr, val); set_flag(FLAG_C, val & 0x80); val <<= 1; write_byte(addr, val); cpu.a |= val; update_zn(cpu.a); break;
        case 0x13: addr = addr_izy_store(); val = read_byte(addr, false); write_byte(addr, val); set_flag(FLAG_C, val & 0x80); val <<= 1; write_byte(addr, val); cpu.a |= val; update_zn(cpu.a); break;

        case 0x27: case 0x37: case 0x2F: case 0x3F: case 0x3B: case 0x23: case 0x33: { 
            addr = (opcode == 0x27) ? addr_zp() : (opcode == 0x37) ? addr_zpx() : (opcode == 0x2F) ? addr_abs() : (opcode == 0x3F) ? addr_abx_store() : (opcode == 0x3B) ? addr_aby_store() : (opcode == 0x23) ? addr_izx() : addr_izy_store();
            val = read_byte(addr, false); write_byte(addr, val); uint8_t c = (cpu.status & FLAG_C) ? 1 : 0;
            set_flag(FLAG_C, val & 0x80); val = (val << 1) | c; write_byte(addr, val);
            cpu.a &= val; update_zn(cpu.a); break;
        }

        case 0x47: case 0x57: case 0x4F: case 0x5F: case 0x5B: case 0x43: case 0x53: { 
            addr = (opcode == 0x47) ? addr_zp() : (opcode == 0x57) ? addr_zpx() : (opcode == 0x4F) ? addr_abs() : (opcode == 0x5F) ? addr_abx_store() : (opcode == 0x5B) ? addr_aby_store() : (opcode == 0x43) ? addr_izx() : addr_izy_store();
            val = read_byte(addr, false); write_byte(addr, val); set_flag(FLAG_C, val & 0x01); val >>= 1; write_byte(addr, val);
            cpu.a ^= val; update_zn(cpu.a); break;
        }

        case 0x67: case 0x77: case 0x6F: case 0x7F: case 0x7B: case 0x63: case 0x73: { 
            addr = (opcode == 0x67) ? addr_zp() : (opcode == 0x77) ? addr_zpx() : (opcode == 0x6F) ? addr_abs() : (opcode == 0x7F) ? addr_abx_store() : (opcode == 0x7B) ? addr_aby_store() : (opcode == 0x63) ? addr_izx() : addr_izy_store();
            val = read_byte(addr, false); write_byte(addr, val); uint8_t c = (cpu.status & FLAG_C) ? 0x80 : 0;
            set_flag(FLAG_C, val & 0x01); val = (val >> 1) | c; update_zn(val);
            do_adc(val); break;
        }

        case 0xCB: { 
            val = read_byte(addr_imm(), false);
            uint8_t temp = (cpu.a & cpu.x);
            set_flag(FLAG_C, temp >= val);
            cpu.x = temp - val;
            update_zn(cpu.x);
            break;
        }

        case 0x0B: case 0x2B: 
            cpu.a &= read_byte(addr_imm(), false);
            update_zn(cpu.a);
            set_flag(FLAG_C, (cpu.a & 0x80) != 0);
            break;

        case 0x4B: 
            cpu.a &= read_byte(addr_imm(), false);
            set_flag(FLAG_C, cpu.a & 0x01);
            cpu.a >>= 1;
            update_zn(cpu.a);
            break;

        case 0x6B: { 
            cpu.a &= read_byte(addr_imm(), false);
            uint8_t c = (cpu.status & FLAG_C) ? 0x80 : 0;
            cpu.a = (cpu.a >> 1) | c;
            update_zn(cpu.a);
            set_flag(FLAG_C, (cpu.a & 0x40) != 0);
            set_flag(FLAG_V, ((cpu.a & 0x40) ^ ((cpu.a & 0x20) << 1)) != 0);
            break;
        }

        case 0x80: case 0x82: case 0x89: case 0xC2: case 0xE2:
            read_byte(addr_imm(), false);
            break;

        case 0x04: case 0x44: case 0x64:
            read_byte(addr_zp(), false);
            break;

        case 0x14: case 0x34: case 0x54: case 0x74: case 0xD4: case 0xF4:
            read_byte(addr_zpx(), false);
            break;

        case 0x0C:
            read_byte(addr_abs(), false);
            break;

        case 0x1C: case 0x3C: case 0x5C: case 0x7C: case 0xDC: case 0xFC:
            read_byte(addr_abx(), false);
            break;
        
        default:
            while (1) {
#if defined(__arm__) && defined(__SAM3X8E__)
                if (READ_PIN_RES()) {
                    reset_pending = true;
                    break;
                }
#endif
                read_byte((uint16_t)(cpu.pc - 1), false);
            }
            break; 
    }

    return cycle_counter;
}