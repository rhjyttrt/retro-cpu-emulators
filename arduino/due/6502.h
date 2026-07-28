#ifndef CPU6502_H
#define CPU6502_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif


typedef struct {
    uint8_t  a;      
    uint8_t  x;      
    uint8_t  y;      
    uint8_t  sp;     
    uint8_t  status; 
    uint16_t pc;     
    bool     rdy;    
    bool     sync;   
} CPU6502;

extern CPU6502 cpu;

void cpu_reset(void);
int  cpu_nmi(void);
int  cpu_irq(void);
int  cpu_step(void);

#ifdef __cplusplus
}
#endif

#endif 