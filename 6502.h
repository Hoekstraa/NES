#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

typedef struct
{
    // Starts at 0x200 on normal 6502, 0x600 on the NES
    uint16_t programcounter;
    // Accumulator register
    uint8_t acc;
    // X register
    uint8_t xr;
    // Y register
    uint8_t yr;
    // flags, as a bitfield;
    uint8_t nflag : 1, vflag : 1, bflag : 1, bflag2: 1, dflag : 1, iflag : 1, zflag : 1, cflag : 1;
    // Maps to 0x100 to 0x1FF e.g. offset by +0x100
    uint8_t stackreg;
    // Keeps track of how many cycles the virtual CPU has ran.
    unsigned long long cycle;
    // Pointer to a block of RAM the CPU can use;
    uint8_t* ram;
    // Size of ram is not known due to it being stored as a pointer, so it's kept seperately;
    int ram_size;
} cpu;


////////////////////////////////////////
// Helper functions

void print_stack(cpu* c, int amount)
{
    for (int i = 0xFF; i > (0xFF - amount); i--)
    {
        const char* stackpointer = c->stackreg == i ? "<--" : "";
        printf("%.2X %.2X %s\n", i, c->ram[0x100 + i], stackpointer);
    }
}

cpu make_cpu()
{
    int ram_size = 0x10000;
    uint8_t* ram = (uint8_t*)malloc(ram_size);

    cpu c =
    {
        0x600, // pc
        0, // acc
        0, // xr
        0, // yr
        0,
        0,
        1,
        1,
        0,
        1,
        0,
        0, // flags
        0xFD, //stackreg
        0, // cycles
        ram, // ram pointer
        ram_size // ram size
    };
    return c;
}

static uint8_t status(const cpu* c)
{
    return ((uint8_t)c->nflag << 7)
           + ((uint8_t)c->vflag << 6)
           + ((uint8_t)c->bflag << 5)
           + ((uint8_t)c->bflag2 << 4)
           + ((uint8_t)c->dflag << 3)
           + ((uint8_t)c->iflag << 2)
           + ((uint8_t)c->zflag << 1)
           + c->cflag;
}

void setstatus(cpu* c, uint8_t status)
{
    c->nflag = (status >> 7) % 2;
    c->vflag = (status >> 6) % 2;
    c->bflag = (status >> 5) % 2;
    c->bflag2 = (status >> 4) % 2;
    c->dflag = (status >> 3) % 2;
    c->iflag = (status >> 2) % 2;
    c->zflag = (status >> 1) % 2;
    c->cflag = status % 2;
}

/// Print the CPU registers in human-readable form.
void print_cpu_state(const cpu* c)
{
    printf("-------\n");
    printf("PC: %.4x\n", c->programcounter);
    printf("Future: %.2x %.2x %.2x %.2x %.2x\n", c->ram[c->programcounter], c->ram[c->programcounter + 1], c->ram[c->programcounter + 2], c->ram[c->programcounter + 3], c->ram[c->programcounter + 4]);
    printf("A: %.2x\n", c->acc);
    printf("X: %.2x\n", c->xr);
    printf("Y: %.2x\n", c->yr);
    printf("Stack reg: %.2x\n", c->stackreg);
    printf("N flag: %x\n", c->nflag);
    printf("V flag: %x\n", c->vflag);
    printf("B flag: %x\n", c->bflag);
    printf("D flag: %x\n", c->dflag);
    printf("I flag: %x\n", c->iflag);
    printf("Z flag: %x\n", c->zflag);
    printf("C flag: %x\n", c->cflag);
    printf("Cycle: %llu\n", c->cycle);
    printf("-------\n");
}

void log_cpu(const cpu* c, FILE* f)
{
    printf("%.4X  %2.2X A:%.2X X:%.2X Y:%.2X P:%.2X SP:%.2X\n",
           c->programcounter,
           c->ram[c->programcounter],
           //c->ram[1 + c->programcounter],
           //c->ram[2 + c->programcounter],
           c->acc,
           c->xr,
           c->yr,
           status(c),
           c->stackreg);
}

static inline void unimplemented(uint8_t op)
{
    printf("Op %x is unimplemented.\n", op);
    fflush(stdout);
    int stop_on_unimplemented = 1;
    if (stop_on_unimplemented == 1)
        exit(99);
}

static inline void add_cycles(cpu* c, int cycles)
{
    c->cycle += cycles;
}

static inline void debug(cpu* c, uint8_t op, uint8_t a, uint8_t b)
{
    printf("%x  %x %x %x\t\tA:%x", c->programcounter, op, a, b, c->acc);
}

////////////////////////////////////////
// Adressing modes

static inline uint8_t absolute(cpu* c, uint16_t address)
{
    return c->ram[address];
}

static inline uint8_t zeropage(cpu* c, uint8_t address)
{
    return c->ram[address];
}

static inline uint8_t zeropageX_address(cpu* c, uint8_t address)
{
    uint8_t zero_address = address + c->xr;
    return zero_address;
}

static inline uint8_t zeropageX(cpu* c, uint8_t address)
{
    uint8_t zero_address = address + c->xr;
    return c->ram[zero_address];
}

static inline uint8_t zeropageY_address(cpu* c, uint8_t address)
{
    uint8_t zero_address = address + c->yr;
    return zero_address;
}

static inline uint8_t zeropageY(cpu* c, uint8_t address)
{
    uint8_t zero_address = address + c->yr;
    return c->ram[zero_address];
}

static uint16_t indirectX_address(cpu* c, uint8_t address)
{
    uint8_t zero_address = address + c->xr;
    uint16_t indirect_address = c->ram[(uint8_t)(zero_address + 1)];
    indirect_address = (indirect_address << 8) + c->ram[zero_address];
    return indirect_address;
}

static inline uint8_t indirectX(cpu* c, uint8_t address)
{
    return c->ram[indirectX_address(c, address)];
}

static uint16_t indirectY_address(cpu* c, uint8_t address)
{
    uint16_t indirect_address = c->ram[(uint8_t)(address + 1)];
    indirect_address = (indirect_address << 8) + c->ram[address] + c->yr;
    return indirect_address;
}

static inline uint8_t indirectY(cpu* c, uint8_t address)
{
    return c->ram[indirectY_address(c, address)];
}

static uint16_t indirect(cpu* c, uint16_t address)
{
    // Hardware bug in OG 6502
    uint16_t high = address & 0xFF00;
    uint8_t low = address;
    ++low;
    uint16_t msb_address = high + low;

    // Now for the actual logic;
    uint16_t indirect_address = c->ram[msb_address];
    indirect_address <<= 8;
    indirect_address += c->ram[address];
    return indirect_address;
}


////////////////////////////////////////
// Instructions

static void ADC(cpu* c, uint8_t value)
{
    // Keep the original value, so we can check later whether we need to carry.
    uint8_t const original_acc = c->acc;

    // Add value and carry to accumulator
    c->acc += value + c->cflag;

    // If acc < original_acc, non-signed overflow occurred, so we need to carry.
    c->cflag = c->acc < original_acc;
    // If accumulator is 0, z flag should turn on, otherwise off.
    c->zflag = c->acc == 0;
    // If the sign changed, the overflow should turn on, otherwise off.
    // Taken from  https://forums.nesdev.com/viewtopic.php?f=3&t=6331
    c->vflag = !!((original_acc ^ c->acc) & (value ^ c->acc) & (uint8_t)0x80);
    // If the sign is set, the negative flag should be set, otherwise unset.
    c->nflag = c->acc >> 7;
}

static void AND(cpu* c, uint8_t value)
{
    // bitwise and value to acc
    c->acc &= value;

    // If accumulator is 0, z flag should turn on, otherwise off.
    c->zflag = c->acc == 0;
    // If the sign is set, the negative flag should be set, otherwise unset.
    c->nflag = c->acc >> 7;
}

static void ASL_acc(cpu* c)
{
    // Set carry to left-most bit
    c->cflag = c->acc >> 7;

    // Shift left
    c->acc <<= 1;

    // If accumulator is 0, z flag should turn on, otherwise off.
    c->zflag = c->acc == 0;
    // If the sign is set, the negative flag should be set, otherwise unset.
    c->nflag = c->acc >> 7;
}

static void ASL(cpu* c, uint16_t address)
{
    // Set carry to left-most bit
    c->cflag = c->ram[address] >> 7;

    // Shift left
    c->ram[address] <<= 1;

    // If value is 0, z flag should turn on, otherwise off.
    c->zflag = c->ram[address] == 0;
    // If the sign is set, the negative flag should be set, otherwise unset.
    c->nflag = c->ram[address] >> 7;

}

// Branch if carry clear
static void BCC(cpu* c, uint8_t offset)
{
    if (c->cflag) return;
    c->programcounter += offset;
}

// Branch if carry set
static void BCS(cpu* c, uint8_t offset)
{
    if (!c->cflag) return;
    c->programcounter += offset;
}

// Branch if equal (zero set)
static void BEQ(cpu* c, uint8_t offset)
{
    if (!c->zflag) return;
    c->programcounter += offset;
}

static void BIT(cpu* c, uint8_t value)
{
    uint8_t test = c->acc & value;

    // Set zero flag if and of acc and value is zero.
    c->zflag = test == 0;
    // Set overflow to bit 6
    c->vflag = (value >> 6) % 2;
    // Set negative to bit 7
    c->nflag = value >> 7;
}

// Branch if minus
static void BMI(cpu* c, uint8_t offset)
{
    if (!c->nflag) return;
    c->programcounter += offset;
}

// Branch if not equal (zero unset)
static void BNE(cpu* c, uint8_t offset)
{
    if (c->zflag) return;
    c->programcounter += offset;
}

// Branch if plus
static void BPL(cpu* c, uint8_t offset)
{
    if (c->nflag) return;
    c->programcounter += offset;
}

static void BRK(cpu* c)
{
    c->bflag = 1;
    unimplemented(0x00);
}

// Branch if overflow clear
static void BVC(cpu* c, uint8_t offset)
{
    if (c->vflag) return;
    c->programcounter += offset;
}

// Branch if overflow set
static void BVS(cpu* c, uint8_t offset)
{
    if (!c->vflag) return;
    c->programcounter += offset;
}

// Clear Carry Flag
static void CLC(cpu* c)
{
    c->cflag = 0;
}

// Clear Decimal Mode Flag
static void CLD(cpu* c)
{
    c->dflag = 0;
}

// Clear Interrupt Flag
static void CLI(cpu* c)
{
    c->iflag = 0;
}

// Clear Overflow Flag
static void CLV(cpu* c)
{
    c->vflag = 0;
}

static void CMP(cpu* c, uint8_t value)
{
    c->cflag = c->acc >= value;
    c->zflag = c->acc == value;
    c->nflag = (c->acc - value) >> 7;
}

static void CPX(cpu* c, uint8_t value)
{
    c->cflag = c->xr >= value;
    c->zflag = c->xr == value;
    c->nflag = (c->xr - value) >> 7;
}

static void CPY(cpu* c, uint8_t value)
{
    c->cflag = c->yr >= value;
    c->zflag = c->yr == value;
    c->nflag = (c->yr - value) >> 7;
}

static void DEC(cpu* c, uint16_t address)
{
    --(c->ram[address]);
    // If value is 0, z flag should turn on, otherwise off.
    c->zflag = c->ram[address] == 0;
    // If the sign is set, the negative flag should be set, otherwise unset.
    c->nflag = c->ram[address] >> 7;
}

static void DEX(cpu* c)
{
    --(c->xr);
    // If value is 0, z flag should turn on, otherwise off.
    c->zflag = c->xr == 0;
    // If the sign is set, the negative flag should be set, otherwise unset.
    c->nflag = c->xr >> 7;
}

static void DEY(cpu* c)
{
    --(c->yr);
    // If value is 0, z flag should turn on, otherwise off.
    c->zflag = c->yr == 0;
    // If the sign is set, the negative flag should be set, otherwise unset.
    c->nflag = c->yr >> 7;
}

static void EOR(cpu* c, uint8_t value)
{
    c->acc ^= value;

    // If value is 0, z flag should turn on, otherwise off.
    c->zflag = c->acc == 0;
    // If the sign is set, the negative flag should be set, otherwise unset.
    c->nflag = c->acc >> 7;
}

static void INC(cpu* c, uint16_t address)
{
    ++(c->ram[address]);
    // If value is 0, z flag should turn on, otherwise off.
    c->zflag = c->ram[address] == 0;
    // If the sign is set, the negative flag should be set, otherwise unset.
    c->nflag = c->ram[address] >> 7;
}

static void INX(cpu* c)
{
    ++(c->xr);
    // If value is 0, z flag should turn on, otherwise off.
    c->zflag = c->xr == 0;
    // If the sign is set, the negative flag should be set, otherwise unset.
    c->nflag = c->xr >> 7;
}

static void INY(cpu* c)
{
    ++(c->yr);
    // If value is 0, z flag should turn on, otherwise off.
    c->zflag = c->yr == 0;
    // If the sign is set, the negative flag should be set, otherwise unset.
    c->nflag = c->yr >> 7;
}

static void JMP(cpu* c, uint16_t address)
{
    c->programcounter = address;
}

static void JSR(cpu* c, uint16_t address)
{
    // Adjust for interesting behaviour
    c->programcounter -= 1;

    // Send that PC to the stack
    uint8_t high_byte = c->programcounter >> 8;
    c->ram[0x100 + c->stackreg] = high_byte;
    --(c->stackreg);
    uint8_t low_byte = c->programcounter;
    c->ram[0x100 + c->stackreg] = low_byte;
    --(c->stackreg);

    c->programcounter = address;
}

static void LDA(cpu* c, uint8_t value)
{
    c->acc = value;

    // If value is 0, z flag should turn on, otherwise off.
    c->zflag = value == 0;
    // If the sign is set, the negative flag should be set, otherwise unset.
    c->nflag = value >> 7;
}

static void LDX(cpu* c, uint8_t value)
{
    c->xr = value;

    // If value is 0, z flag should turn on, otherwise off.
    c->zflag = value == 0;
    // If the sign is set, the negative flag should be set, otherwise unset.
    c->nflag = value >> 7;
}

static void LDY(cpu* c, uint8_t value)
{
    c->yr = value;

    // If value is 0, z flag should turn on, otherwise off.
    c->zflag = value == 0;
    // If the sign is set, the negative flag should be set, otherwise unset.
    c->nflag = value >> 7;
}

static void LSR_acc(cpu* c)
{
    // Set carry to right-most bit
    c->cflag = c->acc & 1;

    // Shift right
    c->acc >>= 1;

    // If accumulator is 0, z flag should turn on, otherwise off.
    c->zflag = c->acc == 0;
    // If the sign is set, the negative flag should be set, otherwise unset.
    c->nflag = c->acc >> 7;
}

static void LSR(cpu* c, uint16_t address)
{
    // Set carry to right-most bit
    c->cflag = c->ram[address] & 1;

    // Shift right
    c->ram[address] >>= 1;

    // If value is 0, z flag should turn on, otherwise off.
    c->zflag = c->ram[address] == 0;
    // If the sign is set, the negative flag should be set, otherwise unset.
    c->nflag = c->ram[address] >> 7;
}

static void NOP(cpu* c)
{

}

static void ORA(cpu* c, uint8_t value)
{
    c->acc |= value;

    // If accumulator is 0, z flag should turn on, otherwise off.
    c->zflag = c->acc == 0;
    // If the sign is set, the negative flag should be set, otherwise unset.
    c->nflag = c->acc >> 7;
}

// Push a copy of acc onto the stack
static void PHA(cpu* c)
{
    c->ram[0x100 + c->stackreg] = c->acc;
    --(c->stackreg);
}

// Push a copy of the status flags onto the stack
static void PHP(cpu* c)
{
    c->ram[0x100 + c->stackreg] = ((uint8_t)c->nflag << 7)
                                  + ((uint8_t)c->vflag << 6)
                                  + ((uint8_t)1 << 5)
                                  + ((uint8_t)1 << 4)
                                  + ((uint8_t)c->dflag << 3)
                                  + ((uint8_t)c->iflag << 2)
                                  + ((uint8_t)c->zflag << 1)
                                  + c->cflag;
    --(c->stackreg);
}

static void PLA(cpu* c)
{
    ++(c->stackreg);
    c->acc = c->ram[0x100 + c->stackreg];

    // If accumulator is 0, z flag should turn on, otherwise off.
    c->zflag = c->acc == 0;
    // If the sign is set, the negative flag should be set, otherwise unset.
    c->nflag = c->acc >> 7;
}

static void PLP(cpu* c)
{
    ++(c->stackreg);
    uint8_t status = c->ram[0x100 + c->stackreg];
    c->nflag = (status >> 7) % 2;
    c->vflag = (status >> 6) % 2;
    c->dflag = (status >> 3) % 2;
    c->iflag = (status >> 2) % 2;
    c->zflag = (status >> 1) % 2;
    c->cflag = status      % 2;
}

static void ROL_acc(cpu* c)
{
    uint8_t bit7 = c->acc >> 7;
    c->acc <<= 1;
    c->acc += c->cflag;
    c->cflag = bit7;

    // If accumulator is 0, z flag should turn on, otherwise off.
    c->zflag = c->acc == 0;
    // If the sign is set, the negative flag should be set, otherwise unset.
    c->nflag = c->acc >> 7;
}

static void ROL(cpu* c, uint16_t address)
{
    uint8_t bit7 = c->ram[address] >> 7;
    c->ram[address] <<= 1;
    c->ram[address] += c->cflag;
    c->cflag = bit7;

    // If accumulator is 0, z flag should turn on, otherwise off.
    c->zflag = c->ram[address] == 0;
    // If the sign is set, the negative flag should be set, otherwise unset.
    c->nflag = c->ram[address] >> 7;
}

static void ROR_acc(cpu* c)
{
    uint8_t bit0 = c->acc % 2;
    c->acc >>= 1;
    c->acc += (uint8_t)c->cflag << 7;
    c->cflag = bit0;

    // If accumulator is 0, z flag should turn on, otherwise off.
    c->zflag = c->acc == 0;
    // If the sign is set, the negative flag should be set, otherwise unset.
    c->nflag = c->acc >> 7;
}

static void ROR(cpu* c, uint16_t address)
{
    uint8_t bit0 = c->ram[address] % 2;
    c->ram[address] >>= 1;
    c->ram[address] += (uint8_t)c->cflag << 7;
    c->cflag = bit0;

    // If accumulator is 0, z flag should turn on, otherwise off.
    c->zflag = c->ram[address] == 0;
    // If the sign is set, the negative flag should be set, otherwise unset.
    c->nflag = c->ram[address] >> 7;
}

// Return from interrupt: load status and PC from stack.
static void RTI(cpu* c)
{

    uint8_t status = c->ram[0x100 + ++(c->stackreg)];
    c->nflag = (status >> 7) % 2;
    c->vflag = (status >> 6) % 2;
    c->dflag = (status >> 3) % 2;
    c->iflag = (status >> 2) % 2;
    c->zflag = (status >> 1) % 2;
    c->cflag = status % 2;

    uint8_t low_byte = c->ram[0x100 + ++(c->stackreg)];
    uint8_t high_byte = c->ram[0x100 + ++(c->stackreg)];
    uint16_t address = (high_byte << 8) + low_byte;
    c->programcounter = address;
}

static void RTS(cpu* c)
{
    uint8_t low_byte = c->ram[0x100 + ++(c->stackreg)];
    uint8_t high_byte = c->ram[0x100 + ++(c->stackreg)];
    uint16_t address = (high_byte << 8) + low_byte;
    // Adjusts for interesting behaviour in JSR
    c->programcounter = address + 1;
}

static void SBC(cpu* c, uint8_t value)
{
    // Nugget from https://stackoverflow.com/questions/29193303/6502-emulation-proper-way-to-implement-adc-and-sbc
    ADC(c, ~value);
}

static void SEC(cpu* c)
{
    c->cflag = 1;
}

static void SED(cpu* c)
{
    c->dflag = 1;
}

static void SEI(cpu* c)
{
    c->iflag = 1;
}

static void STA(cpu* c, uint16_t address)
{
    c->ram[address] = c->acc;
}

static void STX(cpu* c, uint16_t address)
{
    c->ram[address] = c->xr;
}

static void STY(cpu* c, uint16_t address)
{
    c->ram[address] = c->yr;
}

static void TAX(cpu* c)
{
    c->xr = c->acc;

    // If accumulator is 0, z flag should turn on, otherwise off.
    c->zflag = c->xr == 0;
    // If the sign is set, the negative flag should be set, otherwise unset.
    c->nflag = c->xr >> 7;
}

static void TAY(cpu* c)
{
    c->yr = c->acc;

    // If accumulator is 0, z flag should turn on, otherwise off.
    c->zflag = c->yr == 0;
    // If the sign is set, the negative flag should be set, otherwise unset.
    c->nflag = c->yr >> 7;
}

static void TSX(cpu* c)
{
    c->xr = c->stackreg;

    // If accumulator is 0, z flag should turn on, otherwise off.
    c->zflag = c->xr == 0;
    // If the sign is set, the negative flag should be set, otherwise unset.
    c->nflag = c->xr >> 7;
}

static void TXA(cpu* c)
{
    c->acc = c->xr;

    // If accumulator is 0, z flag should turn on, otherwise off.
    c->zflag = c->xr == 0;
    // If the sign is set, the negative flag should be set, otherwise unset.
    c->nflag = c->xr >> 7;
}

static void TXS(cpu* c)
{
    c->stackreg = c->xr;
}

static void TYA(cpu* c)
{
    c->acc = c->yr;

    // If accumulator is 0, z flag should turn on, otherwise off.
    c->zflag = c->yr == 0;
    // If the sign is set, the negative flag should be set, otherwise unset.
    c->nflag = c->yr >> 7;
}

// Fetch a byte, adjusts the programcounter
static uint8_t fetch(cpu* c)
{
    uint8_t byte = c->ram[c->programcounter];
    c->programcounter += 1;
    return byte;
}

/// Fetches 2 bytes and merges them into a word
static uint16_t fetch2(cpu* c)
{
    uint16_t a = fetch(c);
    a += (uint16_t)fetch(c) << 8;
    return a;
}

static void decode(cpu* c, uint8_t op)
{
    switch (op)
    {
        case (0x00):
            BRK(c);
            break;
        case (0x01):
            ORA(c, indirectX(c, fetch(c)));
            break;
        case (0x05):
            ORA(c, zeropage(c, fetch(c)));
            break;
        case (0x06):
            ASL(c, fetch(c));
            break;
        case (0x08):
            PHP(c);
            break;
        case (0x09):
            ORA(c, fetch(c));
            break;
        case (0x0A):
            ASL_acc(c);
            break;
        case (0x0D):
            ORA(c, absolute(c, fetch2(c)));
            break;
        case (0x0E):
            ASL(c, fetch2(c));
            break;
        case (0x10):
            BPL(c, fetch(c));
            break;
        case (0x11):
            ORA(c, indirectY(c, fetch(c)));
            break;
        case (0x15):
            ORA(c, zeropageX(c, fetch(c)));
            break;
        case (0x16):
            ASL(c, zeropageX_address(c, fetch(c)));
            break;
        case (0x18):
            CLC(c);
            break;
        case (0x19):
            ORA(c, absolute(c, fetch2(c) + c->yr));
            break;
        case (0x1D):
            ORA(c, absolute(c, fetch2(c) + c->xr));
            break;
        case (0x1E):
            ASL(c, fetch2(c) + c->xr);
            break;
        case (0x20):
            JSR(c, fetch2(c));
            break;
        case (0x21):
            AND(c, indirectX(c, fetch(c)));
            break;
        case (0x24):
            BIT(c, zeropage(c, fetch(c)));
            break;
        case (0x25):
            AND(c, zeropage(c, fetch(c)));
            break;
        case (0x26):
            ROL(c, fetch(c));
            break;
        case (0x28):
            PLP(c);
            break;
        case (0x29):
            AND(c, fetch(c));
            break;
        case (0x2A):
            ROL_acc(c);
            break;
        case (0x2C):
            BIT(c, absolute(c, fetch2(c)));
            break;
        case (0x2D):
            AND(c, absolute(c, fetch2(c)));
            break;
        case (0x2E):
            ROL(c, fetch2(c));
            break;
        case (0x30):
            BMI(c, fetch(c));
            break;
        case (0x31):
            AND(c, indirectY(c, fetch(c)));
            break;
        case (0x35):
            AND(c, zeropageX(c, fetch(c)));
            break;
        case (0x36):
            ROL(c, zeropageX_address(c, fetch(c)));
            break;
        case (0x38):
            SEC(c);
            break;
        case (0x39):
            AND(c, absolute(c, fetch2(c) + c->yr));
            break;
        case (0x3D):
            AND(c, absolute(c, fetch2(c) + c->xr));
            break;
        case (0x3E):
            ROL(c, fetch2(c) + c->xr);
            break;
        case (0x40):
            RTI(c);
            break;
        case (0x41):
            EOR(c, indirectX(c, fetch(c)));
            break;
        case (0x45):
            EOR(c, zeropage(c, fetch(c)));
            break;
        case (0x46):
            LSR(c, fetch(c));
            break;
        case (0x48):
            PHA(c);
            break;
        case (0x49):
            EOR(c, fetch(c));
            break;
        case (0x4A):
            LSR_acc(c);
            break;
        case (0x4C):
            JMP(c, fetch2(c));
            break;
        case (0x4D):
            EOR(c, absolute(c, fetch2(c)));
            break;
        case (0x4E):
            LSR(c, fetch2(c));
            break;
        case (0x50):
            BVC(c, fetch(c));
            break;
        case (0x51):
            EOR(c, indirectY(c, fetch(c)));
            break;
        case (0x55):
            EOR(c, zeropageX(c, fetch(c)));
            break;
        case (0x56):
            LSR(c, zeropageX_address(c, fetch(c)));
            break;
        case (0x59):
            EOR(c, absolute(c, fetch2(c) + c->yr));
            break;
        case (0x5D):
            EOR(c, absolute(c, fetch2(c) + c->xr));
            break;
        case (0x5E):
            LSR(c, fetch2(c) + c->xr);
            break;
        case (0x60):
            RTS(c);
            break;
        case (0x61):
            ADC(c, indirectX(c, fetch(c)));
            break;
        case (0x65):
            ADC(c, zeropage(c, fetch(c)));
            break;
        case (0x66):
            ROR(c, fetch(c));
            break;
        case (0x68):
            PLA(c);
            break;
        case (0x69):
            ADC(c, fetch(c));
            break;
        case (0x6A):
            ROR_acc(c);
            break;
        case (0x6C):
            JMP(c, indirect(c, fetch2(c)));
            break;
        case (0x6D):
            ADC(c, absolute(c, fetch2(c)));
            break;
        case (0x6E):
            ROR(c, fetch2(c));
            break;
        case (0x70):
            BVS(c, fetch(c));
            break;
        case (0x71):
            ADC(c, indirectY(c, fetch(c)));
            break;
        case (0x75):
            ADC(c, zeropageX(c, fetch(c)));
            break;
        case (0x76):
            ROR(c, zeropageX_address(c, fetch(c)));
            break;
        case (0x78):
            SEI(c);
            break;
        case (0x79):
            ADC(c, absolute(c, fetch2(c) + c->yr));
            break;
        case (0x7D):
            ADC(c, absolute(c, fetch2(c) + c->xr));
            break;
        case (0x7E):
            ROR(c, fetch2(c) + c->xr);
            break;
        case (0x81):
            STA(c, indirectX_address(c, fetch(c)));
            break;
        case (0x84):
            STY(c, fetch(c));
            break;
        case (0x85):
            STA(c, fetch(c));
            break;
        case (0x86):
            STX(c, fetch(c));
            break;
        case (0x88):
            DEY(c);
            break;
        case (0x8A):
            TXA(c);
            break;
        case (0x8C):
            STY(c, fetch2(c));
            break;
        case (0x8D):
            STA(c, fetch2(c));
            break;
        case (0x8E):
            STX(c, fetch2(c));
            break;
        case (0x90):
            BCC(c, fetch(c));
            break;
        case (0x91):
            STA(c, indirectY_address(c, fetch(c)));
            break;
        case (0x94):
            STY(c, zeropageX_address(c, fetch(c)));
            break;
        case (0x95):
            STA(c, zeropageX_address(c, fetch(c)));
            break;
        case (0x96):
            STX(c, zeropageY_address(c, fetch(c)));
            break;
        case (0x98):
            TYA(c);
            break;
        case (0x99):
            STA(c, fetch2(c) + c->yr);
            break;
        case (0x9A):
            TXS(c);
            break;
        case (0xA0):
            LDY(c, fetch(c));
            break;
        case (0x9D):
            STA(c, fetch2(c) + c->xr);
            break;
        case (0xA1):
            LDA(c, indirectX(c, fetch(c)));
            break;
        case (0xA2):
            LDX(c, fetch(c));
            break;
        case (0xA4):
            LDY(c, zeropage(c, fetch(c)));
            break;
        case (0xA5):
            LDA(c, zeropage(c, fetch(c)));
            break;
        case (0xA6):
            LDX(c, zeropage(c, fetch(c)));
            break;
        case (0xA8):
            TAY(c);
            break;
        case (0xA9):
            LDA(c, fetch(c));
            break;
        case (0xAA):
            TAX(c);
            break;
        case (0xAC):
            LDY(c, absolute(c, fetch2(c)));
            break;
        case (0xAD):
            LDA(c, absolute(c, fetch2(c)));
            break;
        case (0xAE):
            LDX(c, absolute(c, fetch2(c)));
            break;
        case (0xB0):
            BCS(c, fetch(c));
            break;
        case (0xB1):
            LDA(c, indirectY(c, fetch(c)));
            break;
        case (0xB4):
            LDY(c, zeropageX(c, fetch(c)));
            break;
        case (0xB5):
            LDA(c, zeropageX(c, fetch(c)));
            break;
        case (0xB6):
            LDX(c, zeropageY(c, fetch(c)));
            break;
        case (0xB8):
            CLV(c);
            break;
        case (0xB9):
            LDA(c, absolute(c, fetch2(c) + c->yr));
            break;
        case (0xBA):
            TSX(c);
            break;
        case (0xBC):
            LDY(c, absolute(c, fetch2(c) + c->xr));
            break;
        case (0xBD):
            LDA(c, absolute(c, fetch2(c) + c->xr));
            break;
        case (0xBE):
            LDX(c, absolute(c, fetch2(c) + c->yr));
            break;
        case (0xC0):
            CPY(c, fetch(c));
            break;
        case (0xC1):
            CMP(c, indirectX(c, fetch(c)));
            break;
        case (0xC4):
            CPY(c, zeropage(c, fetch(c)));
            break;
        case (0xC5):
            CMP(c, zeropage(c, fetch(c)));
            break;
        case (0xC6):
            DEC(c, fetch(c));
            break;
        case (0xC8):
            INY(c);
            break;
        case (0xC9):
            CMP(c, fetch(c));
            break;
        case (0xCA):
            DEX(c);
            break;
        case (0xCC):
            CPY(c, absolute(c, fetch2(c)));
            break;
        case (0xCD):
            CMP(c, absolute(c, fetch2(c)));
            break;
        case (0xCE):
            DEC(c, fetch2(c));
            break;
        case (0xD0):
            BNE(c, fetch(c));
            break;
        case (0xD1):
            CMP(c, indirectY(c, fetch(c)));
            break;
        case (0xD5):
            CMP(c, zeropageX(c, fetch(c)));
            break;
        case (0xD6):
            DEC(c, zeropageX_address(c, fetch(c)));
            break;
        case (0xD8):
            CLD(c);
            break;
        case (0xD9):
            CMP(c, absolute(c, fetch2(c) + c->yr));
            break;
        case (0xDD):
            CMP(c, absolute(c, fetch2(c) + c->xr));
            break;
        case (0xDE):
            DEC(c, fetch2(c) + c->xr);
            break;
        case (0xE0):
            CPX(c, fetch(c));
            break;
        case (0xE1):
            SBC(c, indirectX(c, fetch(c)));
            break;
        case (0xE4):
            CPX(c, zeropage(c, fetch(c)));
            break;
        case (0xE5):
            SBC(c, zeropage(c, fetch(c)));
            break;
        case (0xE6):
            INC(c, fetch(c));
            break;
        case (0xE8):
            INX(c);
            break;
        case (0xE9):
            SBC(c, fetch(c));
            break;
        case (0xEA):
            NOP(c);
            break;
        case (0xEC):
            CPX(c, absolute(c, fetch2(c)));
            break;
        case (0xED):
            SBC(c, absolute(c, fetch2(c)));
            break;
        case (0xEE):
            INC(c, fetch2(c));
            break;
        case (0xF0):
            BEQ(c, fetch(c));
            break;
        case (0xF1):
            SBC(c, indirectY(c, fetch(c)));
            break;
        case (0xF5):
            SBC(c, zeropageX(c, fetch(c)));
            break;
        case (0xF6):
            INC(c, zeropageX_address(c, fetch(c)));
            break;
        case (0xF8):
            SED(c);
            break;
        case (0xF9):
            SBC(c, absolute(c, fetch2(c) + c->yr));
            break;
        case (0xFD):
            SBC(c, absolute(c, fetch2(c) + c->xr));
            break;
        case (0xFE):
            INC(c, fetch2(c) + c->xr);
            break;
        default:
            unimplemented(op);
    }
}

// Execute one instruction
void execute(cpu* c)
{
    decode(c, fetch(c));
}
