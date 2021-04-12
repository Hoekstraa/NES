#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include "6502.h"

void load_into_mem(cpu* c, char* filename)
{
    FILE* f = fopen(filename, "rb");
    char data = 0;

    if (!f)
    {
        printf("Unable to open file!\n");
        exit(2);
    }

    char header[16];

    // Read the first 16 bytes as the file header.
    for (int i = 0; i < 16; i++)
    {
        header[i] = fgetc(f);
        if (feof(f))
        {
            printf("Invalid file!\n");
            exit(3);
        }
    }

    int iNESFormat = 0;
    if (header[0] == 'N' && header[1] == 'E' && header[2] == 'S' && header[3] == 0x1A)
        iNESFormat = 1;

    int NES20Format = 0;
    if (iNESFormat == 1 && (header[7] & 0x0C) == 0x08)
        NES20Format = 1;

    //printf("ROM is iNES: %s\n", iNESFormat ? "true" : "false");
    //printf("ROM is NES20: %s\n", NES20Format ? "true" : "false");
    //fflush(stdout);

    int PRGROM_size = 16384 * header[4];
    int CHR_size = 8192 * header[5];

    for (int i = 0; i < PRGROM_size ; i++)
    {
        data = fgetc(f);

        if (feof(f))
            break;

        //printf("Setting memory address block %x with %.2x \n", i, (uint8_t)data);
        c->ram[0xC000 + i] = (uint8_t)data;
    }
    fclose(f);
}

int main(int argc, char** argv)
{
    if (argc < 3 || argc > 3)
    {
        printf("Please run this command as follows: %s [runtype] [programname]\n", argv[0]);
        return 1;
    }

    cpu c = make_cpu();
    c.programcounter = 0xC000; // NOTE: For NESTEST!
    c.stackreg = 0xFD; // NOTE: For NESTEST!
    setstatus(&c, 0x24); // NOTE: For NESTEST!

    load_into_mem(&c, argv[2]); // Load in program.

    if (strncmp(argv[1], "step", 5) == 0)
        while (1)
        {
            print_cpu_state(&c);
            printf("\n");
            print_stack(&c, 10);
            getchar();
            execute(&c);
        }
    if (strncmp(argv[1], "steplog", 8) == 0)
        while (1)
        {
            log_cpu(&c, stdout);
            getchar();
            execute(&c);
        }
    if (strncmp(argv[1], "stacklog", 9) == 0)
        while (1)
        {
            log_cpu(&c, stdout);
            print_stack(&c, 0xFF);
            execute(&c);
        }
    if (strncmp(argv[1], "log", 4) == 0)
        while (1)
        {
            log_cpu(&c, stdout);
            execute(&c);
        }
    if (strncmp(argv[1], "run", 4) == 0)
        while (1)
            execute(&c);
    return 0;
}
