// https://www.jmeiners.com/lc3-vm/

#include <stdio.h>
#include <stdint.h>
#include <signal.h>
/* unix only */
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/termios.h>
#include <sys/mman.h>

#define MEMORY_MAX (1 << 16) 

uint16_t memory[MEMORY_MAX];

// Registers
enum
{
	R_R0 = 0,
	R_R1,
	R_R2,
	R_R3,
	R_R4,
	R_R5,
	R_R6,
	R_R7,
	R_PC,
	R_COND,
	R_COUNT
};

uint16_t reg[R_COUNT];

// opcodes
enum
{
	OP_BR = 0, /* branch */
    OP_ADD,    /* add  */
    OP_LD,     /* load */
    OP_ST,     /* store */
    OP_JSR,    /* jump register */
    OP_AND,    /* bitwise and */
    OP_LDR,    /* load register */
    OP_STR,    /* store register */
    OP_RTI,    /* unused */
    OP_NOT,    /* bitwise not */
    OP_LDI,    /* load indirect */
    OP_STI,    /* store indirect */
    OP_JMP,    /* jump */
    OP_RES,    /* reserved (unused) */
    OP_LEA,    /* load effective address */
    OP_TRAP    /* execute trap */
};

// Conditions flags
enum
{
    FL_POS = 1 << 0, /* P */
    FL_ZRO = 1 << 1, /* Z */
    FL_NEG = 1 << 2, /* N */

};

// Trap codes
enum
{
    TRAP_GETC = 0x20,  /* get character from keyboard, not echoed onto the terminal */
    TRAP_OUT = 0x21,   /* output a character */
    TRAP_PUTS = 0x22,  /* output a word string */
    TRAP_IN = 0x23,    /* get character from keyboard, echoed onto the terminal */
    TRAP_PUTSP = 0x24, /* output a byte string */
    TRAP_HALT = 0x25   /* halt the program */
};

// Memory Mapped Registers
enum
{
    MR_KBSR = 0xFE00, /* keyboard status */
    MR_KBDR = 0xFE02  /* keyboard data */
};

void update_flags(uint16_t r){
    if (reg[r] == 0){
        reg[R_COND] = FL_ZRO;
    } else if (reg[r] >> 15){
        reg[R_COND] = FL_NEG;
    } else {
        reg[R_COND] = FL_POS;
    }
}

uint16_t swap16( uint16_t x ){
    /*
     * Convert big endian to little endian
     * So a big endian uint16_t might come in as
     * 01000010 00001100
     * 
     * We might think this means the opcode is 0100, etc etc
     *
     * Actually, this is incorrect as we are in the wrong byte order for our PC
     *
     * So when we bitshift x << 8 this string becomes
     * 00001100 00000000
     * And when we bitshift x >> 8 we get
     * 00000000 01000010
     *
     * Bitwise or puts this as
     * 00001100 01000010
     *
     * So we see our opcode is actually 0000 ( Bad example )
     */
    return (x << 8) | (x >> 8 );
} 


void read_image_file(FILE* file){
    // Origin tells is where in memory to start the image
    uint16_t origin;

    /*
     * When an LC3 assembly file is compiled into binary,
     * the first 2 bytes represent the origin
     */
    fread(&origin, sizeof(origin), 1, file);

    /*
     * LC3 is big endian. x86 systems are little endian. So when we
     * are reading any data from an LC3 binary file it will need to be
     * swapped around into little endian.
     */
    origin = swap16(origin);
     
    uint16_t max_read = MEMORY_MAX - origin;  // The space between the end of the memory and the start of the program
    uint16_t* p = memory + origin; // Pointer to the start of the program in memory

    /*
     * fread will write the file into the memory which "p" points to.
     * It will break the file into 16 bit chunks ( sizeof uint16_t ).
     * It will do this as many times as max_read
     * fread will return the number of items read
     *
     * ------ IMPORTANT -------
     *
     *  MEMORY_MAX ( defined at the start of the program ) isn't the number of bits the memory can hold.
     *  memory is defined as an array of uint16_t ( 16 bits ) with a size of MEMORY_MAX. Therefore, memory
     *  isn't the size of MEMORY_MAX bits, but MEMORY_MAX uint16_t. Therefore max_read isn't the number of bits
     *  between the origin & the end of memory. It is the number of 16 bit instructions we can hold in memory.
     */
    size_t read = fread( p, sizeof( uint16_t ), max_read, file );

    // Just iterating through the memory and converting the program to little endian 
    while (read-- > 0 ){
        /* Here we dereference pointer "p". So *p access the value at that memory location.
         * This is saying, swap the value stored at p around, and then replace the value stored at p
         * with this new value.
         */
        *p = swap16(*p);
        ++p;
    }
}

int read_image(const char* image_path){
    FILE* file = fopen(image_path, "rb");
    if (!file) { return 0; };
    read_image_file(file);
    fclose(file);
    return 1;
}

/*
 * imm5 is a 5 bit number. However, we need it to be a 16 bit number.
 * Therefore, we must pad it out to be 16 bits. We could just use 11 zeros,
 * but this will not work if the number is negative. Therefore, this function
 * will pad the number correctly to 16 bits while maintaining positive / negative.
 */
uint16_t sign_extend(uint16_t x, int bit_count)
{
    if ((x >> (bit_count - 1)) & 1) {
        x |= (0xFFFF << bit_count);
    }
    return x;
}

struct termios original_tio;

void disable_input_buffering()
{
    tcgetattr(STDIN_FILENO, &original_tio);
    struct termios new_tio = original_tio;
    new_tio.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);
}

void restore_input_buffering()
{
    tcsetattr(STDIN_FILENO, TCSANOW, &original_tio);
}

uint16_t check_key()
{
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(STDIN_FILENO, &readfds);

    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;
    return select(1, &readfds, NULL, NULL, &timeout) != 0;
}

void handle_interrupt(int signal)
{
    restore_input_buffering();
    printf("\n");
    exit(-2);
}

uint16_t mem_read(uint16_t address){
    if (address == MR_KBSR){
        if (check_key()){
            char c = getchar();

            memory[MR_KBSR] = (1 << 15);
            memory[MR_KBDR] = c;
        } else {
            memory[MR_KBSR] = 0;
        }
    }

    return memory[address];
}

void mem_write(uint16_t location, uint16_t value){
    memory[location] = value;
}

void operation_add(uint16_t instr){
	/*
	 * 0x7 = 0000 0000 0000 0111
	 * This does a bit-shift and then a bitwise AND operation.
	 * The bitwise AND is useful for isolating certain bits.
	 * For each bit, it returns 1 if both bits are 1, and 0 if not.
	 * So in our above example, instruction 0001 0101 0100 0100 is as follows:
	 * Bit-shift : 0000 0000 0000 1010
	 * Bitwise AND : 0000 0000 0000 0010
	 * r0 then is set to "2"
	 */
	uint16_t r0 = ( instr >> 9 ) & 0x7; // Interested in 0000 xxx0 0000 0000 ( Destination Register )
	uint16_t r1 = ( instr >> 6 ) & 0x7; // Interested in 0000 000x xx00 0000 ( SR1 - Address for register containing first number operand )
	uint16_t imm_flag = ( instr >> 5 ) & 0x1; // Intersted in 0000 0000 00x0 0000 ( 1 for a number operand, 0 for another register )

	if (imm_flag){
		/*
		 * 0x1F = 0000 0000 0001 1111
		 * imm5 ( Number operand ) is 5 bits long which is what we're finding here.
		 */
		uint16_t imm5 = sign_extend(instr & 0x1F, 5);
		reg[r0] = reg[r1] + imm5;
	} else {
		/*
		 * 0x7 = 0000 0000 0000 0111
		 * SR2 ( name of second register which contains the number to add ) is 3 bits long which is what we're
		 * finding here
		 */
		uint16_t r2 = instr & 0x7;
		reg[r0] = reg[r1] + reg[r2];
	}

	// Update the condition flag to denote whether the result is positive, negative or 0
	update_flags(r0);
}

void operation_and(uint16_t instr){
	// Desintation Register
	uint16_t r0 = (instr >> 9) & 0x7;
	uint16_t r1 = (instr >> 6) & 0x7;

	uint16_t imm_flag = (instr >> 5) & 0x1;

	if (imm_flag){
		// imm5
		uint16_t imm5 = sign_extend(instr & 0x1F, 5); // 0x1F
		reg[r0] = reg[r1] & imm5;
	} else {
		// SR2
		uint16_t r2 = instr & 0x7; //0000 0000 0000 0011
		reg[r0] = reg[r1] & reg[r2];
	}
	update_flags(r0);

}

void operation_ldi(uint16_t instr){
	uint16_t r0 = ( instr >> 9 ) & 0x7;
	uint16_t pc_offset = sign_extend(instr & 0x1FF, 9);

	reg[r0] = mem_read(mem_read(reg[R_PC] + pc_offset));

	update_flags(r0);
}

void operation_br(uint16_t instr){
	uint16_t cond_flag = (instr >> 9 ) & 0x7;

	if ( cond_flag & reg[R_COND] ){
		uint16_t pc_offset = sign_extend(instr & 0x1FF, 9);
		reg[R_PC] += pc_offset;
	}

}

void operation_jmp(uint16_t instr){
	uint16_t r1 = ( instr >> 6 ) & 0x7;
	reg[R_PC] = reg[r1];

}

void operation_jsr(uint16_t instr){
	uint16_t jmp_flag = ( instr >> 11 ) & 1;
	reg[R_R7] = reg[R_PC];
	if ( jmp_flag ){
		uint16_t pc_offset = sign_extend(instr & 0x7FF, 11); // 0000 0111 1111 1111 0x7FF
		reg[R_PC] += pc_offset; /* JSR */
	} else {
		uint16_t r1 = ( instr >> 6 ) & 0x7;
		reg[R_PC] += reg[r1]; /* JSRR */
	}

}

void operation_ld(uint16_t instr){
	uint16_t r0 = (instr >> 9) & 0x7; // Destination Register
	uint16_t pc_offset = sign_extend(instr & 0x1FF, 9);
	reg[r0] = mem_read(reg[R_PC]+pc_offset);
	update_flags(r0);
	
}

void operation_ldr(uint16_t instr){
	uint16_t r0 = (instr >> 9) & 0x7; // DR
	uint16_t r1 = (instr >> 6) & 0x7; //BR
	uint16_t offset = sign_extend(instr & 0x3F, 6); 

	reg[r0] = mem_read(reg[r1] + offset);
	update_flags(r0);

}

void operation_lea(uint16_t instr){
	uint16_t r0 = (instr >> 9) & 0x7; // DR
	uint16_t offset = sign_extend(instr & 0x1FF, 9);

	reg[r0] = reg[R_PC] + offset;

	update_flags(r0);
}

void operation_not(uint16_t instr){
	uint16_t r0 = (instr >> 9) & 0x7; // DR
	uint16_t r1 = (instr >> 6) & 0x7; // SR
	
	reg[r0] = ~reg[r1];

	update_flags(r0);
}

void operation_st(uint16_t instr){
	uint16_t r0 = (instr >> 9) & 0x7;
	uint16_t offset = sign_extend(instr & 0x1FF, 9);

	mem_write(reg[R_PC] + offset, reg[r0]);

}

void operation_sti(uint16_t instr){
	uint16_t r0 = (instr >> 9) & 0x7;
	uint16_t offset = sign_extend(instr & 0x1FF, 9);

	mem_write(mem_read(reg[R_PC] + offset), reg[r0]);
}

void operation_str(uint16_t instr){
	uint16_t r0 = (instr >> 9) & 0x7; // SR
	uint16_t r1 = (instr >> 6) & 0x7; // BaseR

	uint16_t offset = sign_extend(instr & 0x3F, 6); 

	mem_write(reg[r1] + offset, reg[r0]);
}


int main (int argc, const char* argv[]){

    // Load Arguments
    if (argc < 2){
        printf("lc3 [image-file1] ...\n");
        exit(2);
    }

    for ( int j = 1; j < argc; ++j ){
        if (!read_image(argv[j])){
            printf("failed to load image: %s\n", argv[j]);
            exit(1);
        }
    }

    // Setup
    signal(SIGINT, handle_interrupt);
    disable_input_buffering();

    // Set the condition flag to ZERO
    reg[R_COND] = FL_ZRO;
    

    // Set the program counter. Default is 0x3000 as per LC3 docs 
    enum { PC_START = 0x3000 };
    reg[R_PC] = PC_START;

    int running = 1;

    while (running){
        // Retrieve the instruction from the memory
        uint16_t instr = mem_read(reg[R_PC]++);

        /*
         * We bit-shift the instruction here to remove additional data.
         * In LC3, the first 4 bits of an instruction contain the opcode.
         * Add is opcode 1 ( 0001 in binary as opcode is 4bit ).
         * An add instruction might look like 0001 0101 0100 0100.
         */
        uint16_t op = instr >> 12;
        switch (op){
            case OP_ADD:
                operation_add(instr);
                break;
            case OP_AND:
                operation_and(instr);
                break;
            case OP_LDI:
                operation_ldi(instr);
                break;
            case OP_BR:
                operation_br(instr);
                break;
            case OP_JMP:
                operation_jmp(instr);
                break;
            case OP_JSR:
                operation_jsr(instr);
                break;
            case OP_LD:
                operation_ld(instr);
                break;
            case OP_LDR:
                operation_ldr(instr);
                break;
            case OP_LEA:
                operation_lea(instr);
                break;
            case OP_NOT:
                operation_not(instr);
                break;
            case OP_ST:
                operation_st(instr);
                break;
            case OP_STI:
                operation_sti(instr);
                break;
            case OP_STR:
                operation_str(instr);
                break;
            case OP_TRAP:
                reg[R_R7] = reg[R_PC];
                

                switch (instr & 0xFF){
                    case TRAP_GETC:
                    {
                        reg[R_R0] = (uint16_t)getchar();
                        update_flags(R_R0);
                    }
                    break;

                    case TRAP_IN:
                    {
                        printf("Please enter a character:");
                        char c = getchar();
                        putc(c, stdout);
                        fflush(stdout);
                        reg[R_R0] = (uint16_t)c;
                        update_flags(R_R0);
                    }
                    break;

                    case TRAP_OUT:
                    {
                        putc( (char)reg[R_R0], stdout );
                        fflush(stdout);
                    }
                    break;

                    case TRAP_HALT:
                    {
                        puts("Program exited...");
                        fflush(stdout);
                        running = 0;
                    }
                    break;

                    case TRAP_PUTS:
                    {
                        uint16_t* c = memory + reg[R_R0];
                        while (*c){
                            putc((char)*c, stdout);
                            ++c;
                        }
                        fflush(stdout);
                    }
                    break;

                    case TRAP_PUTSP:
                    {
                        uint16_t* c = memory + reg[R_R0];
                        while(*c) {
                            char char1 = (*c) & 0xFF;
                            putc(char1, stdout);

                            char char2 = ( *c ) >> 8;
                            if (char2){
                                putc( char2, stdout );
                            }
                            ++c;
                        }

                        fflush(stdout);
                    }
                    break;

                }
                break;
            case OP_RES:
            case OP_RTI:
            default:
                abort();
                break;
            }
    }

    restore_input_buffering();
}
