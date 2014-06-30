#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>
#include <sys/mman.h>


// mux selection for port 0
#define HW_PINCTRL_MUXSEL0_SET 0x80018104
#define HW_PINCTRL_MUXSEL1_SET 0x80018114

// mux selection for port 1
#define HW_PINCTRL_MUXSEL2_SET 0x80018124
#define HW_PINCTRL_MUXSEL3_SET 0x80018134

// mux selection for port 2
#define HW_PINCTRL_MUXSEL4_SET 0x80018144
#define HW_PINCTRL_MUXSEL5_SET 0x80018154

// mux selection for port 3
#define HW_PINCTRL_MUXSEL6_SET 0x80018164
#define HW_PINCTRL_MUXSEL7_SET 0x80018174

// mux selection for port 4
#define HW_PINCTRL_MUXSEL8_SET 0x80018184
#define HW_PINCTRL_MUXSEL9_SET 0x80018194

// set muxes for pins that we care about
#define HW_PINCTRL_MUX_0 0x0000FFFF
#define HW_PINCTRL_MUX_1 0x03FFFFFF
#define HW_PINCTRL_MUX_2 0xFFFFFFFF
#define HW_PINCTRL_MUX_3 0xFFFFFFFF
#define HW_PINCTRL_MUX_4 0xFF00FF00
#define HW_PINCTRL_MUX_5 0x00FF0FFF
#define HW_PINCTRL_MUX_6 0xFFFFFF0F
#define HW_PINCTRL_MUX_7 0x3CFFFF3F
#define HW_PINCTRL_MUX_8 0xFFFFFFFF
#define HW_PINCTRL_MUX_9 0x00000303

#define DOUT0_IMX287_HI_LO 0x15550055
#define DOUT1_IMX287_HI_LO 0x55555555
#define DOUT2_IMX287_HI_LO 0x05555050
#define DOUT3_IMX287_HI_LO 0xA552AAA1
#define DOUT4_IMX287_HI_LO 0x00015555

// output enable register addresses
#define HW_PINCTRL_DOE0_SET		0x80018b04
#define HW_PINCTRL_DOE1_SET		0x80018b14
#define HW_PINCTRL_DOE2_SET		0x80018b24
#define HW_PINCTRL_DOE3_SET		0x80018b34
#define HW_PINCTRL_DOE4_SET		0x80018b44

#define HW_PINCTRL_DOE0_BITS 0x1FFF00FF
#define HW_PINCTRL_DOE1_BITS 0xFFFFFFFF
#define HW_PINCTRL_DOE2_BITS 0x0FFFF8F0
#define HW_PINCTRL_DOE3_BITS 0x6FF7FFF3
#define HW_PINCTRL_DOE4_BITS 0x0011FFFF

// output set/clear/toggle registers
#define HW_PINCTRL_DOUT0_SET	0x80018704		
#define HW_PINCTRL_DOUT1_SET	0x80018714		
#define HW_PINCTRL_DOUT2_SET	0x80018724		
#define HW_PINCTRL_DOUT3_SET	0x80018734		
#define HW_PINCTRL_DOUT4_SET	0x80018744		

#define HW_PINCTRL_DOUT0_CLR	0x80018708
#define HW_PINCTRL_DOUT1_CLR	0x80018718
#define HW_PINCTRL_DOUT2_CLR	0x80018728		
#define HW_PINCTRL_DOUT3_CLR	0x80018738		
#define HW_PINCTRL_DOUT4_CLR	0x80018748		

#define HW_PINCTRL_DOUT0_TOG	0x8001870C
#define HW_PINCTRL_DOUT1_TOG	0x8001871C
#define HW_PINCTRL_DOUT2_TOG	0x8001872C		
#define HW_PINCTRL_DOUT3_TOG	0x8001873C		
#define HW_PINCTRL_DOUT4_TOG	0x8001874C		

#define FATAL do { fprintf(stderr, "Error at line %d, file %s (%d) [%s]\n", __LINE__, __FILE__, errno, strerror(errno)); exit(1); } while(0)
 
#define MAP_SIZE 4096UL
#define MAP_MASK (MAP_SIZE - 1)

// prototypes
void imx287_port0_alternating_hi_lo();
void imx287_port1_alternating_hi_lo();
void imx287_port2_alternating_hi_lo();
void imx287_port3_alternating_hi_lo();
void imx287_port4_alternating_hi_lo();
void imx287_port0_toggle();
void imx287_port1_toggle();
void imx287_port2_toggle();
void imx287_port3_toggle();
void imx287_port4_toggle();
void create_persistent_memory_maps();
void do_hw_pinctrl_GPIO_set_all();
static inline void *fixup_addr(void *addr, size_t size);

// our future persistent memory maps
void *port0_data_out_set_mmap = 0;
void *port1_data_out_set_mmap = 0;
void *port2_data_out_set_mmap = 0;
void *port3_data_out_set_mmap = 0;
void *port4_data_out_set_mmap = 0;

void *port0_data_out_clr_mmap = 0;
void *port1_data_out_clr_mmap = 0;
void *port2_data_out_clr_mmap = 0;
void *port3_data_out_clr_mmap = 0;
void *port4_data_out_clr_mmap = 0;

void *port0_data_out_toggle_mmap = 0;
void *port1_data_out_toggle_mmap = 0;
void *port2_data_out_toggle_mmap = 0;
void *port3_data_out_toggle_mmap = 0;
void *port4_data_out_toggle_mmap = 0;

// global handle for /dev/mem 
int devmem_handle = 0;

// ************************************************* //

void sleep_ms(unsigned int ms)
{
	struct timespec req = {0};
	req.tv_sec = 0;
	req.tv_nsec = ms * 1000000L;
	nanosleep(&req, (struct timespec *)NULL);
	return;
}

// ************************************************* //

// configures all the port pins we can use for GPIO
void do_hw_pinctrl_GPIO_set_all()
{
    void *map_base, *virt_addr; 

    if((devmem_handle = open("/dev/mem", O_RDWR | O_SYNC)) == -1) FATAL;

    map_base = mmap(0, MAP_SIZE, PROT_WRITE, MAP_SHARED, devmem_handle, HW_PINCTRL_MUXSEL0_SET & ~MAP_MASK);
    if(map_base == (void *) -1) FATAL;

    virt_addr = (map_base + (HW_PINCTRL_MUXSEL0_SET & MAP_MASK));
	virt_addr = fixup_addr(virt_addr, sizeof(unsigned long));
	*((unsigned long *) virt_addr) = HW_PINCTRL_MUX_0;
	if(munmap(map_base, MAP_SIZE) == -1) FATAL;

    map_base = mmap(0, MAP_SIZE, PROT_WRITE, MAP_SHARED, devmem_handle, HW_PINCTRL_MUXSEL1_SET & ~MAP_MASK);
    if(map_base == (void *) -1) FATAL;

    virt_addr = (map_base + (HW_PINCTRL_MUXSEL1_SET & MAP_MASK));
	virt_addr = fixup_addr(virt_addr, sizeof(unsigned long));
	*((unsigned long *) virt_addr) = HW_PINCTRL_MUX_1;
	if(munmap(map_base, MAP_SIZE) == -1) FATAL;

    map_base = mmap(0, MAP_SIZE, PROT_WRITE, MAP_SHARED, devmem_handle, HW_PINCTRL_MUXSEL2_SET & ~MAP_MASK);
    if(map_base == (void *) -1) FATAL;

    virt_addr = (map_base + (HW_PINCTRL_MUXSEL2_SET & MAP_MASK));
	virt_addr = fixup_addr(virt_addr, sizeof(unsigned long));
	*((unsigned long *) virt_addr) = HW_PINCTRL_MUX_2;
	if(munmap(map_base, MAP_SIZE) == -1) FATAL;

    map_base = mmap(0, MAP_SIZE, PROT_WRITE, MAP_SHARED, devmem_handle, HW_PINCTRL_MUXSEL3_SET & ~MAP_MASK);
    if(map_base == (void *) -1) FATAL;

    virt_addr = (map_base + (HW_PINCTRL_MUXSEL3_SET & MAP_MASK));
	virt_addr = fixup_addr(virt_addr, sizeof(unsigned long));
	*((unsigned long *) virt_addr) = HW_PINCTRL_MUX_3;
	if(munmap(map_base, MAP_SIZE) == -1) FATAL;

    map_base = mmap(0, MAP_SIZE, PROT_WRITE, MAP_SHARED, devmem_handle, HW_PINCTRL_MUXSEL4_SET & ~MAP_MASK);
    if(map_base == (void *) -1) FATAL;

    virt_addr = (map_base + (HW_PINCTRL_MUXSEL4_SET & MAP_MASK));
	virt_addr = fixup_addr(virt_addr, sizeof(unsigned long));
	*((unsigned long *) virt_addr) = HW_PINCTRL_MUX_4;
	if(munmap(map_base, MAP_SIZE) == -1) FATAL;

    map_base = mmap(0, MAP_SIZE, PROT_WRITE, MAP_SHARED, devmem_handle, HW_PINCTRL_MUXSEL5_SET & ~MAP_MASK);
    if(map_base == (void *) -1) FATAL;

    virt_addr = (map_base + (HW_PINCTRL_MUXSEL5_SET & MAP_MASK));
	virt_addr = fixup_addr(virt_addr, sizeof(unsigned long));
	*((unsigned long *) virt_addr) = HW_PINCTRL_MUX_5;
	if(munmap(map_base, MAP_SIZE) == -1) FATAL;

    map_base = mmap(0, MAP_SIZE, PROT_WRITE, MAP_SHARED, devmem_handle, HW_PINCTRL_MUXSEL6_SET & ~MAP_MASK);
    if(map_base == (void *) -1) FATAL;

    virt_addr = (map_base + (HW_PINCTRL_MUXSEL6_SET & MAP_MASK));
	virt_addr = fixup_addr(virt_addr, sizeof(unsigned long));
	*((unsigned long *) virt_addr) = HW_PINCTRL_MUX_6;
	if(munmap(map_base, MAP_SIZE) == -1) FATAL;

    map_base = mmap(0, MAP_SIZE, PROT_WRITE, MAP_SHARED, devmem_handle, HW_PINCTRL_MUXSEL7_SET & ~MAP_MASK);
    if(map_base == (void *) -1) FATAL;

    virt_addr = (map_base + (HW_PINCTRL_MUXSEL7_SET & MAP_MASK));
	virt_addr = fixup_addr(virt_addr, sizeof(unsigned long));
	*((unsigned long *) virt_addr) = HW_PINCTRL_MUX_7;
	if(munmap(map_base, MAP_SIZE) == -1) FATAL;

    map_base = mmap(0, MAP_SIZE, PROT_WRITE, MAP_SHARED, devmem_handle, HW_PINCTRL_MUXSEL8_SET & ~MAP_MASK);
    if(map_base == (void *) -1) FATAL;

    virt_addr = (map_base + (HW_PINCTRL_MUXSEL8_SET & MAP_MASK));
	virt_addr = fixup_addr(virt_addr, sizeof(unsigned long));
	*((unsigned long *) virt_addr) = HW_PINCTRL_MUX_8;
	if(munmap(map_base, MAP_SIZE) == -1) FATAL;

    map_base = mmap(0, MAP_SIZE, PROT_WRITE, MAP_SHARED, devmem_handle, HW_PINCTRL_MUXSEL9_SET & ~MAP_MASK);
    if(map_base == (void *) -1) FATAL;

    virt_addr = (map_base + (HW_PINCTRL_MUXSEL9_SET & MAP_MASK));
	virt_addr = fixup_addr(virt_addr, sizeof(unsigned long));
	*((unsigned long *) virt_addr) = HW_PINCTRL_MUX_9;
	if(munmap(map_base, MAP_SIZE) == -1) FATAL;
}

// ************************************************* //

// create the memory maps for use later to set, clear, and toggle the pins
void create_persistent_memory_maps()
{
	port0_data_out_set_mmap = mmap(0, MAP_SIZE, PROT_WRITE, MAP_SHARED, devmem_handle, HW_PINCTRL_DOUT0_SET & ~MAP_MASK);
	if(port0_data_out_set_mmap == (void *) -1) FATAL;

	port1_data_out_set_mmap = mmap(0, MAP_SIZE, PROT_WRITE, MAP_SHARED, devmem_handle, HW_PINCTRL_DOUT1_SET & ~MAP_MASK);
   if(port1_data_out_set_mmap == (void *) -1) FATAL;

	port2_data_out_set_mmap = mmap(0, MAP_SIZE, PROT_WRITE, MAP_SHARED, devmem_handle, HW_PINCTRL_DOUT2_SET & ~MAP_MASK);
   if(port2_data_out_set_mmap == (void *) -1) FATAL;

	port3_data_out_set_mmap = mmap(0, MAP_SIZE, PROT_WRITE, MAP_SHARED, devmem_handle, HW_PINCTRL_DOUT3_SET & ~MAP_MASK);
   if(port3_data_out_set_mmap == (void *) -1) FATAL;

	port4_data_out_set_mmap = mmap(0, MAP_SIZE, PROT_WRITE, MAP_SHARED, devmem_handle, HW_PINCTRL_DOUT4_SET & ~MAP_MASK);
	if(port4_data_out_set_mmap == (void *) -1) FATAL;
	
	port0_data_out_clr_mmap = mmap(0, MAP_SIZE, PROT_WRITE, MAP_SHARED, devmem_handle, HW_PINCTRL_DOUT0_CLR & ~MAP_MASK);
	if(port0_data_out_clr_mmap == (void *) -1) FATAL;

	port1_data_out_clr_mmap = mmap(0, MAP_SIZE, PROT_WRITE, MAP_SHARED, devmem_handle, HW_PINCTRL_DOUT0_CLR & ~MAP_MASK);
   if(port1_data_out_clr_mmap == (void *) -1) FATAL;

	port2_data_out_clr_mmap = mmap(0, MAP_SIZE, PROT_WRITE, MAP_SHARED, devmem_handle, HW_PINCTRL_DOUT0_CLR & ~MAP_MASK);
   if(port2_data_out_clr_mmap == (void *) -1) FATAL;

	port3_data_out_clr_mmap = mmap(0, MAP_SIZE, PROT_WRITE, MAP_SHARED, devmem_handle, HW_PINCTRL_DOUT0_CLR & ~MAP_MASK);
   if(port3_data_out_clr_mmap == (void *) -1) FATAL;

	port4_data_out_clr_mmap = mmap(0, MAP_SIZE, PROT_WRITE, MAP_SHARED, devmem_handle, HW_PINCTRL_DOUT0_CLR & ~MAP_MASK);
	if(port4_data_out_clr_mmap == (void *) -1) FATAL;
	
	port0_data_out_toggle_mmap = mmap(0, MAP_SIZE, PROT_WRITE, MAP_SHARED, devmem_handle, HW_PINCTRL_DOUT0_TOG & ~MAP_MASK);
	if(port0_data_out_toggle_mmap == (void *) -1) FATAL;

	port1_data_out_toggle_mmap = mmap(0, MAP_SIZE, PROT_WRITE, MAP_SHARED, devmem_handle, HW_PINCTRL_DOUT1_TOG & ~MAP_MASK);
   if(port1_data_out_toggle_mmap == (void *) -1) FATAL;

	port2_data_out_toggle_mmap = mmap(0, MAP_SIZE, PROT_WRITE, MAP_SHARED, devmem_handle, HW_PINCTRL_DOUT2_TOG & ~MAP_MASK);
   if(port2_data_out_toggle_mmap == (void *) -1) FATAL;

	port3_data_out_toggle_mmap = mmap(0, MAP_SIZE, PROT_WRITE, MAP_SHARED, devmem_handle, HW_PINCTRL_DOUT3_TOG & ~MAP_MASK);
   if(port3_data_out_toggle_mmap == (void *) -1) FATAL;

	port4_data_out_toggle_mmap = mmap(0, MAP_SIZE, PROT_WRITE, MAP_SHARED, devmem_handle, HW_PINCTRL_DOUT4_TOG & ~MAP_MASK);
	if(port4_data_out_toggle_mmap == (void *) -1) FATAL;
	
}

// ************************************************* //

// enable output on the pins we have configured for GPIO
void do_hw_pinctrl_data_output_enable()
{
	void *map_base, *virt_addr; 

    if((devmem_handle = open("/dev/mem", O_RDWR | O_SYNC)) == -1) FATAL;

    map_base = mmap(0, MAP_SIZE, PROT_WRITE, MAP_SHARED, devmem_handle, HW_PINCTRL_DOE0_SET & ~MAP_MASK);
    if(map_base == (void *) -1) FATAL;

    virt_addr = (map_base + (HW_PINCTRL_DOE0_SET & MAP_MASK));
	virt_addr = fixup_addr(virt_addr, sizeof(unsigned long));
	*((unsigned long *) virt_addr) = HW_PINCTRL_DOE0_BITS;
	if(munmap(map_base, MAP_SIZE) == -1) FATAL;

    map_base = mmap(0, MAP_SIZE, PROT_WRITE, MAP_SHARED, devmem_handle, HW_PINCTRL_DOE1_SET & ~MAP_MASK);
    if(map_base == (void *) -1) FATAL;

    virt_addr = (map_base + (HW_PINCTRL_DOE1_SET & MAP_MASK));
	virt_addr = fixup_addr(virt_addr, sizeof(unsigned long));
	*((unsigned long *) virt_addr) = HW_PINCTRL_DOE1_BITS;
	if(munmap(map_base, MAP_SIZE) == -1) FATAL;

    map_base = mmap(0, MAP_SIZE, PROT_WRITE, MAP_SHARED, devmem_handle, HW_PINCTRL_DOE2_SET & ~MAP_MASK);
    if(map_base == (void *) -1) FATAL;

    virt_addr = map_base + (HW_PINCTRL_DOE2_SET & MAP_MASK);
	virt_addr = fixup_addr(virt_addr, sizeof(unsigned long));
	*((unsigned long *) virt_addr) = HW_PINCTRL_DOE2_BITS;
	if(munmap(map_base, MAP_SIZE) == -1) FATAL;

    map_base = mmap(0, MAP_SIZE, PROT_WRITE, MAP_SHARED, devmem_handle, HW_PINCTRL_DOE3_SET & ~MAP_MASK);
    if(map_base == (void *) -1) FATAL;

    virt_addr = (map_base + (HW_PINCTRL_DOE3_SET & MAP_MASK));
	virt_addr = fixup_addr(virt_addr, sizeof(unsigned long));
	*((unsigned long *) virt_addr) = HW_PINCTRL_DOE3_BITS;
	if(munmap(map_base, MAP_SIZE) == -1) FATAL;

    map_base = mmap(0, MAP_SIZE, PROT_WRITE, MAP_SHARED, devmem_handle, HW_PINCTRL_DOE4_SET & ~MAP_MASK);
    if(map_base == (void *) -1) FATAL;

    virt_addr = map_base + (HW_PINCTRL_DOE4_SET & MAP_MASK);
	virt_addr = fixup_addr(virt_addr, sizeof(unsigned long));
	*((unsigned long *) virt_addr) = HW_PINCTRL_DOE4_BITS;
	if(munmap(map_base, MAP_SIZE) == -1) FATAL;
}

// ************************************************* //

// clean up a bit
void unmap_memory()
{
	if(munmap(port0_data_out_set_mmap, MAP_SIZE) == -1) FATAL;
	if(munmap(port1_data_out_set_mmap, MAP_SIZE) == -1) FATAL;
	if(munmap(port2_data_out_set_mmap, MAP_SIZE) == -1) FATAL;
	if(munmap(port3_data_out_set_mmap, MAP_SIZE) == -1) FATAL;
	if(munmap(port4_data_out_set_mmap, MAP_SIZE) == -1) FATAL;
	if(munmap(port0_data_out_clr_mmap, MAP_SIZE) == -1) FATAL;
	if(munmap(port1_data_out_clr_mmap, MAP_SIZE) == -1) FATAL;
	if(munmap(port2_data_out_clr_mmap, MAP_SIZE) == -1) FATAL;
	if(munmap(port3_data_out_clr_mmap, MAP_SIZE) == -1) FATAL;
	if(munmap(port4_data_out_clr_mmap, MAP_SIZE) == -1) FATAL;
	if(munmap(port0_data_out_toggle_mmap, MAP_SIZE) == -1) FATAL;
	if(munmap(port1_data_out_toggle_mmap, MAP_SIZE) == -1) FATAL;
	if(munmap(port2_data_out_toggle_mmap, MAP_SIZE) == -1) FATAL;
	if(munmap(port3_data_out_toggle_mmap, MAP_SIZE) == -1) FATAL;
	if(munmap(port4_data_out_toggle_mmap, MAP_SIZE) == -1) FATAL;
}

// ************************************************* //

void imx287_port0_alternating_hi_lo()
{
	void *DOUT0 = port0_data_out_set_mmap + (HW_PINCTRL_DOUT0_SET & MAP_MASK);
	DOUT0 = fixup_addr(DOUT0, sizeof(unsigned long));
	*((unsigned long *) DOUT0) = DOUT0_IMX287_HI_LO;
}

// ************************************************* //

void imx287_port1_alternating_hi_lo()
{
	void *DOUT1 = port1_data_out_set_mmap + (HW_PINCTRL_DOUT1_SET & MAP_MASK);
	DOUT1 = fixup_addr(DOUT1, sizeof(unsigned long));
	*((unsigned long *) DOUT1) = DOUT1_IMX287_HI_LO;
}

// ************************************************* //

void imx287_port2_alternating_hi_lo()
{
	void *DOUT2 = port2_data_out_set_mmap + (HW_PINCTRL_DOUT2_SET & MAP_MASK);
	DOUT2 = fixup_addr(DOUT2, sizeof(unsigned long));
	*((unsigned long *) DOUT2) = DOUT2_IMX287_HI_LO;
}

// ************************************************* //

void imx287_port3_alternating_hi_lo()
{
	void *DOUT3 = port3_data_out_set_mmap + (HW_PINCTRL_DOUT3_SET & MAP_MASK);
	DOUT3 = fixup_addr(DOUT3, sizeof(unsigned long));
	*((unsigned long *) DOUT3) = DOUT3_IMX287_HI_LO;
}

// ************************************************* //

void imx287_port4_alternating_hi_lo()
{
	void *DOUT4 = port4_data_out_set_mmap + (HW_PINCTRL_DOUT4_SET & MAP_MASK);
	DOUT4 = fixup_addr(DOUT4, sizeof(unsigned long));
	*((unsigned long *) DOUT4) = DOUT4_IMX287_HI_LO;
}

// ************************************************* //

void imx287_port0_toggle()
{
	void *DOUT0 = port0_data_out_toggle_mmap + (HW_PINCTRL_DOUT0_TOG & MAP_MASK);
	DOUT0 = fixup_addr(DOUT0, sizeof(unsigned long));
	*((unsigned long *) DOUT0) = HW_PINCTRL_DOE0_BITS;
}

// ************************************************* //

void imx287_port1_toggle()
{
	void *DOUT1 = port1_data_out_toggle_mmap + (HW_PINCTRL_DOUT1_TOG & MAP_MASK);
	DOUT1 = fixup_addr(DOUT1, sizeof(unsigned long));
	*((unsigned long *) DOUT1) = HW_PINCTRL_DOE1_BITS;
}

// ************************************************* //

void imx287_port2_toggle()
{
	void *DOUT2 = port2_data_out_toggle_mmap + (HW_PINCTRL_DOUT2_TOG & MAP_MASK);
	DOUT2 = fixup_addr(DOUT2, sizeof(unsigned long));
	*((unsigned long *) DOUT2) = HW_PINCTRL_DOE2_BITS;
}

// ************************************************* //

void imx287_port3_toggle()
{
	void *DOUT3 = port3_data_out_toggle_mmap + (HW_PINCTRL_DOUT3_TOG & MAP_MASK);
	DOUT3 = fixup_addr(DOUT3, sizeof(unsigned long));
	*((unsigned long *) DOUT3) = HW_PINCTRL_DOE3_BITS;
}

// ************************************************* //

void imx287_port4_toggle()
{
	void *DOUT4 = port4_data_out_toggle_mmap + (HW_PINCTRL_DOUT4_TOG & MAP_MASK);
	DOUT4 = fixup_addr(DOUT4, sizeof(unsigned long));
	*((unsigned long *) DOUT4) = HW_PINCTRL_DOE4_BITS;
}

// ************************************************* //

void prep_dancedancedance()
{
	imx287_port0_alternating_hi_lo();
	imx287_port1_alternating_hi_lo();
	imx287_port2_alternating_hi_lo();
	imx287_port3_alternating_hi_lo();
	imx287_port4_alternating_hi_lo();
}

void dancedancedance()
{
	imx287_port0_toggle();
	imx287_port1_toggle();
	imx287_port2_toggle();
	imx287_port3_toggle();
	imx287_port4_toggle();
}

// ************************************************* //

int main()
{

	do_hw_pinctrl_data_output_enable();
	create_persistent_memory_maps();
	do_hw_pinctrl_GPIO_set_all();

	prep_dancedancedance();
	
	while(1)
	{
		dancedancedance();
		sleep_ms(100);
	}

	// unmap the memory for i/o
	unmap_memory();

    // close out /dev/mem
    close(devmem_handle);
	return 0;
}
// ************************************************* //

static inline void *fixup_addr(void *addr, size_t size)
{
#ifdef FORCE_STRICT_ALIGNMENT
	unsigned long aligned_addr = (unsigned long)addr;
	aligned_addr &= ~(size - 1);
	addr = (void *)aligned_addr;
#endif
	return addr;
}
