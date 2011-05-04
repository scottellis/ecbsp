  ecbsp
=======

Linux OMAP3 driver using the McBSP controller to operate some motors over a
SPI interface.

Overo expansion board 

	Pin	Signal
	27	mcbsp_clkx
	28	mcbsp_dr
	29	mcbsp_fsx
	30	mcbsp_dx


  u-boot
-------

The driver requires some pin mux changes. The recipe and mux patch are in the
patches directory. 

Copy the patch to 

	${OVEROTOP}/org.openembedded.dev/recipes/u-boot/u-boot-omap3-git/

and replace the recipe here

	${OVEROTOP}/org.openembedded.dev/recipes/u-boot/u-boot-omap3_git.bb


Then rebuild u-boot

	$ bitbake -c clean virtual/bootloader
	$ bitbake virtual/bootloader


There is also a kernel patch and modified 2.6.36 recipe if you want a little
more debug from the plat/mcbsp.c driver. It's not necessary.


  build
-------

There is file to source [overo-source-me.txt] that should properly set up
your paths. You may have to modify the OETMP variable if you are using a 
non-standard location. 

After that, the makefile should work.


  test
-------

It doesn't work yet. You can load the driver and call start, but you'll get
output like this.

	root@overo:~# insmod ecbsp.ko 
	[ 8589.576599] tx_reg [MCBSP3.DXR] = 0x49024008  dma_tx_sync = 17
	[ 8589.582550] Initializing dma blocks
	[ 8589.586090] block[0] data ptr: cfadb000  dma handle: 0x8FADB000

	root@overo:~# echo start > /dev/ecbsp 
	[ 8591.761169] dma_channel = 4
	[ 8591.764709] calling omap_start_dma
	[ 8591.768157] DMA misaligned error with device 17
	[ 8591.772705] ecbsp_dma_callback ch_status [CSR4]: 0x0800
	[ 8591.777954] ecbsp_mcbsp_stop


I have num_motors and NUM_DMA_BLOCKS set to 1 while I try to track down
this DMA misaligned error.

When you run this, the CLKX runs and the FSX goes high, but no data and
no pulsing of the FSX. It goes low again when the stop is called which also
stops the CLKX. I just took a WAG at the initial McBSP register config.

I'm not sure what I'm doing wrong with the DMA allocation. Can't really proceed
until that is fixed.


