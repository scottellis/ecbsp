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

So we have basic functionality now. Once you load the driver you should immediately
start seeing activity on the CLKX line and FSX should be high. The clock should
be running at ~1MHz.

	root@overo:~# insmod ecbsp.ko 
	[ 1580.154907] ecbsp_mcbsp_request
	[ 1580.158081]     omap_mcbsp_request()
	[ 1580.161773] Initializing dma blocks
	[ 1580.165313] block[0] data ptr: dfeb0000  dma handle: 0x9FEB0000
	[ 1580.171264] ecbsp_mcbsp_start
	[ 1580.174255] ecbsp_set_mcbsp_config
	[ 1580.177703]     omap_mcbsp_set_tx_threshold()
	[ 1580.182067]     omap_mcbsp_config()
	[ 1580.185607]     omap_mcbsp_start()

If you then invoke the write function, you should see 16 pulses of the FSX line,
each time FSX is held low for 32 clocks representing the SPI CS signal held low
for a 32 bit data transfer. Between each 32-bit transfer the FSX line should go
high for 2 clock pulses. This is configurable. I just hard-coded 2.

	root@overo:~# echo write > /dev/ecbsp 
	[ 1588.560852] dma_channel = 1
	[ 1588.563659] ecbsp_mcbsp_dma_write(0)
	[ 1588.567260]     omap_set_dma_src_params()
	[ 1588.571350]     omap_start_dma()
	[ 1588.574615] DMA synchronization event drop occurred with device 17
	[ 1588.580810] ecbsp_dma_callback ch_status [CSR1]: 0x0022


I have num_motors set to 16, hence the 16 cycles for each write call. 


