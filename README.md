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
be running at ~1MHz. You can specify num_motors 1-1024.

	root@overo:~# insmod ecbsp.ko num_motors=256
	[ 1345.906829] ecbsp_mcbsp_request
	[ 1345.910003]     omap_mcbsp_request()
	[ 1345.913696] Initializing dma blocks
	[ 1345.917236] block[0] data ptr: dfab8000  dma handle: 0x9FAB8000
	[ 1345.923217] ecbsp_mcbsp_start
	[ 1345.926177] ecbsp_set_mcbsp_config
	[ 1345.929626]     omap_mcbsp_set_tx_threshold()
	[ 1345.933990]     omap_mcbsp_config()
	[ 1345.937530]     omap_mcbsp_start()


If you then invoke the write function, you should see num_motor pulses of the 
FSX line. For each 32-bit transfer the FSX line is held low representing the 
SPI CS signal. Between each 32-bit transfer the FSX line should go high for 2 
clock pulses. This is configurable. I just hard-coded 2.

	root@overo:~# echo write > /dev/ecbsp 
	[ 1353.562133] dma_channel = 4
	[ 1353.564941] ecbsp_mcbsp_dma_write(0)
	[ 1353.568542]     omap_set_dma_src_params()
	[ 1353.572631]     omap_start_dma()
	[ 1353.575897] ecbsp_dma_callback ch_status [CSR4]: 0x0020
	root@overo:~# echo write > /dev/ecbsp 
	[ 1358.116210] ecbsp_mcbsp_dma_write(0)
	[ 1358.119812]     omap_set_dma_src_params()
	[ 1358.123901]     omap_start_dma()
	[ 1358.127166] ecbsp_dma_callback ch_status [CSR4]: 0x0020
	root@overo:~# echo write > /dev/ecbsp 
	[ 1362.053649] ecbsp_mcbsp_dma_write(0)
	[ 1362.057250]     omap_set_dma_src_params()
	[ 1362.061340]     omap_start_dma()
	[ 1362.064605] ecbsp_dma_callback ch_status [CSR4]: 0x0020

I have num_motors defaulting to 4 for easier scope debugging, but you can pass
in whatever you want.


