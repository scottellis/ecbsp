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

The [simple] branch just tries to do one 32-bit write with the frame-sync
line held low.


	root@overo:~# insmod ecbsp.ko 
	[   31.192413] ecbsp_mcbsp_request
	[   31.195648]     omap_mcbsp_request()
	[   31.199310]     tx_reg [MCBSP3.DXR] = 0x49024008  dma_tx_sync = 17
	[   31.205535] Initializing dma block
	[   31.208984]     data ptr: 0xdfe55000  dma handle: 0x9FE55000
	root@overo:~# echo start > /dev/ecbsp 
	[   43.983032] ecbsp_mcbsp_start
	[   43.986053] ecbsp_set_mcbsp_config
	[   43.989532]     omap_mcbsp_set_tx_threshold()
	[   43.993896]     omap_mcbsp_config()
	[   43.997436]     omap_request_dma()
	[   44.000854]     dma_channel = 1
	[   44.003997]     omap_set_dma_transfer_params()
	[   44.008483]     omap_set_dma_dest_params()
	[   44.012603] ecbsp_mcbsp_dma_write()
	[   44.016143]     omap_set_dma_src_params()
	[   44.020172]     omap_start_dma()
	[   44.023437]     omap_mcbsp_start()
	[   44.026855] ecbsp_dma_callback ch_status [CSR1]: 0x0020


There is a o-scope screen shot, the blue is mcbsp_clkx and the red is mcbsp_fsx.

Making progress, but still not working correctly.

