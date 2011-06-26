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

The [simple] branch now write a six 32-bit words with the frame-sync pulsed
between each 32-bit block. The six is hard coded as DEFAULT_NUM_MOTORS. I
tried other values and they work fine.


	root@overo:~# insmod ecbsp.ko 
	[ 7796.225463] ecbsp_mcbsp_request
	[ 7796.228637]     omap_mcbsp_request()
	[ 7796.232330]     tx_reg [MCBSP3.DXR] = 0x49024008  dma_tx_sync = 17
	[ 7796.238555] Initializing dma block
	[ 7796.242004]     data ptr: 0xdfe95000  dma handle: 0x9FE95000
	[ 7796.247711] ecbsp_mcbsp_start
	[ 7796.250701] ecbsp_set_mcbsp_config
	[ 7796.254119]     omap_mcbsp_set_tx_threshold()
	[ 7796.258514]     omap_mcbsp_config()
	[ 7796.262023]     omap_mcbsp_start()
	root@overo:~# echo write > /dev/ecbsp
	[ 7804.428131]     omap_request_dma()
	[ 7804.431549]     dma_channel = 0
	[ 7804.434722]     omap_set_dma_transfer_params()
	[ 7804.439270]     omap_set_dma_dest_params()
	[ 7804.443389] ecbsp_mcbsp_dma_write()
	[ 7804.446899]     omap_set_dma_src_params()
	[ 7804.450958]     omap_start_dma()
	[ 7804.454193] ecbsp_dma_callback ch_status [CSR0]: 0x0020


There is a o-scope screen shot, the blue is mcbsp_clkx and the red is mcbsp_fsx.

Just need to verify the data now.
