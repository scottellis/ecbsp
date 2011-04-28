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

	$ insmod ecbsp.ko
	$ echo start > /dev/ecbsp
	[  601.207519] Calling omap_start_dma
	[  601.210937] DMA misaligned error with device 17
	[  601.215515] ecbsp_dma_callback
	[  601.218566] Calling omap_start_dma
	[  601.222045] DMA misaligned error with device 17
	[  601.226623] ecbsp_dma_callback
	[  601.229675] Calling omap_start_dma
	[  601.233123] DMA misaligned error with device 17
	[  601.237670] ecbsp_dma_callback
	[  601.240753] ecbsp_mcbsp_stop


I have num motors set to 4.

When you run this, the CLKX runs and the FSX goes high, but no data and
no pulsing of the FSX. It goes low again when the stop is called which also
stops the CLKX.

I think it's just twiddling register flags to get it working. Need to do some
more research.

I'm not sure what I'm doing wrong with the DMA allocation.

