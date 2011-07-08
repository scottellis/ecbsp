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

You can specify one module param for the number of motors per row

	motors=(16-2048 in multiples of 16, default is 256)

You can also change this with an ioctl.

There are some test programs in the user-progs directory.

Once the driver has received a 'queue_threshold' number of motor commands from
userland, it will start transmitting. The default frequency is 16 MHz.

The driver is still under development, but it mostly works for the portions
that are implemented. Hasn't got a lot of testing yet.


TODO

1. Need to work out how to handle the delay between cycles, in particular what
the true minimum requirement is so we can decide whether we need udelay or
can just use hrtimers. Also, do we need a McBSP TX callback to start the delay
timing more accurately. Right now we use the DMA callback which doesn't mean
the McBSP controller is done transmitting.

2. The user/driver interface is still not settled.


